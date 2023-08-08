# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import zmq
import sys
import asyncio
import time
import pickle
import traceback
import cv2

sys.path.append('..')
sys.path.append('../utils')
sys.path.append('../onboard')
import zmq_wrapper 
import zmq_topics
import config
from tracker.of import OF

from tracker.rope_detect import rope_detect_depth

cam_main_inv=np.linalg.inv(np.array(config.cam_main_int))
printer_source = zmq_wrapper.push_source(zmq_topics.printer_sink_port)

def printer(txt,c=None):
    print('printing:',txt)
    printer_source.send_pyobj({'txt':txt,'c':c})

crop_ltrb=(200,100,200,2)

async def recv_and_process():
    keep_running=True
    subs_socks=[]
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))

    vid_topics=[zmq_topics.topic_main_cam,zmq_topics.topic_main_cam_depth,zmq_topics.topic_main_cam_ts]
    subs_socks.append(zmq_wrapper.subscribe(vid_topics, zmq_topics.topic_main_cam_port)) #for sync perposes

    ### plugin outputs
    sock_pub=zmq_wrapper.publisher(zmq_topics.topic_main_tracker_port)
    im_gray=None
    of=OF()
    last_depth16=None
    scale_to_mm=0
    main_image=None
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            data = pickle.loads(ret[1]) if ret[0] not in vid_topics else None
            if main_image is not None and ret[0]==zmq_topics.topic_remote_cmd:
                if data['cmd']=='main_track':
                    print('got click',data['click_pt'])
                    pt=data['click_pt']
                    if pt is None:
                        im_gray=None
                        of.reset()
                        
                    if im_gray is not None:
                        x,y=pt
                        x=int(x*main_image.shape[1])-crop_ltrb[0]
                        y=int(y*main_image.shape[0])-crop_ltrb[1]
                        #x=int(x*im_gray.shape[1])
                        #y=int(y*im_gray.shape[0])
                        of.set(im_gray,(x,y))
                        print('setting track',x,y)
 
            if ret[0]==zmq_topics.topic_main_cam_ts:
                ts_fcnt,ts_depth,ts_color=pickle.loads(ret[1])

            if ret[0]==zmq_topics.topic_main_cam:
                frame_main_cnt,shape = pickle.loads(ret[1])
                main_image=np.frombuffer(ret[2],'uint8').reshape(shape)#.copy()
                rows,cols=main_image.shape[:2]
                #im_gray=cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
                l,t,r,b=crop_ltrb
                im_gray=cv2.cvtColor(main_image[t:-b,l:-r,:].copy(),cv2.COLOR_BGR2GRAY) 
                #im_gray=main_image[l:-r,t:-b,1].copy()
                
                _ret=of.track(im_gray)
                #_ret=None

                xw,yw,d=None,None,None
                if _ret is not None:
                    _ret=(_ret[0]+l,_ret[1]+t)
                    if last_depth16 is not None:
                        #print('ret===',_ret)
                        try:
                            d=last_depth16[int(_ret[1]),int(_ret[0])]*scale_to_mm*config.water_scale
                        except IndexError:
                            print('===error getting depth for ',_ret)
                            d=0

                        if config.valid_range_mm[0]<d<config.valid_range_mm[1]:
                            #xw,yw,s=(np.linalg.inv(np.array(config.cam_main_int)) @ np.array([[_ret[0]*d,_ret[1]*d,d]]).T).flatten()
                            xw,yw,s=(cam_main_inv @ np.array([[_ret[0]*d,_ret[1]*d,d]]).T).flatten()
                            #print('===+===',xw,yw,s,_ret)
                        else:
                            d=None
                    _ret=[_ret[0]/cols,_ret[1]/rows]

                res={'xy':_ret,'range':d,'left':xw,'up':yw}
                #print('returning: ',res)
                sock_pub.send_multipart([zmq_topics.topic_main_tracker,pickle.dumps(res)])

            if ret[0]==zmq_topics.topic_main_cam_depth:
                _,scale_to_mm,shape = pickle.loads(ret[1])
                last_depth16=np.frombuffer(ret[2],'uint16').reshape(shape)
                d,col,up_validation,down_validation,_=rope_detect_depth(last_depth16,scale_to_mm,config.water_scale)
                xw,yw,s=(cam_main_inv @ np.array([[col*d,0*d,d]]).T).flatten()
                down_valid = abs(d-down_validation) < config.range_validation_up_down_mm
                up_valid = abs(d-up_validation)<config.range_validation_up_down_mm
                range_valid= down_valid and up_valid
                rope_end = down_validation-d>config.down_validiation_diff_tresh_rope_detect_mm
                if rope_end:
                    printer(f'rope_end: {down_validation:.1f},{d:.1f}')
                range_valid = config.valid_range_mm[0]<d<config.valid_range_mm[1]
                track_valid = down_valid and up_valid and range_valid
                #import ipdb;ipdb.set_trace()
                #d,col,_=rope_detect_depth(scaled_d)
                res={'rope_col':col/last_depth16.shape[1],'range':s/1000,'dy':xw/1000,
                        'valid':track_valid, #'down_valid': down_valid,'up_valid': up_valid,
                        'rope_end_down':rope_end}
                #print('returning rope position: ',res)
                sock_pub.send_multipart([zmq_topics.topic_tracker,pickle.dumps(res)])

                

        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    ### plugin inputs
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())
