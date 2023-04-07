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

printer_source = zmq_wrapper.push_source(zmq_topics.printer_sink_port)
def printer(txt,c=None):
    print('printing:',txt)
    printer_source.send_pyobj({'txt':txt,'c':c})


async def recv_and_process():
    keep_running=True
    subs_socks=[]
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))

    vid_topics=[zmq_topics.topic_main_cam,zmq_topics.topic_main_cam_depth]
    subs_socks.append(zmq_wrapper.subscribe(vid_topics+[zmq_topics.topic_main_cam_ts], zmq_topics.topic_main_cam_port)) #for sync perposes

    ### plugin outputs
    sock_pub=zmq_wrapper.publisher(zmq_topics.topic_main_tracker_port)
    im_gray=None
    of=OF()
    last_depth16=None
    scale_to_mm=0
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            data = pickle.loads(ret[1]) if ret[0] not in vid_topics else None
            if ret[0]==zmq_topics.topic_remote_cmd:
                if data['cmd']=='main_track':
                    print('got click',data['click_pt'])
                    x,y=data['click_pt']
                    if im_gray is not None:
                        x=int(x*im_gray.shape[1])
                        y=int(y*im_gray.shape[0])
                        of.set(im_gray,(x,y))
                        print('setting track',x,y)
 
            if ret[0]==zmq_topics.topic_main_cam:
                frame_main_cnt,shape = pickle.loads(ret[1])
                main_image=np.frombuffer(ret[2],'uint8').reshape(shape).copy()
                im_gray=cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
                _ret=of.track(im_gray)
                rows,cols=im_gray.shape

                xw,yw,d=0,0,0
                if _ret is not None:
                    if last_depth16 is not None:
                        print('ret===',_ret)
                        d=last_depth16[int(_ret[1]),int(_ret[0])]*scale_to_mm
                        xw,yw,s=(np.linalg.inv(np.array(config.cam_main_int)) @ np.array([[_ret[0]*d,_ret[1]*d,d]]).T).flatten()
                        print('===+===',xw,yw,s,_ret)
                        _ret=[\
                            _ret[0]/cols,
                            _ret[1]/rows]


                res={'xy':_ret,'range':d,'left':xw,'up':yw}
                #print('returning: ',ret)
                sock_pub.send_multipart([zmq_topics.topic_main_tracker,pickle.dumps(res)])

            if ret[0]==zmq_topics.topic_main_cam_depth:
                _,scale_to_mm,shape = pickle.loads(ret[1])
                last_depth16=np.frombuffer(ret[2],'uint16').reshape(shape)


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
