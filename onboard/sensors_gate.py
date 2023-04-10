# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#resposible for recording/preprossesing/distributing sensor data
import numpy as np
import zmq
import sys
import asyncio
import time
import pickle
import mixer


sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper
import zmq_topics
import image_enc_dec
import config
from utils import im16to8_22

if config.is_sim_zmq:
    print('using zmq to ground streamming...')
    sys.exit(0)

#import gst
import gst2

subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_stereo_camera],zmq_topics.topic_camera_port))
subs_socks.append(
    zmq_wrapper.subscribe([zmq_topics.topic_main_cam, zmq_topics.topic_main_cam_depth], zmq_topics.topic_main_cam_port)) #for sync perposes

keep_running=True
main_cam_gst=gst2.Writer(config.gst_cam_main_port,config.cam_main_sx,config.cam_main_sy)
main_cam_depth_gst=gst2.Writer(config.gst_cam_main_depth_port,config.cam_main_dgui_sx,config.cam_main_dgui_sy)
stereo_cam_writer=[gst2.Writer(config.gst_ports[i],config.cam_res_rgbx,config.cam_res_rgby,pad_lines=config.cam_res_gst_pad_lines) for i in [0,1]]

async def recv_and_process():
    global current_command
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            if ret[0]==zmq_topics.topic_stereo_camera:
                frame_cnt,shape=pickle.loads(ret[1])
                if frame_cnt%config.send_modulo==0:
                    imgl=np.frombuffer(ret[2],'uint8').reshape(shape).copy()
                    image_enc_dec.encode(imgl,frame_cnt)
                    togst = [imgl]
                    if config.camera_setup == 'stereo':
                        imgr=np.frombuffer(ret[3],'uint8').reshape(shape).copy()
                        image_enc_dec.encode(imgr,frame_cnt)
                        togst.append(imgr)
                    #print('hhhh,,sending to gst',frame_cnt)
                    #gst.send_gst(togst)
                    for i,im in enumerate(togst):
                        stereo_cam_writer[i].write(im)
            if ret[0]==zmq_topics.topic_main_cam:
                frame_main_cnt,shape = pickle.loads(ret[1])
                main_image=np.frombuffer(ret[2],'uint8').reshape(shape).copy()
                image_enc_dec.encode(main_image,frame_main_cnt)
                main_cam_gst.write(main_image)

            if ret[0]==zmq_topics.topic_main_cam_depth:
                frame_main_cnt,scale_to_mm,shape = pickle.loads(ret[1])
                image_depth=np.frombuffer(ret[2],'uint16').reshape(shape).astype('float32')*scale_to_mm
                main_image_depth=im16to8_22(image_depth).copy()
                #print(f'==== {scale_to_mm} {image_depth.max()},{image_depth.mean()}')

                image_enc_dec.encode(main_image_depth,frame_main_cnt)
                main_cam_depth_gst.write(main_image_depth)


        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    #gst.init_gst(config.cam_res_rgbx,config.cam_res_rgby,2 if config.camera_setup=='stereo' else 1)
    #asyncio.run(main())
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())




