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
import gst
from config import Joy_map as jm

subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_stereo_camera],zmq_topics.topic_camera_port))
keep_running=True
joy_buttons=[0]*8

async def recv_and_process():
    global current_command,joy_buttons
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.000)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            if ret[0]==zmq_topics.topic_stereo_camera:
                frame_cnt,shape=pickle.loads(ret[1])
                imgl=np.frombuffer(ret[2],'uint8').reshape(shape).copy()
                imgr=np.frombuffer(ret[3],'uint8').reshape(shape).copy()
                image_enc_dec.encode(imgl,frame_cnt)
                image_enc_dec.encode(imgr,frame_cnt)
                gst.send_gst([imgl,imgr])

        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    gst.init_gst(config.cam_res_rgbx,config.cam_res_rgby,2)
    #asyncio.run(main())
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())




