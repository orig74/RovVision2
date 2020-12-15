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

subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_stereo_camera],zmq_topics.topic_camera_port))
keep_running=True

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
                    gst.send_gst(togst)

        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    gst.init_gst(config.cam_res_rgbx,config.cam_res_rgby,2 if config.camera_setup=='stereo' else 1)
    #asyncio.run(main())
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())




