# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import zmq
import sys
import asyncio
import time
import pickle
import mixer


sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
import zmq_topics
import config
from config import Joy_map as jm

pub_sock = utils.publisher(zmq_topics.topic_controller_port)
subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))

keep_running=True
joy_buttons=[0]*8

async def recv_and_process():
    global current_command,joy_buttons,record_state
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.000)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            if ret[0]==zmq_topics.topic_axes:
                ret=pickle.loads(ret[1])
                #print('joy ',ret[jm.yaw])
                if joy_buttons[jm.shift_bt]==0:
                    thruster_cmd = mixer.mix(ret[jm.ud],ret[jm.lr],-ret[jm.fb],0,0,-ret[jm.yaw])
                else: #shift mode
                    thruster_cmd = mixer.mix(ret[jm.ud],0,0,ret[jm.lr],-ret[jm.fb],-ret[jm.yaw])
                pub_sock.send_multipart([zmq_topics.topic_thrusters_comand,pickle.dumps((time.time(),thruster_cmd))])

                #print('botton',ret)

        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())




