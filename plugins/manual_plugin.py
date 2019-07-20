# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import zmq
import sys
import asyncio
import time
import pickle

sys.path.append('..')
sys.path.append('../utils')
sys.path.append('../onboard')
import mixer
import zmq_wrapper 
import zmq_topics
import config
from config import Joy_map as jm



async def recv_and_process():
    keep_running=True
    joy_buttons=[0]*16
    yaw,pitch,roll=0,0,0
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])
            if topic==zmq_topics.topic_axes:
                #print('joy ',ret[jm.yaw])
                roll_copensate,pitch_copensate=0,0
                
                if joy_buttons[jm.shift2_bt]==1:
                    roll_copensate,pitch_copensate=roll,pitch
               
                if joy_buttons[jm.shift_bt]==0:
                    thruster_joy_cmd = mixer.mix(data[jm.ud],data[jm.lr],-data[jm.fb],0,0,data[jm.yaw],pitch_copensate,roll_copensate)
                else: #shift mode
                    thruster_joy_cmd = mixer.mix(data[jm.ud],0,0,data[jm.lr],-data[jm.fb],data[jm.yaw],pitch_copensate,roll_copensate)

                thrusters_source.send_pyobj(['joy',time.time(),thruster_joy_cmd])
            if topic==zmq_topics.topic_button:
                new_joy_buttons=data
                joy_buttons=new_joy_buttons
            if topic==zmq_topics.topic_imu:
                yaw,pitch,roll=data['yaw'],data['pitch'],data['roll']

        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    ### plugin inputs
    subs_socks=[]
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))

    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port) 

    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


