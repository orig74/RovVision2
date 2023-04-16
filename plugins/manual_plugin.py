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
from joy_mix import Joy_map  


async def recv_and_process():
    keep_running=True
    system_state={'mode':[]}
    
    jm=Joy_map()
    
    yaw,pitch,roll=0,0,0
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])
            
            if topic==zmq_topics.topic_system_state:
                _,system_state=data

            if topic==zmq_topics.topic_axes:
                #print('joy ',ret[jm.yaw])
                jm.update_axis(data)
                roll_copensate,pitch_copensate=0,0
                joy = jm.joy_mix() 
                if joy['inertial']:
                    roll_copensate,pitch_copensate=roll,pitch
              	
                #if 'ATT_HOLD' in system_state['mode']:
                #    thruster_joy_cmd = mixer.mix(joy['ud'],joy['lr'],joy['fb'],0, 0, 0,pitch_copensate,roll_copensate)
                #else:
                thruster_joy_cmd = mixer.mix(joy['ud'],joy['lr'],joy['fb'],joy['roll'],joy['pitch'],joy['yaw'],pitch_copensate,roll_copensate)
                thruster_joy_cmd_clipped = np.clip(thruster_joy_cmd, -config.manual_control_limit, config.manual_control_limit)
                thrusters_source.send_pyobj(['joy',time.time(),thruster_joy_cmd_clipped])

            if topic==zmq_topics.topic_button:
                jm.update_buttons(data)
            if topic==zmq_topics.topic_imu:
                yaw,pitch,roll=data['yaw'],data['pitch'],data['roll']

            if topic==zmq_topics.topic_remote_cmd:
                if data['cmd']=='manual_control_limit':
                    config.manual_control_limit=data['value']

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
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))

    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))
    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port) 

    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


