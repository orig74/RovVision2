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
subs_socks.append(utils.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))
thruster_sink = utils.pull_sink(zmq_topics.thrusters_sink_port)
subs_socks.append(thruster_sink)


async def recv_and_process():
    keep_running=True
    joy_buttons=[0]*16
    thruster_cmd=np.zeros(8)
    timer10hz=time.time()+1/10.0
    timer20hz=time.time()+1/20.0
    system_state={'arm':False,'mode':'MANUAL'}
    thrusters_dict={}
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            if sock==thruster_sink:
                source,_,thruster_src_cmd=sock.recv_pyobj() 
                thrusters_dict[source]=thruster_src_cmd
            else:
                ret=sock.recv_multipart()
                topic,data=ret[0],pickle.loads(ret[1])
                if topic==zmq_topics.topic_button:
                    new_joy_buttons=data
                    #if new_joy_buttons[jm.record_bt]==1 and joy_buttons[jm.record_bt]==0:
                        #togel functions here
                    joy_buttons=new_joy_buttons
                    if joy_buttons[jm.arm_disarm]==1:
                        system_state['arm']=not system_state['arm']

        tic=time.time()
        if tic-timer10hz>0:
            timer10hz=tic+1/10.0
            pub_sock.send_multipart([zmq_topics.topic_system_state,pickle.dumps((tic,system_state))]) 
        if tic-timer20hz>0:
            timer20hz=tic+1/20.0
            if not system_state['arm']:
                thruster_cmd=np.zeros(8)
            for k in thrusters_dict:
                thruster_cmd += thrusters_dict[k]
            pub_sock.send_multipart([zmq_topics.topic_thrusters_comand,pickle.dumps((tic,list(thruster_cmd)))])
            thruster_cmd = np.zeros(8)


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




