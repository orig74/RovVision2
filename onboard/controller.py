# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import zmq
import sys
import asyncio
import time
import pickle

sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
import zmq_topics
from joy_mix import Joy_map

pub_sock = utils.publisher(zmq_topics.topic_controller_port)
subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))
thruster_sink = utils.pull_sink(zmq_topics.thrusters_sink_port)
subs_socks.append(thruster_sink)


async def recv_and_process():
    keep_running=True
    thruster_cmd=np.zeros(8)
    timer10hz=time.time()+1/10.0
    timer20hz=time.time()+1/20.0
    system_state={'arm':False,'mode':[], 'lights':0} #lights 0-5
    thrusters_dict={}

    jm=Joy_map()

    def togle_mode(m):
        s=system_state
        if m in s['mode']:
            s['mode'].remove(m)
        else:
            s['mode'].append(m)

    def test_togle(b):
        return new_joy_buttons[b]==1 and joy_buttons[b]==0

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
                    jm.update_buttons(data)
                    if jm.depth_hold_event():
                        togle_mode('DEPTH_HOLD')
                    if jm.att_hold_event():
                        togle_mode('ATT_HOLD')
                    if jm.Rx_hold_event():
                        togle_mode('RX_HOLD')
                    if jm.Ry_hold_event():
                        togle_mode('RY_HOLD')
                    if jm.Rz_hold_event():
                        togle_mode('RZ_HOLD')
                    if jm.arm_event():
                        system_state['arm']=not system_state['arm']
                if topic==zmq_topics.topic_axes:
                    jm.update_axis(data)
                    if jm.inc_lights_event():
                        system_state['lights']=min(5,system_state['lights']+1)
                        pub_sock.send_multipart([zmq_topics.topic_lights,pickle.dumps(system_state['lights'])])
                        print('lights set to',system_state['lights'])
                    if jm.dec_lights_event():
                        system_state['lights']=max(0,system_state['lights']-1)
                        pub_sock.send_multipart([zmq_topics.topic_lights,pickle.dumps(system_state['lights'])])
                        print('lights set to',system_state['lights'])



        tic=time.time()
        if tic-timer10hz>0:
            timer10hz=tic+1/10.0
            pub_sock.send_multipart([zmq_topics.topic_system_state,pickle.dumps((tic,system_state))]) 
        if tic-timer20hz>0:
            timer20hz=tic+1/20.0
            if not system_state['arm']:
                thruster_cmd=np.zeros(8)
            else:
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




