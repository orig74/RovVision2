# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import zmq
import sys
import asyncio
import time
import pickle
import numpy as np

sys.path.append('..')
sys.path.append('../utils')
sys.path.append('../onboard')
import mixer
import zmq_wrapper 
import zmq_topics
from joy_mix import Joy_map  
from config_pid import pos_pids
from pid import PID

async def recv_and_process():
    keep_running=True
    target_pos=np.zeros(3)
    pids=[None]*3
    system_state={'mode':[]}

    jm=Joy_map()
    joy=None

    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])

            if topic==zmq_topics.topic_imu:
                yaw,pitch,roll=data['yaw'],data['pitch'],data['roll']
        
           
            if topic==zmq_topics.topic_tracker:
                td=data
                cmds=[0]*3
                
                for ind in range(len(pids)):
                    #mapping from track image coords to world coords
                    imap = ['dx_f','dy_f','dz_f'][ind]
                    pid_states=['RX_HOLD','RY_HOLD','RZ_HOLD']
                    if pid_states[ind] not in system_state['mode'] \
                            or pids[ind] is None:
                        pids[ind] = PID(**pos_pids[ind])
                        if td['valid'] and imap in td:
                            x,_ = td[imap]
                            target_pos[ind]=x
                    else:
                        if td['valid'] and imap in td:
                            x,v = td[imap]
                            cmds[ind] = -pids[ind](x,target_pos[ind],v)
                            #print('pid=',ind,x,v,str(pids[ind]))
                            ts=time.time()
                            debug_pid = {'P':pids[ind].p,'I':pids[ind].i,'D':pids[ind].d,'C':cmds[ind],'T':target_pos[ind],'N':x, 'R':v, 'TS':ts}
                            pub_sock.send_multipart([zmq_topics.topic_pos_hold_pid_fmt%ind, pickle.dumps(debug_pid,-1)])
                
                thruster_cmd = mixer.mix(cmds[2],cmds[1],cmds[0],0,0,0,0,0)
                thrusters_source.send_pyobj(['pos',time.time(),thruster_cmd])

            if topic==zmq_topics.topic_axes:
                jm.update_axis(data)

            if topic==zmq_topics.topic_button:
                jm.update_buttons(data)
                #target_depth+=data[jm.ud] 

            if topic==zmq_topics.topic_system_state:
                _,system_state=data

        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    ### plugin inputs
    subs_socks=[]
    #subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes],zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_tracker],zmq_topics.topic_tracker_port))

    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port) 
    
    pub_sock = zmq_wrapper.publisher(zmq_topics.topic_pos_hold_port)


    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


