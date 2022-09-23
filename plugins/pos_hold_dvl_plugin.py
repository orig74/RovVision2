# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import zmq
import sys
import asyncio
import time
import pickle
import numpy as np
import os

sys.path.append('..')
sys.path.append('../utils')
sys.path.append('../onboard')
sys.path.append('../hw')
import config
import mixer
import zmq_wrapper 
import zmq_topics
from dvl import parse_line,reset_cmd
from joy_mix import Joy_map  
from config_pid_dvl import pos_pids
from pid import PID

async def recv_and_process():
    keep_running=True
    target_pos=np.zeros(3)
    pids=[None]*3
    system_state={'mode':[]}

    jm=Joy_map()
    joy=None
    dvl_last_vel=None
    dvl_last_pos=None
    last_vel_report=time.time()

    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            if ret[0]==zmq_topics.topic_dvl_cmd:
                print('got dvl command ',ret[1])
                if ret[1]==reset_cmd:
                    print('got dvl reset cmd')
                    if dvl_last_pos is not None:
                        for ind in range(len(pids)):
                            pids[ind] = PID(**pos_pids[ind])
                            target_pos=np.zeros(3)
                continue

            try:
                topic,data=ret[0],pickle.loads(ret[1])
            except Exception as E:
                print('error in pickle','ret: ',ret)
                continue

            if topic==zmq_topics.topic_imu:
                yaw,pitch,roll=data['yaw'],data['pitch'],data['roll']


            if topic==zmq_topics.topic_dvl_raw:
                dd=parse_line(data['dvl_raw'])
                if dd['type']=='deadreacon':
                    dvl_last_pos=dd
                if dd['type']=='vel' and dd['valid']==b'y':
                    dvl_last_vel=dd
                    last_vel_report=time.time()
                if dvl_last_pos and dvl_last_vel:
                    cmds=[0]*3
                    for ind in range(len(pids)):
                        pid_states=['RX_HOLD','RY_HOLD','RZ_HOLD']
                        if pid_states[ind] not in system_state['mode'] \
                                or pids[ind] is None or \
                                (pid_states[ind]=='RX_HOLD' and abs(jm.joy_mix()['fb'])>0.1) or \
                                (pid_states[ind]=='RY_HOLD' and abs(jm.joy_mix()['lr'])>0.1):
                            pids[ind] = PID(**pos_pids[ind])
                            fname='xyz'[ind]+'_hold_pid.json'
                            if os.path.isfile(fname):
                                pids[ind].load(fname)
                            target_pos[ind]=dvl_last_pos['xyz'[ind]]
                        elif dvl_last_vel['valid']==b'y':
                            x,v=dvl_last_pos['xyz'[ind]],dvl_last_vel['v'+'xyz'[ind]]
                            cmds[ind] = -pids[ind](x,target_pos[ind],v)
                            ts=time.time()
                            debug_pid = {'P':pids[ind].p,'I':pids[ind].i,'D':pids[ind].d,'C':cmds[ind],'T':target_pos[ind],'N':x, 'R':v, 'TS':ts}
                            pub_sock.send_multipart([zmq_topics.topic_pos_hold_pid_fmt%ind, pickle.dumps(debug_pid,-1)])
                    
                    thruster_cmd = mixer.mix(cmds[2],-cmds[1],-cmds[0],0,0,0,0,0)
                    thrusters_source.send_pyobj(['pos',time.time(),thruster_cmd])

            if topic==zmq_topics.topic_remote_cmd:
                if data['cmd']=='go':
                    target_pos = target_pos + data['point'] if data['rel'] else np.array(data['point'])
                if data['cmd']=='exec' and data['script']==os.path.basename(__file__):
                    try:
                        exec(data['torun'])
                    except Exception as E:
                        print('Error in exec command: ',E,data['torun'])



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

def printer(txt,c=None):
    print('printing:',txt)
    printer_source.send_pyobj({'txt':txt,'c':c})


if __name__=='__main__':
    ### plugin inputs
    subs_socks=[]
    #subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes],zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_dvl_cmd],zmq_topics.topic_controller_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_dvl_raw],zmq_topics.topic_dvl_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))

    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port) 
    printer_source = zmq_wrapper.push_source(zmq_topics.printer_sink_port)
    
    pub_sock = zmq_wrapper.publisher(zmq_topics.topic_pos_hold_port)


    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


