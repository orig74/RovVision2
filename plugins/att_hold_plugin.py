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
from config import Joy_map as jm
from config_pid import yaw_pid,pitch_pid,roll_pid
from pid import PID

#def rates_in_body_frame(yaw,pitch,roll,rot_rates):
#    dcm=mixer.todcm(yaw,pitch,roll)
#    v=np.array(rot_rates).reshape(3,1)
#    rx,ry,rz=(dcm.T @ v).flatten()
#    return rx,ry,rz

async def recv_and_process():
    keep_running=True
    target_att=np.zeros(3)
    pid_y,pid_p,pid_r=[None]*3
    system_state={'mode':[]}

    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])

            if topic==zmq_topics.topic_imu:
                yaw,pitch,roll=data['yaw'],data['pitch'],data['roll']
                ans=mixer.from_ang_rates_to_euler_rates(yaw,pitch,roll,data['rates'])
                if ans is not None:
                    yawr,pitchr,rollr=mixer.from_ang_rates_to_euler_rates(yaw,pitch,roll,data['rates']) 

                if 'ATT_HOLD' in system_state['mode'] and ans is not None:
                    if pid_y is None:
                        pid_y=PID(**yaw_pid)
                        pid_p=PID(**pitch_pid)
                        pid_r=PID(**roll_pid)
                    else:
                        yaw_cmd = pid_y(yaw,target_att[0],yawr,0)
                        #print('R{:06.3f} Y{:06.3f} YT{:06.3f} C{:06.3f}'.format(yawr,yaw,target_att[0],yaw_cmd))
                        pitch_cmd = pid_p(pitch,target_att[1],pitchr,0)
                        #print('R{:06.3f} P{:06.3f} PT{:06.3f} C{:06.3f}'.format(pitchr,pitch,target_att[1],pitch_cmd))
                        roll_cmd = pid_r(roll,target_att[2],rollr,0)
                        #print('RR{:06.3f} R{:06.3f} RT{:06.3f} C{:06.3f}'.format(rollr,roll,target_att[2],roll_cmd))

                        #debug_pid = {'P':pid.P,'I':pid.I,'D':pid.D,'C':ud_command,'T':target_depth,'N':depth,'TS':new_depth_ts}
                        #pub_sock.send_multipart([zmq_topics.topic_depth_hold_pid, pickle.dumps(debug_pid,-1)])
                        thruster_cmd = mixer.mix(0,0,0,roll_cmd,pitch_cmd,yaw_cmd,0,0)
                        thrusters_source.send_pyobj(['att',time.time(),thruster_cmd])
                else:
                    if pid_y is not None:
                        pid_y.reset(),pid_r.reset(),pid_y.reset()
                    target_att=[yaw,pitch,roll]
                    thrusters_source.send_pyobj(['att',time.time(),mixer.zero_cmd()])


            if topic==zmq_topics.topic_axes:
                pass
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
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes],zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))

    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port) 
    #pub_sock = zmq_wrapper.publisher(zmq_topics.topic_depth_hold_port)


    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


