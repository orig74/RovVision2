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


AUTOSCAN_PATH = 10 * [np.array([10.0, 0.0, 0.0]),
                 np.array([0.0, 0.5, 0.0]),
                 np.array([-10.0, 0.0, 0.0]),
                 np.array([0.0, 0.5, 0.0])]
AUTO_VELOCITY = 0.2
AUTO_TARGET_THRESH = 0.1

def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def rotate_yaw(v,yaw):
    c,s = np.cos(yaw),np.sin(yaw)
    x,y = v[:2]
    x,y = x*c-y*s,x*s+y*c
    return np.array([x,y])
#deadreacon class
class DRxy(object):
    def __init__(self,yaw_rad=0):
        self.reset(yaw_rad)

    def reset(self,yaw_rad):
        self.last_v=None
        self.last_t=None
        self.x=None
        self._rel_yaw=yaw_rad

    def rel_yaw(self,yaw_rad):
        return normalize_angle(yaw_rad-self._rel_yaw)

    def __call__(self,t,v,yaw_rad):
        if self.x is None:
            self.x=np.zeros(2)
        else:
            va=(np.array(v)+np.array(self.last_v))/2
            dt=t-self.last_t
            self.x+=rotate_yaw(va*dt,-self.rel_yaw(yaw_rad))
        self.last_v=v
        self.last_t=t
        return self.x
D2R=np.radians
async def recv_and_process():
    keep_running=True
    target_pos=np.zeros(3)
    pids=[None]*2
    drs=DRxy() #dr on x, dr on y
    system_state={'mode':[]}

    jm=Joy_map()
    joy=None
    dvl_last_vel=None
    dvl_last_pos=None
    last_vel_report=time.time()
    tracker_lock_range=None
    Pxy=0.1
    yaw=0
    prev_target=None
    prev_ts=time.time()
    path_vec_idx=0
    dvl_internal_last_pos=None
    msg_cnt=0
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            msg_cnt+=1
            if ret[0]==zmq_topics.topic_dvl_cmd:
                print('got dvl command ',ret[1])
                if ret[1]==reset_cmd:
                    print('got dvl reset cmd')
                    dvl_last_pos=None
                    if dvl_last_pos is not None:
                        for ind in range(len(pids)):
                            pids[ind] = PID(**pos_pids[ind])
                    target_pos=np.zeros(3)
                    drs.reset(D2R(yaw))
                    printer(f'reset dvl {yaw}')
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

                if dd and dd['type']=='deadreacon':
                    dvl_internal_last_pos=dd

                if dd and dd['type']=='vel' and dd['valid']==b'y':
                    dvl_last_vel=dd
                    last_vel_report=time.time()
                    t=dd['tov']/1e6
                    vel_valid = abs(dd['vx'])<config.dvl_max_valid_speed
                    vel_valid = vel_valid and abs(dd['vy'])<config.dvl_max_valid_speed
                    if not vel_valid:
                        #mark as non valid
                        dd['valid']=b'n'
                        printer(f"{msg_cnt}: dvl vel err {dd['vx']:.3f},{dd['vy']:.3f}")
                    #else:
                    #    printer(f"{msg_cnt}: dvl vel good {dd['vx']:.3f},{dd['vy']:.3f}")

                    xy=drs(t,(dd['vx'],dd['vy']) if vel_valid else (0,0),D2R(yaw))
                    #external dvl position calculation based on vel only
                    dvl_last_pos  = {'x':xy[0],'y':xy[1],'yaw':D2R(yaw)}
                    #print('++++',dvl_last_pos,t,(dd['vx'],dd['vy']),yaw)
                    #print('===',dvl_internal_last_pos,dvl_last_pos)

                if dvl_last_pos and dvl_last_vel:
                    cmds=[0]*3
                    for ind in range(len(pids)):
                        pid_states=['RX_HOLD','RY_HOLD','RZ_HOLD']
                        is_override = False
                        if ind<2: #only apply for x and y hold
                            is_override = abs(jm.joy_mix()[('fb','lr')[ind]])>0.03 

                        mod_active = pid_states[ind] in system_state['mode']
                        is_autonav = 'AUTONAV' in system_state['mode'] 
                        #if pid_states[ind] not in system_state['mode'] and 'AUTONAV' not in system_state['mode'] \
                        #        or pids[ind] is None or \
                        #        (pid_states[ind]=='RX_HOLD' and abs(jm.joy_mix()['fb'])>0.1 and 'AUTONAV' not in system_state['mode']) or \
                        #        (pid_states[ind]=='RY_HOLD' and abs(jm.joy_mix()['lr'])>0.1 and 'AUTONAV' not in system_state['mode']):
                        
                        #reset pid
                        if not mod_active or is_override:
                            pids[ind] = PID(**pos_pids[ind])
                            fname='xyz'[ind]+'_hold_pid.json'
                            if os.path.isfile(fname):
                                pids[ind].load(fname)
                            target_pos[ind]=dvl_last_pos['xyz'[ind]]
                        elif dvl_last_vel['valid']==b'y':
                            x,v=dvl_last_pos['xyz'[ind]],dvl_last_vel['v'+'xyz'[ind]]
                            tetha = -np.radians(dvl_last_pos['yaw'])
                            c=np.cos(tetha)
                            s=np.sin(tetha)
                            if ind==0: #xrot #should do it more cleanly
                                x=dvl_last_pos['x']*c-dvl_last_pos['y']*s
                                v=dvl_last_vel['vx']#*c-dvl_last_vel['vy']*s
                            if ind==1: #yrot
                                x=dvl_last_pos['x']*s+dvl_last_pos['y']*c
                                v=dvl_last_vel['vy']

                            cmds[ind] = -pids[ind](x,target_pos[ind],v)
                            ts=time.time()
                            debug_pid = {'P':pids[ind].p,'I':pids[ind].i,'D':pids[ind].d,'C':cmds[ind],'T':target_pos[ind],'N':x, 'R':v, 'TS':ts}
                            pub_sock.send_multipart([zmq_topics.topic_pos_hold_pid_fmt%ind, pickle.dumps(debug_pid,-1)])
                    thruster_cmd = mixer.mix(cmds[2],-cmds[1],-cmds[0],0,0,0,0,0)
                    thrusters_source.send_pyobj(['pos',time.time(),thruster_cmd])

            if topic==zmq_topics.topic_tracker:
                if data['valid'] and tracker_lock_range is not None:
                    dx=tracker_lock_range-data['range']
                    #printer(f">>>dx={dx:.2f},{tracker_lock_range:.2f},{data['range']:.2f}")
                    dy=data['dy']
                    #target_pos[0]=dvl_last_pos['x']-Pxy*dx
                    #target_pos[1]=dvl_last_pos['y']+Pxy*dy
                    target_pos[0]-=Pxy*dx
                    target_pos[1]+=Pxy*dy

            if topic==zmq_topics.topic_main_tracker:
                if data['range']:
                    rng,left,up=config.grip_pos_rel_mm
                    dx=np.clip((rng-data['range']),-10,10)/1000 #mm to m
                    dy=-(left-data['left'])/1000
                    #dz=up-data['up']
                    target_pos[0]-=Pxy*dx
                    target_pos[1]+=Pxy*dy


            if topic==zmq_topics.topic_remote_cmd:
                if data['cmd']=='go':
                    pt=data['point']
                    #pt=rotate_yaw(pt,D2R(yaw))
                    pt=np.array([pt[0],pt[1],0])
                    printer(f'-go {data} {yaw}')
                    target_pos = target_pos + pt if data['rel'] else pt

                if data['cmd']=='tracker_vert_object_lock':
                    tracker_lock_range = data['range']
                    Pxy=data['Pxy']

                if data['cmd']=='Pxy update':
                    Pxy=data['Pxy']
                    printer(f'Pxy changed to {Pxy}')
                
                if data['cmd']=='tracker_vert_object_unlock':
                    tracker_lock_range = None

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

            # Autoscan position interpolation
            if 'AUTONAV' in system_state['mode']:
                dt = time.time() - prev_ts
                prev_ts = time.time()
                cur_path_vec = AUTOSCAN_PATH[path_vec_idx]
                vec_length = np.linalg.norm(cur_path_vec)
                target_pos += dt * AUTO_VELOCITY * cur_path_vec / vec_length
                target_err = prev_target + cur_path_vec - target_pos
                if np.linalg.norm(target_err) < AUTO_TARGET_THRESH:
                    print(path_vec_idx)
                    prev_target += cur_path_vec
                    path_vec_idx += 1
                    if path_vec_idx == len(AUTOSCAN_PATH):
                        path_vec_idx = 0
            else:
                path_vec_idx = 0
                prev_ts = time.time()
                prev_target = target_pos.copy()

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
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_tracker],zmq_topics.topic_tracker_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_main_tracker,zmq_topics.topic_tracker],zmq_topics.topic_main_tracker_port))

    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port) 
    printer_source = zmq_wrapper.push_source(zmq_topics.printer_sink_port)
    
    pub_sock = zmq_wrapper.publisher(zmq_topics.topic_pos_hold_port)


    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


