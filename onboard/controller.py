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
import config
from joy_mix import Joy_map

pub_sock = utils.publisher(zmq_topics.topic_controller_port)
subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button],zmq_topics.topic_joy_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_imu],zmq_topics.topic_imu_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))
thruster_sink = utils.pull_sink(zmq_topics.thrusters_sink_port)
subs_socks.append(thruster_sink)
printer_source = utils.push_source(zmq_topics.printer_sink_port)

def printer(txt,c=None):
    print('printing:',txt)
    printer_source.send_pyobj({'txt':txt,'c':c})



async def recv_and_process():
    keep_running=True
    thruster_cmd=np.zeros(8)
    timer10hz=time.time()+1/10.0
    timer20hz=time.time()+1/20.0
    system_state={'arm':False,'mode':[], 'lights':0, 'main_camera_servo':0 } #lights 0-5
    thrusters_dict={}
    last_axes_joy_message_time = 0 #keep alive time
    jm=Joy_map()
    THRSTR_LIMIT=config.thruster_limit_controler

    def togle_mode(m):
        s=system_state
        if m in s['mode']:
            s['mode'].remove(m)
        else:
            s['mode'].append(m)

    def test_togle(b):
        return new_joy_buttons[b]==1 and joy_buttons[b]==0

    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.002)[0]
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
                        if 'SONAR_HOLD' in system_state['mode']:
                            system_state['mode'].remove('SONAR_HOLD')
                    if jm.sonar_hold_event():
                        togle_mode('SONAR_HOLD')
                        if 'DEPTH_HOLD' in system_state['mode']:
                            system_state['mode'].remove('DEPTH_HOLD')
                    if jm.att_hold_event():
                        togle_mode('ATT_HOLD')
                    if jm.Rx_hold_event():
                        togle_mode('RX_HOLD')
                    if jm.Ry_hold_event():
                        togle_mode('RY_HOLD')
                    if jm.Rz_hold_event():
                        togle_mode('RZ_HOLD')
                    if jm.auto_nav_event():
                        togle_mode('AUTONAV')
                    if jm.cam_calib_event():
                        togle_mode('CAM_CALIB')
                    if jm.image_rect_event():
                        togle_mode('RECT')
                    if jm.arm_event():
                        system_state['arm']=not system_state['arm']
                        if not system_state['arm']:
                            system_state['mode']=[]

                if topic==zmq_topics.topic_remote_cmd:
                    print('got command',data)
                    if data['cmd']=='armdisarm':
                        system_state['arm']=not system_state['arm']
                        print('system state is:',system_state)
                        if not system_state['arm']:
                            system_state['mode']=[] 
            
                    if data['cmd']=='depth_hold':
                        togle_mode('DEPTH_HOLD')

                    if data['cmd']=='att_hold':
                        togle_mode('ATT_HOLD')

                    if data['cmd']=='heartbit':
                        last_axes_joy_message_time=time.time()

                    if data['cmd']=='x_hold':
                        togle_mode('RX_HOLD')

                    if data['cmd']=='y_hold':
                        togle_mode('RY_HOLD')

                    if data['cmd']=='z_hold':
                        togle_mode('RZ_HOLD')

                    if data['cmd']=='dvl_reset':
                        pub_sock.send_multipart([zmq_topics.topic_dvl_cmd,b'wcr\n'])
                        printer('controller:\n send dvl reset')

                    if data['cmd']=='dvl_calib':
                        pub_sock.send_multipart([zmq_topics.topic_dvl_cmd,b'wcg\n'])
                        printer('controller:\n send dvl calib')

                    if data['cmd']=='lights+':
                        system_state['lights']=min(5,system_state['lights']+1)
                        pub_sock.send_multipart([zmq_topics.topic_lights,pickle.dumps(system_state['lights'])])
                        printer(f"controller:\n lights+ {system_state['lights']}")

                    if data['cmd']=='lights-':
                        system_state['lights']=max(0,system_state['lights']-1)
                        pub_sock.send_multipart([zmq_topics.topic_lights,pickle.dumps(system_state['lights'])])
                        printer(f"controller:\n lights- {system_state['lights']}")

                    if data['cmd']=='gripper':
                        system_state['gripper']=data['val']
                        pub_sock.send_multipart([zmq_topics.topic_gripper_cmd,pickle.dumps(data['val'])])
                        printer(f"controller:\n gripper {system_state['gripper']}")

                    if data['cmd']=='thruster_limit':
                        THRSTR_LIMIT=data['value']

                           
                if topic==zmq_topics.topic_axes:
                    last_axes_joy_message_time=time.time()
                    jm.update_axis(data)
                    if jm.inc_lights_event():
                        system_state['lights']=min(5,system_state['lights']+1)
                        pub_sock.send_multipart([zmq_topics.topic_lights,pickle.dumps(system_state['lights'])])
                        print('lights set to',system_state['lights'])
                    if jm.dec_lights_event():
                        system_state['lights']=max(0,system_state['lights']-1)
                        pub_sock.send_multipart([zmq_topics.topic_lights,pickle.dumps(system_state['lights'])])
                        print('lights set to',system_state['lights'])
                    if jm.main_camera_down_event() or jm.main_camera_up_event():
                        system_state['main_camera_servo']=np.clip(system_state['main_camera_servo'] 
                                + (.01  if jm.main_camera_down_event() else -0.01 ) ,-1,1)
                        print('main camera servo',system_state['main_camera_servo'])
                        pub_sock.send_multipart([zmq_topics.topic_camera_servo,pickle.dumps(system_state['main_camera_servo'])])
                    if jm.dvl_reset_event():
                        pub_sock.send_multipart([zmq_topics.topic_dvl_cmd,b'wcr\n'])

                    if jm.gripper():
                        system_state['gripper']=jm.gripper()
                        pub_sock.send_multipart([zmq_topics.topic_gripper_cmd,pickle.dumps(jm.gripper())])
                        #print('gripper ', jm.gripper())





        tic=time.time()
        
        if tic-last_axes_joy_message_time>4.0: #when lost joy signal disarm
            system_state['arm']=False
        if tic-timer10hz>0:
            timer10hz=tic+1/10.0
            pub_sock.send_multipart([zmq_topics.topic_system_state,pickle.dumps((tic,system_state))])
        if tic-timer20hz>0:
            timer20hz=tic+1/40.0
            if not system_state['arm']:
                thruster_cmd=np.zeros(8)
            else:
                for k in thrusters_dict:
                    thruster_cmd += thrusters_dict[k]
            #thruster_cmd = 8*[0.043]
            thruster_cmd=np.clip(thruster_cmd,-THRSTR_LIMIT,THRSTR_LIMIT)
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




