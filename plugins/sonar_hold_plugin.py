# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
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
from joy_mix import Joy_map
from config_pid import sonar_pid
from pid import PID
from filters import ab_filt

async def recv_and_process():
    keep_running=True
    pitch,roll=0,0
    target_range=0
    pid=None
    ab=None
    rate=0
    system_state={'mode':[]}
    jm=Joy_map()

    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])

            if topic==zmq_topics.topic_sonar:
                new_sonar_ts, range=data['ts'],data['sonar'][0]/1000    # Convert to m
                if ab is None:
                    ab=ab_filt([range,0])
                else:
                    depth,rate=ab(range,new_sonar_ts-sonar_ts)
                sonar_ts=new_sonar_ts

                if 'SONAR_HOLD' in system_state['mode']:
                    if pid is None:
                        pid=PID(**sonar_pid)
                    else:
                        ud_command = -pid(range,target_range,rate,0)
                        debug_pid = {'P':pid.p,'I':pid.i,'D':pid.d,'C':ud_command,'T':target_range,'N':range,'TS':new_sonar_ts}
                        pub_sock.send_multipart([zmq_topics.topic_sonar_hold_pid, pickle.dumps(debug_pid,-1)])
                        thruster_cmd = mixer.mix(ud_command,0,0,0,0,0,pitch,roll)
                        thrusters_source.send_pyobj(['sonar',time.time(),thruster_cmd])
                else:
                    if pid is not None:
                        pid.reset()
                    thrusters_source.send_pyobj(['sonar',time.time(),mixer.zero_cmd()])
                    target_range=range


            if topic==zmq_topics.topic_axes:
                jm.update_axis(data)
                target_range+=-jm.joy_mix()['ud']/25.0

            if topic==zmq_topics.topic_imu:
                pitch,roll=data['pitch'],data['roll']

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
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_sonar],zmq_topics.topic_sonar_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))

    ### plugin outputs
    thrusters_source = zmq_wrapper.push_source(zmq_topics.thrusters_sink_port)
    pub_sock = zmq_wrapper.publisher(zmq_topics.topic_sonar_hold_port)


    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())
