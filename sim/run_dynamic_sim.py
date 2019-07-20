# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import dill
import zmq
import sys
import asyncio
import time
import pickle
from numpy import cos,sin

sys.path.append('..')
sys.path.append('../utils')
sys.path.append('unreal_proxy')
import zmq_wrapper as utils
import zmq_topics
import ue4_zmq_topics

dill.settings['recurse'] = True
lamb=dill.load(open('lambda.pkl','rb'))
current_command=[0 for _ in range(8)] # 8 thrusters
dt=1/60.0
keep_running=True


#pub_pos_sim = utils.publisher(zmq_topics.topic_sitl_position_report_port)
pub_pos_sim = utils.publisher(ue4_zmq_topics.zmq_pub_drone_fdm[1])
pub_imu = utils.publisher(zmq_topics.topic_imu_port)
pub_depth = utils.publisher(zmq_topics.topic_depth_port)

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port))

position_struct={}

def get_next_state(curr_q,curr_u,control,dt,lamb):
    forces=control
    u_dot_f=lamb(curr_q,curr_u,*forces).flatten()
    next_q=curr_q+curr_u*dt
    next_u=curr_u+u_dot_f*dt
    return next_q,next_u


async def pubposition():
    curr_q = np.zeros(6)
    curr_u = np.zeros(6)
    while keep_running:
        await asyncio.sleep(dt)
        next_q,next_u=get_next_state(curr_q,curr_u,current_command,dt,lamb)
        next_q,next_u=next_q.flatten(),next_u.flatten()
        curr_q,curr_u=next_q,next_u
        #next_u=next_u*.97
        ps=position_struct
        #print('dsim {:4.2f} {:4.2f} {:4.2f} {:3.1f} {:3.1f} {:3.1f}'.format(*curr_q),current_command)
        ps['posx'],ps['posy'],ps['posz']=curr_q[:3]
        ps['yaw'],ps['roll'],ps['pitch']=-np.rad2deg(curr_q[3:])
        ps['roll']+=90
        #pub_pos_sim.send_multipart([xzmq_topics.topic_sitl_position_report,pickle.dumps((time.time(),curr_q))])
        pub_pos_sim.send_multipart([ue4_zmq_topics.topic_sitl_position_report,pickle.dumps(position_struct)])

        tic=time.time()
        imu={'ts':tic}
        imu['yaw'],imu['pitch'],imu['roll']=-np.rad2deg(curr_q[3:])

        #rates from dsym notebook
        #print(R.ang_vel_in(N).express(N))
        #(-u4*sin(q3) + u5*cos(q3)*cos(q4))*N.x + (u4*cos(q3) + u5*sin(q3)*cos(q4))*N.y + (u3 - u5*sin(q4))*N.z
        q3,q4,q5=curr_q[3:]
        u3,u4,u5=curr_u[3:]
        imu['rates']=(\
                -u4*sin(q3) + u5*cos(q3)*cos(q4),\
                u4*cos(q3) + u5*sin(q3)*cos(q4),\
                u3 - u5*sin(q4))

        print('dsim Y{:4.2f} P{:4.2f} R{:4.2f}'.format(imu['yaw'],imu['pitch'],imu['roll'])
                +' X{:4.2f} Y{:4.2f} Z{:4.2f}'.format(*curr_q[:3]))
        pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])
        pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':curr_q[2]})])

async def recv_and_process():
    global current_command
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.000)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            if ret[0]==zmq_topics.topic_thrusters_comand:
                _,current_command=pickle.loads(ret[1])
        await asyncio.sleep(0.001)
        #print('-1-',time.time()) 

async def main():
    await asyncio.gather(
            recv_and_process(),
            pubposition(),
            )

if __name__=='__main__':
#    asyncio.run(main())
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())

