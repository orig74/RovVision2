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
#lamb=dill.load(open('../notebooks/lambda.pkl','rb'))
current_command=[0 for _ in range(8)] # 8 thrusters
dt=1/60.0
keep_running=True
savetofile=False
if savetofile:
    savefd=open(r'dynsim.txt','w')
#pub_pos_sim = utils.publisher(zmq_topics.topic_sitl_position_report_port)
pub_pos_sim = utils.publisher(ue4_zmq_topics.zmq_pub_drone_fdm[1])
pub_imu = utils.publisher(zmq_topics.topic_imu_port)
pub_depth = utils.publisher(zmq_topics.topic_depth_port)

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port))

position_struct={}

from scipy.interpolate import interp1d
pts_16v_t200 = [(1100,-4.07) , (1184, -2.94), (1308, -1.35) , (1472,0) , (1528,0), 
                (1624,0.87) , (1728,2.22), (1840,4.25), (1864,4.71), (1900,5.25)]
pwm,thrust = zip(*pts_16v_t200)
thrust=np.array(thrust)/0.101972 #kgf to newton
pwm_to_thrust=interp1d(pwm, thrust)
def scale_thrust(control):
    return np.array([pwm_to_thrust(c*400+1500)/10 for c in control])

def get_next_state(curr_q,curr_u,control,dt,lamb):
    control = np.clip(control,-1,1)
    forces=scale_thrust(control)
    currents_vector = [0,0.1,0]
    u_dot_f=lamb(curr_q,curr_u,*forces,*currents_vector).flatten()
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
        ps['yaw']=-ps['yaw']
        ps['posy']=-ps['posy']
        ps['pitch']=-ps['pitch']
        #ps['roll']=-ps['roll']
        #pub_pos_sim.send_multipart([xzmq_topics.topic_sitl_position_report,pickle.dumps((time.time(),curr_q))])
        pub_pos_sim.send_multipart([ue4_zmq_topics.topic_sitl_position_report,pickle.dumps(ps)])

        tic=time.time()
        imu={'ts':tic}
        imu['yaw'],imu['pitch'],imu['roll']=np.rad2deg(curr_q[3:])

        #rates from dsym notebook
        #R.ang_vel_in(R).express(R).to_matrix(R)
        #good video in https://www.youtube.com/watch?v=WZEFoWP0Tzs
        q3,q4,q5=curr_q[3:]
        u3,u4,u5=curr_u[3:]
        imu['rates']=(
                -u3*sin(q4) + u5,
                u3*sin(q5)*cos(q4) + u4*cos(q5),
                u3*cos(q4)*cos(q5) - u4*sin(q5))

        print('dsim Y{:4.2f} P{:4.2f} R{:4.2f}'.format(imu['yaw'],imu['pitch'],imu['roll'])
                +' X{:4.2f} Y{:4.2f} Z{:4.2f}'.format(*curr_q[:3]))
        if savetofile:
            print('dsim Y{:4.2f} P{:4.2f} R{:4.2f}'.format(imu['yaw'],imu['pitch'],imu['roll'])
                    +' X{:4.2f} Y{:4.2f} Z{:4.2f}'.format(*curr_q[:3]),file=savefd)
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

if 1 and __name__=='__main__':
#    asyncio.run(main())
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())

if 0 and __name__=='__main__':
    q,u=np.zeros(6),np.zeros(6)
    f=[1,1,-1,-1,0,0,0,0.0]
    for i in range(1000):
        q,u=get_next_state(q,u,f,dt,lamb)
        print(q)

