# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import dill
import zmq
import sys
import asyncio
import time
import pickle

sys.path.append('..')
sys.path.append('unreal_proxy')
import utils
import zmq_topics
import ue4_zmq_topics

dill.settings['recurse'] = True
lamb=dill.load(open('lambda.pkl','rb'))
current_command=[0 for _ in range(8)] # 8 thrusters
dt=0.050
keep_running=True


#pub_pos_sim = utils.publisher(zmq_topics.topic_sitl_position_report_port)
pub_pos_sim = utils.publisher(ue4_zmq_topics.zmq_pub_drone_fdm[1])
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
        ps=position_struct
        print('----',curr_q)
        ps['posx'],ps['posy'],ps['posz']=curr_q[:3]
        ps['yaw'],ps['pitch'],ps['roll']=curr_q[3:]
        ps['pitch']+=00
        ps['roll']+=90
        #pub_pos_sim.send_multipart([xzmq_topics.topic_sitl_position_report,pickle.dumps((time.time(),curr_q))])
        pub_pos_sim.send_multipart([ue4_zmq_topics.topic_sitl_position_report,pickle.dumps(position_struct)])
        print('---',time.time(),curr_q) 

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
    asyncio.run(main())

