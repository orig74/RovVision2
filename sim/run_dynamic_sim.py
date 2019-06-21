# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import dill
import zmq
import sys
import asyncio
import time
import pickle

sys.path.append('..')
import utils
import zmq_topics


dill.settings['recurse'] = True
lamb=dill.load(open('lambda.pkl','rb'))

pub_pos_sim = utils.publisher(zmq_topics.topic_sitl_position_report_port)

def get_next_state(curr_q,curr_u,control,dt,lamb):
    forces=control
    u_dot_f=lamb(curr_q,curr_u,*forces).flatten()
    next_q=curr_q+curr_u*dt
    next_u=curr_u+u_dot_f*dt
    return next_q,next_u

current_command=[0 for _ in range(8)] # 8 thrusters
dt=0.050

async def pubposition():
    curr_q = np.zeros(6)
    curr_u = np.zeros(6)
    while True:
    #if 1: 
        await asyncio.sleep(dt)
        next_q,next_u=get_next_state(curr_q,curr_u,current_command,dt,lamb)
        next_q,next_u=next_q.flatten(),next_u.flatten()
        curr_q,curr_u=next_q,next_u
        pub_pos_sim.send_multipart([zmq_topics.topic_sitl_position_report,pickle.dumps((time.time(),curr_q))])
        #print('---',time.time()) 

async def main():
    # Schedule nested() to run soon concurrently
    # with "main()".
    task = asyncio.create_task(pubposition)

    # "task" can now be used to cancel "nested()", or
    # can simply be awaited to wait until it is complete:
    await task

asyncio.run(pubposition())

