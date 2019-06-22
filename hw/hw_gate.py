import numpy as np
import serial
import zmq
import sys
import asyncio
import time
import pickle
import struct

sys.path.append('..')
import utils
import zmq_topics

current_command=[0 for _ in range(8)] # 8 thrusters
keep_running=True


subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port))



async def send_serial_command_50hz():
    while keep_running:
        await asyncio.sleep(1/50.0)
        #using struct.pack send serial command to arduino
        #serial.send....

### todo: add process to publish vector nav data???

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
            send_serial_command_50hz(),
            )

if __name__=='__main__':
    asyncio.run(main())

