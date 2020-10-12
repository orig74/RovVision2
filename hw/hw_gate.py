import numpy as np
import serial
import zmq
import sys
import asyncio
import time
import pickle
import struct
import os

sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
import zmq_topics
import detect_usb
import config

current_command=[0 for _ in range(8)] # 8 thrusters
keep_running=True

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port))

ser = serial.Serial(detect_usb.devmap['ESC_USB'], 115200)


def mcmnds_to_serialbuffer(cmnds):
    fmt = "<BbbbbbbbbB"
    serial_buff = [255]*10
    for idx, cmd in enumerate(cmnds):
        serial_buff[idx+1] = int(cmd * 127)
    return struct.pack(fmt, *serial_buff)


async def send_serial_command_50hz():
    while keep_running:
        await asyncio.sleep(1/50.0)

        rov_type = int(os.environ.get('ROV_TYPE','1'))
        m = [0]*8
        c=current_command
        if rov_type == 1:
            m[0]=c[5]
            m[1]=c[4]
            m[2]=c[6]
            m[3]=c[7]
            m[4]=c[1]
            m[5]=-c[0]
            m[6]=-c[2]
            m[7]=c[3]
        elif rov_type == 2:
            m[0]=c[6]
            m[1]=c[7]
            m[2]=-c[5]
            m[3]=-c[4]
            m[4]=c[2]
            m[5]=-c[3]
            m[6]=c[1]
            m[7]=-c[0]
        print(["%.1f"%i for i in m])
        m=np.clip(m,-config.thruster_limit,config.thruster_limit)
        s_buff = mcmnds_to_serialbuffer(m)
        ser.write(s_buff)
        ln = 'None'
        while ser.inWaiting():
            ln=ser.readline()
        if ln != 'None':
            print('>',ln)


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
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())
