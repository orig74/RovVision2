import numpy as np
import serial
import zmq
import sys
import asyncio
import time
import pickle
import struct

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

def motor_cmnd_to_DShot(cmnds):
    dshot_msgs = [0]*len(cmnds)
    for idx, cmd in enumerate(cmnds):
        zero_val = 48
        if np.sign(cmd) >= 0.001:
            zero_val = 1048
        cmd_dshot = int(zero_val + min(max(round(abs(cmd)*999), 0), 999)) << 1
        csum = (cmd_dshot ^ (cmd_dshot >> 4) ^ (cmd_dshot >> 8)) & 0xf
        dshot_msgs[idx] = cmd_dshot << 4 | csum

    return dshot_msgs


def dshotmsg_to_serialbuffer(dshot_msg_l):
    serial_buff = [0]*17
    serial_buff[0] = 145    #start and code nibbles
    binary_message_list = [[0]*16 for i in range(len(dshot_msg_l))]
    for msg_indx, msg in enumerate(dshot_msg_l):
        for bit_indx, bit in enumerate(bin(msg)[2:][::-1]):
            binary_message_list[msg_indx][15 - bit_indx] = int(bit)
    frame_list = list(np.array(binary_message_list).transpose())
    for buff_indx in range(16):
        frame_byte = 0
        for bit in frame_list[buff_indx]:
            frame_byte = (frame_byte << 1) | bit
        serial_buff[buff_indx + 1] = frame_byte

    return serial_buff


async def send_serial_command_50hz():
    while keep_running:
        await asyncio.sleep(1/50.0)

        # Need to convert comands to list of -1 -> 1?
        m = [0]*8
        c=current_command
        m[0]=c[5]
        m[1]=c[4]
        m[2]=-c[6]
        m[3]=-c[7]
        m[4]=-c[1]
        m[5]=-c[0]
        m[6]=-c[2]
        m[7]=-c[3]

        m=np.clip(m,-config.thruster_limit,config.thruster_limit)
        dshot_frames = motor_cmnd_to_DShot(m)
        s_buff_64 = dshotmsg_to_serialbuffer(dshot_frames)
        ser.write(s_buff_64)
        #serial.write([struct.pack('>B', byte) for byte in s_buff_64])


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
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())
