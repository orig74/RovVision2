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


async def motor_cmnd_to_DShot(cmnds):
    dshot_msgs = [0]*len(cmnds)
    for idx, cmd in enumerate(cmnds):
        zero_val = 1048
        if np.sign(cmd) == -1:
            zero_val = 48
        cmd_dshot = (zero_val + min(max(abs(round(cmd*999)), 0), 999)) << 1
        csum = (cmd_dshot ^ (cmd_dshot >> 4) ^ (cmd_dshot >> 8)) & 0xf
        dshot_msgs[idx] = cmd_dshot << 4 | csum

    return dshot_msgs


async def dshotmsg_to_serialbuffer(dshot_msg_l):
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
        dshot_frames = motor_cmnd_to_DShot(current_command)
        s_buff_64 = dshotmsg_to_serialbuffer(dshot_frames)
        serial.write(s_buff_64)
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
    asyncio.run(main())

