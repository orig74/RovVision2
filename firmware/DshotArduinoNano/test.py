import numpy as np
import time
import struct
import serial


def motor_cmnd_to_DShot(cmnds):
    dshot_msgs = [0]*len(cmnds)
    for idx, cmd in enumerate(cmnds):
        zero_val = 1048
        if np.sign(cmd) == -1:
            zero_val = 48
        cmd_dshot = (zero_val + min(max(abs(round(cmd*999)), 0), 999)) << 1
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

ser = serial.Serial('/dev/ttyUSB1', 115200)

current_command = [0, 0, 0, 0, 0, 0, 0, 0]
dshot_frames = motor_cmnd_to_DShot(current_command)
s_buff_64 = dshotmsg_to_serialbuffer(dshot_frames)
ser.write(s_buff_64)
time.sleep(5)
itt = 0
while True:
    time.sleep(1)
    start_time = time.time()
    #thrust_val = float(input("\nEnter value between -1 and 1: "))
    #print(thrust_val)
    if itt % 2 == 0:
        current_command = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    else:
        current_command = [-0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1]
    itt += 1
    dshot_frames = motor_cmnd_to_DShot(current_command)
    s_buff_64 = dshotmsg_to_serialbuffer(dshot_frames)
    ser.write(s_buff_64)

    #print([struct.pack('>B', byte) for byte in s_buff_64])
    #print(time.time() - start_time)
