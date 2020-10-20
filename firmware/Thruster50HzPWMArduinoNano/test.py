import numpy as np
import serial
import asyncio
import time
import struct

ser = serial.Serial('/dev/ttyUSB1', 115200)


def mcmnds_to_serialbuffer(cmnds):
    fmt = "<BbbbbbbbbB"
    serial_buff = [255]*10
    for idx, cmd in enumerate(cmnds):
        serial_buff[idx+1] = int(cmd * 127)
    return struct.pack(fmt, *serial_buff)


m = [-1]*8
sign = 1
while True:
    time.sleep(1/50.0)
    
    print(["%.1f"%i for i in m])
    m=np.clip(m,-1,1)
    s_buff = mcmnds_to_serialbuffer(m)
    ser.write(s_buff)

    m = [n + sign * 0.02 for n in m]
    if (m[0] <= -1 or m[0] >= 1):
        sign *= -1
    #m[-1] = 0.2

    ln = 'None'
    while ser.inWaiting():
        ln=ser.readline()
    if ln != 'None':
        print('>',ln)