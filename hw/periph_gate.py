import serial
import time
import numpy as np

ser = serial.Serial('/dev/ttyUSB0', 115200)

cmnd = 2
while cmnd < 7:
    cmnd = int(input("\nEnter value between 0-1 (cam trig ON/OFF), 2-7 (light level): "))
    ser.write([np.uint8(cmnd)])

while True:
    if ser.in_waiting >= 4:
        if ser.read(1)[0] == 255:
            periph_msg = ser.read(3)
            bar_D = float(periph_msg[0])/10
            batt_V = float(periph_msg[1])/10
            batt_I = float(periph_msg[2])/10
            print("Batt V: {}".format(batt_V))
            print("Batt I: {}".format(batt_I))
        else:
            print("Start byte not correct!")
            time.sleep(1)

