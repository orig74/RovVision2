import serial
import time
import numpy as np
import sys
import pickle
import zmq
sys.path.append('../utils')
sys.path.append('../')
import detect_usb
import zmq_wrapper
import zmq_topics
pub_depth = zmq_wrapper.publisher(zmq_topics.topic_depth_port) 
pub_volt = zmq_wrapper.publisher(zmq_topics.topic_volt_port)
ser = serial.Serial(detect_usb.devmap['PERI_USB'], 115200)


BATT_AMP_OFFSET = 0.330
BATT_AMP_PERVOLT = 37.8788
ADC_VOLTAGE_MUL = 0.0046
BATT_VOLTAGE_MULT = 11


print('connected to ', detect_usb.devmap['PERI_USB'])
subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_lights],zmq_topics.topic_controller_port))
# cmnd = 2

# while cmnd < 7:
#    cmnd = int(input("\nEnter value between 0-1 (cam trig ON/OFF), 2-7 (light level): "))
#   ser.write([np.uint8(cmnd)])

# while True:
#     if ser.in_waiting >= 4:
#         if ser.read(1)[0] == 255:
#             periph_msg = ser.read(3)
#             bar_D = float(periph_msg[0])/10
#             batt_V = float(periph_msg[1])/10
#             batt_I = float(periph_msg[2])/10
#             print("Batt V: {}".format(batt_V))
#             print("Batt I: {}".format(batt_I))
#         else:
#             print("Start byte not correct!")
#             time.sleep(1)

#start triggering
ser.write(b'\x01')
ser.flush()
print('trigger sent')

while True:
    socks=zmq.select(subs_socks,[],[],0.005)[0]
    for sock in socks:
        ret=sock.recv_multipart()
        topic,data=ret[0],pickle.loads(ret[1])
        if topic==zmq_topics.topic_lights:
            print('got lights command',data)
            ser.write(b'%c'%(data+2))  
            #ser.flush()

    if ser.in_waiting >= 7:
        if ser.read(1)[0] == 255:
            periph_msg = ser.read(6)
            tic=time.time()
            bar_D = float(periph_msg[0] | periph_msg[1] << 8) / 200
            pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':bar_D})])

            adc_V = round(float(periph_msg[2] | periph_msg[3] << 8), 2)
            batt_V = adc_V * ADC_VOLTAGE_MUL * BATT_VOLTAGE_MULT
            adc_I = round(float(periph_msg[4] | periph_msg[5] << 8), 2)
       	    batt_I = (adc_I * ADC_VOLTAGE_MUL - BATT_AMP_OFFSET) * BATT_AMP_PERVOLT
            pub_volt.send_multipart([zmq_topics.topic_volt,pickle.dumps({'ts':tic,'V':batt_V,'I':batt_I})])
            print("Batt V: {}".format(batt_V))
            print("Batt I: {}".format(batt_I))
        else:
            print("Start byte not correct!")
            time.sleep(.1)


