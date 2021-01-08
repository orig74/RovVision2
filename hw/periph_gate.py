import serial
import time
import numpy as np
import struct
import sys
import pickle
import zmq
sys.path.append('../utils')
sys.path.append('../')
import detect_usb
import zmq_wrapper
import zmq_topics
import config
pub_depth = zmq_wrapper.publisher(zmq_topics.topic_depth_port) 
pub_volt = zmq_wrapper.publisher(zmq_topics.topic_volt_port)
ser = serial.Serial(detect_usb.devmap['PERI_USB'], 115200)
time.sleep(2)

BATT_AMP_OFFSET = 0.330
BATT_AMP_PERVOLT = 37.8788
ADC_VOLTAGE_MUL = 0.0046
BATT_VOLTAGE_MUL = 11

LIGHTS_MAX=5

print('connected to ', detect_usb.devmap['PERI_USB'])
subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_lights],zmq_topics.topic_controller_port))
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_record_state ],zmq_topics.topic_record_state_port))

#start triggering at config fps
ser.write(b'\x01')
ser.write(b'%c'%(config.fps+10))
#ser.flush()
print('trigger sent')

cur_lights_cmd = 0
prev_rec_state = 0

while True:
    socks=zmq.select(subs_socks,[],[],0.005)[0]
    for sock in socks:
        ret=sock.recv_multipart()
        topic,data=ret[0],pickle.loads(ret[1])
        if topic == zmq_topics.topic_record_state and data != prev_rec_state:
            print('Record start/stop sync light blink')
            ser.write(b'%c' % (0+2))
            time.sleep(0.5)
            ser.write(b'%c' % (LIGHTS_MAX+2))
            time.sleep(0.5)
            ser.write(b'%c' % (cur_lights_cmd+2))
            prev_rec_state = data

        if topic==zmq_topics.topic_lights:
            print('got lights command',data)
            ser.write(b'%c'%(data+2))
            cur_lights_cmd = data
            #ser.flush()

    if ser.in_waiting >= 9:
        if ser.read(1)[0] == 255:
            periph_msg = ser.read(8)
            msg_data = struct.unpack('HHhh', periph_msg)
            tic=time.time()

            bar_D = msg_data[0] / 200
            temp_C = msg_data[1] / 200
            pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':bar_D, 'temp':temp_C})])

            batt_V = round(msg_data[2] * ADC_VOLTAGE_MUL * BATT_VOLTAGE_MUL, 2)
            batt_I = round((msg_data[3] * ADC_VOLTAGE_MUL - BATT_AMP_OFFSET) * BATT_AMP_PERVOLT, 2)
            
            pub_volt.send_multipart([zmq_topics.topic_volt,pickle.dumps({'ts':tic,'V':batt_V,'I':batt_I})])
            print("Batt V: {}".format(batt_V))
            print("Batt I: {}".format(batt_I))
        else:
            print("Start byte not correct!")
            time.sleep(.1)


