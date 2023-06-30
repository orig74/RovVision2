import numpy as np
import serial
import sys
import asyncio
import time
import pickle
import struct
import os
import zmq
sys.path.append('../')
from utils import detect_usb
from utils import zmq_wrapper
import zmq_topics
import config

THRSTR_LIMIT = config.thruster_limit
N_SERIAL_RX_BYTES = 15
N_SERIAL_TX_BYTES = 20

BATT_AMP_OFFSET = 0.330
BATT_AMP_PERVOLT = 37.8788
BATT_VOLTAGE_MUL = 11
ESC_AMP_OFFSET = 0.122
ESC_AMP_PERVOLT = 38.0
ADC_VOLTAGE_MUL = 0.000805
ADC_VOLTAGE_OFFSET = 0.122

CMND_TIMEOUT = 1.0
SERIAL_TIMEOUT = 2.0

current_command=[0 for _ in range(8)] # 8 thrusters
keep_running=True
last_thrstcmnd_ts=0
last_serial_rx_ts=time.time()
gripper_val=0
camera_servo = 0.0  # -1.0 -> 1.0, or use 0-255 without scale_val function (mapped to 45->135 degrees camera angle)
rec_state=False
serial_rx_bytes=b''

pub_depth = zmq_wrapper.publisher(zmq_topics.topic_depth_port)
pub_telem = zmq_wrapper.publisher(zmq_topics.topic_telem_port)
subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port))
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_gripper_cmd, zmq_topics.topic_camera_servo],zmq_topics.topic_controller_port))
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_record_state],zmq_topics.topic_record_state_port))

ser = serial.Serial(detect_usb.devmap['ESP_USB'], 500000)

def resetESP32():
    ser.setDTR(False)
    time.sleep(0.5)
    ser.setDTR(True)
    time.sleep(0.5)

def scale_val(value, val_min, val_max, n_bits):
    scaled_range = 2 ** n_bits
    val_norm = float(value - val_min) / (val_max - val_min)
    val_scaled = int(val_norm * (scaled_range - 1))
    assert -1 <= val_scaled <= scaled_range
    return val_scaled

def CalcChksm(bytes):
    chksm = 0
    for byte in bytes:
        chksm += byte
    return chksm

async def send_serial_command_50hz():
    global serial_rx_bytes,camera_servo, last_serial_rx_ts
    while keep_running:
        await asyncio.sleep(1/100.0)
        
        if (time.time() - last_thrstcmnd_ts) < CMND_TIMEOUT:
            m = current_command
        else:
            m = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        m = np.clip(m, -THRSTR_LIMIT, THRSTR_LIMIT)
        
        #print('< ', ["%.1f" % i for i in m], end='')
        #m = 8*[0.0]

        tx_ints = [scale_val(thr, -1.0, 1.0, 16) for thr in m]
        tx_ints.append(scale_val(gripper_val, 0, 1, 8))
        tx_ints.append(scale_val(camera_servo, -1.0, 1.0, 8))
        tx_data = struct.pack('>HHHHHHHHBB', *tx_ints)
        tx_data += struct.pack('>H', CalcChksm(tx_data))
        assert len(tx_data) == N_SERIAL_TX_BYTES 
        ser.write(tx_data)

        while ser.inWaiting():
            serial_rx_bytes += ser.read()
            last_serial_rx_ts = time.time()

        if (time.time() - last_serial_rx_ts) > SERIAL_TIMEOUT:
            print("Serial timeout! Resetting ESP32...")
            resetESP32()
            last_serial_rx_ts = time.time()

        if len(serial_rx_bytes) > N_SERIAL_RX_BYTES-1:
            buff_msg = serial_rx_bytes[-N_SERIAL_RX_BYTES:]
            msg_data = struct.unpack('<HHhhhhBH', buff_msg)
            if CalcChksm(buff_msg[:-2]) == msg_data[-1]:
                serial_rx_bytes = b''
                #print('> ', msg_data)
                tic = time.time()

                bar_D = msg_data[0] / 200
                temp_C = msg_data[1] / 200
                pub_depth.send_multipart([zmq_topics.topic_depth, pickle.dumps({'ts': tic, 'depth': bar_D, 'temp': temp_C})])

                adc_voltage_V = msg_data[2] * ADC_VOLTAGE_MUL + ADC_VOLTAGE_OFFSET
                adc_voltage_I = msg_data[3] * ADC_VOLTAGE_MUL + ADC_VOLTAGE_OFFSET
                batt_V = round(adc_voltage_V * BATT_VOLTAGE_MUL, 2)
                batt_I = round((adc_voltage_I - BATT_AMP_OFFSET) * BATT_AMP_PERVOLT, 2)

                adc_voltage_esc1 = msg_data[4] * ADC_VOLTAGE_MUL + ADC_VOLTAGE_OFFSET
                adc_voltage_esc2 = msg_data[5] * ADC_VOLTAGE_MUL + ADC_VOLTAGE_OFFSET
                esc1_I = round((adc_voltage_esc1 - ESC_AMP_OFFSET) * ESC_AMP_PERVOLT, 2)
                esc2_I = round((adc_voltage_esc2 - ESC_AMP_OFFSET) * ESC_AMP_PERVOLT, 2)

                leak = msg_data[6]

                tosend=pickle.dumps({'ts': tic, 'V': batt_V, 'I': batt_I, 'ESC1_I': esc1_I, 
                                     'ESC2_I': esc2_I, 'leak': leak})
                pub_telem.send_multipart([zmq_topics.topic_telem, tosend])

                print('< ', ["%.1f" % i for i in m], end='')
                #print(' Lights: {}, CamServo: {}'.format(lights_pw, camera_servo))
                print("> Batt V: {}, "\
                      "Batt I: {}, "\
                      "ESC1 I: {}, "\
                      "ESC2 I: {}, "\
                      "Depth: {}, "\
                      "W Temp: {}, "\
                      "Leak: {}".format(batt_V, batt_I, esc1_I, esc1_I, bar_D, temp_C, leak))
            else:
                print("INVALID RX CHKSM")


async def recv_and_process():
    global current_command, rec_state, gripper_val, last_thrstcmnd_ts,camera_servo
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.000)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic, data = ret[0], pickle.loads(ret[1])
            if topic==zmq_topics.topic_thrusters_comand:
                last_thrstcmnd_ts,current_command=data
            if topic == zmq_topics.topic_record_state:
                rec_state=data
            if topic == zmq_topics.topic_gripper_cmd:
                if 'openning' in data:
                    gripper_val=min(max(data['openning'], 0), 1)
            if topic == zmq_topics.topic_camera_servo:
                camera_servo=data
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
