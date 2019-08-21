#torun 
# conda activate 3.6 

from vnpy import *
print('done import')
import time,sys
import os,zmq
sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
print('done import 2')
import zmq_topics
import asyncio,pickle
import detect_usb

pub_imu = utils.publisher(zmq_topics.topic_imu_port)

print('connecting to vnav')
s=EzAsyncData.connect(detect_usb.devmap['VNAV_USB'],115200)
print('done')
s.sensor.write_async_data_output_frequency(40) #hz
def get_data():
    cd=s.current_data
    ret={}
    ypr=cd.yaw_pitch_roll
    rates=cd.angular_rate
    #acc=cd.acceleration
    ret['ypr']=(ypr.x,ypr.y,ypr.z)
    ret['rates']=(rates.x,rates.y,rates.z)
    return ret

def recv_and_process():
    cnt=0
    prev_ypr = None
    while prev_ypr is None: #wait for data
        time.sleep(0.010)
        cd=s.current_data
        prev_tt = cd.time_startup
        prev_ypr = cd.yaw_pitch_roll

    while 1:
        cd=s.current_data
        if cd.yaw_pitch_roll is None or cd.time_startup==prev_tt:
            time.sleep(0.01)
            continue
        #new packet
        dt = (cd.time_startup - prev_tt)/1e9
        prev_tt = cd.time_startup
        tic=time.time()
        imu={'ts':tic, 'imuts': cd.time_startup}
        ypr=cd.yaw_pitch_roll
        imu['yaw'],imu['pitch'],imu['roll']=(ypr.x,ypr.y,ypr.z)
        imu['yawr'],imu['pitchr'],imu['rollr']=((ypr.x-prev_ypr.x)/dt, (ypr.y-prev_ypr.y)/dt, (ypr.z-prev_ypr.z)/dt)
        prev_ypr=ypr
        imu['rates']=(rates.x,rates.y,rates.z)

        if cnt%5==0:
            print('dsim Y{:4.2f} P{:4.2f} R{:4.2f}'.format(imu['yaw'],imu['pitch'],imu['roll'])
                        +' X{:4.2f} Y{:4.2f} Z{:4.2f}'.format(*imu['rates']))
        pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])
        cnt+=1

if __name__=='__main__':
    recv_and_process()
