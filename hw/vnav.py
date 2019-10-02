#torun 
# conda activate 3.6 
#from vnpy import *
print('done import')
import time,sys
import os,zmq
sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
print('done import 2')
import zmq_topics
import asyncio,pickle
import serial
pub_imu = utils.publisher(zmq_topics.topic_imu_port)
##### vnav commands (setup)
#doc in https://www.vectornav.com/docs/default-source/documentation/vn-100-documentation/vn-100-user-manual-(um001).pdf?sfvrsn=b49fe6b9_32
vn_pause = b'$VNASY,0*XX'
vn_resume = b'$VNASY,1*XX'
vn_setoutput_quaternion =b'$VNWRG,06,2*XX'
#Yaw, Pitch, Roll, Magnetic, Acceleration, and Angular Rate Measurements
#header VNYMR
vn_setoutput=b'$VNWRG,06,14*XX' 
##### 

print('connecting to vnav')
#s=EzAsyncData.connect(detect_usb.devmap['VNAV_USB'],115200)
def init_serial():
    import detect_usb
    ser = serial.Serial(detect_usb.devmap['VNAV_USB'],115200)
    return ser

def parse_line(line):
    parts=line.strip().split(b'*')[0].split(b',')
    if parts[0]==b'$VNRRG':
        y,p,r,mx,my,mz,ax,ay,az,gx,gy,gz = map(float,parts[2:])
    ret={}
    ret['ypr'] = (y,p,r)
    ret['rates'] = (gx,gy,gz)
    ret['mag'] = (mx,my,mz)
    ret['acc'] = (ax,ay,az)


print('done')
#hz=40
#s.sensor.write_async_data_output_frequency(hz) #hz
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
        print('waiting...')
        cd=s.current_data
        prev_tt = id(cd)
        prev_ypr = cd.yaw_pitch_roll

    while 1:
        cd=s.current_data
        #import pdb;pdb.set_trace()
        if cd.yaw_pitch_roll is None:# or id(cd)==prev_tt:
            time.sleep(0.01)
            print('skip')
            continue
        #print('got pack',cnt)
        #new packet
        #dt = (cd.time_startup - prev_tt)/1e9
        dt=1.0/hz
        #prev_tt = id(cd)
        tic=time.time()
        imu={'ts':tic}
        ypr=cd.yaw_pitch_roll
        imu['yaw'],imu['pitch'],imu['roll']=(ypr.x,ypr.y,ypr.z)
        #imu['yawr'],imu['pitchr'],imu['rollr']=((ypr.x-prev_ypr.x)/dt, (ypr.y-prev_ypr.y)/dt, (ypr.z-prev_ypr.z)/dt)
        prev_ypr=ypr
        rates=cd.angular_rate
        imu['rates']=(rates.x,rates.y,rates.z)

        if cnt%5==0:
            print('dsim Y{:4.2f} P{:4.2f} R{:4.2f}'.format(imu['yaw'],imu['pitch'],imu['roll'])
                        +' X{:4.2f} Y{:4.2f} Z{:4.2f}'.format(*imu['rates']))
        pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])
        cnt+=1
        time.sleep(1.0/hz)

if __name__=='__main__':
    #ser = init_serial()
    #recv_and_process()
    testline=b'$VNRRG,27,+006.380,+000.023,-001.953,+1.0640,-0.2531,+3.0614,+00.005,+00.344,-09.758,-0.001222,-0.000450,-0.001218*4F'
    print(parse_line(testline))
