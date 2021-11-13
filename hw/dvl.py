#torun 
# conda activate 3.6 
#from vnpy import *
#dvl protocol info:
#https://waterlinked.github.io/dvl/dvl-protocol/
#https://store.waterlinked.com/product/dvl-a50/
#alternative dvl http://www.teledynemarine.com/Wayfinder

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
import crcmod
crc = crcmod.predefined.mkPredefinedCrcFun("crc-8")
def check_crc(line):
    data, checksum = line.split(b"*")
    return crc(data) == int(checksum, 16)


def init_serial(dev=None):
    if dev is None:
        import detect_usb
        dev=detect_usb.devmap['DVL_USB']

    ser = serial.Serial(dev,115200)
    return ser

def parse_line(line):
    if not check_crc(line):
        print('Error Bad CRC')
        return None
    data=line.split('*')[0].split(',')
    ret=None
    if data[0]=='wrx':
        #Velocity report
        ret={'type':'vel'}
        keys='time,vx,vy,vz,fom,alt,valid,status'.split(',')
        for i in range(len(keys):
            ret[keys[i]]=float(data[i+1]) if i<=5 else data[i+1]

    if data[0]=='wrt':
        #Transducer report
        ret={'type':transducer}
        ret={'dist':[float(data[i]) for i in range(4)]}
        
    if data[0]=='wrp':
        keys='time,x,y,z,pos_std,roll,pitch,yaw,status'
        for i in range(len(keys):
            ret[keys[i]]=float(data[i+1]) if i<=7 else data[i+1]
    return ret

if __name__=='__main__':
    ser = init_serial()
    pub_dvl = utils.publisher(zmq_topics.topic_dvl_port)
    while 1:
        line=ser.readline()
        try:
            d=parse_line(line)
        except Exception:
            print('-----------------------')
            traceback.print_exc(file=sys.stdout)
        pub_imu.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps({'ts':time.time(),'dvl_raw':line})])

