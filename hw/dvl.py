#torun 
# conda activate 3.6 
#from vnpy import *
#dvl protocol info:
#https://waterlinked.github.io/dvl/dvl-protocol/
#https://store.waterlinked.com/product/dvl-a50/
#alternative dvl http://www.teledynemarine.com/Wayfinder

print('done import')
import time,sys
import traceback
import os,zmq
import math
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
    data, checksum = line.split(b'*')
    return crc(bytes(data)) == int(checksum, 16)


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
    #import ipdb;ipdb.set_trace()
    #sys.exit(0)
    data=line.split(b'*')[0].split(b',')
    ret=None
    if data[0]==b'wrz':
        # Velocity report
        ret={'type':'vel'}
        keys='vx,vy,vz,valid,alt,fom,cov,tov,tot,time,status'.split(',')  
        for i in range(len(keys)):
            if i == 3:
                ret[keys[i]]=data[i+1]
            elif i == 6:
                ret[keys[i]]=[float(v) for v in data[i+1].split(b';')]
            else:
                ret[keys[i]]=float(data[i+1])

    if data[0]==b'wru':
        # Transducer report
        ret={'type':'transducer'}
        keys='id,velocity,distance,rssi,nsd'.split(',')
        for i in range(len(keys)):
            ret[keys[i]]=float(data[i+1])
        
    if data[0]==b'wrp':
        # Deadreckoning report
        keys='time,x,y,z,pos_std,roll,pitch,yaw,status'.split(',')
        ret={'type':'deadreacon'}
        for i in range(len(keys)):
            #print(keys[i],data[i+1])
            ret[keys[i]]=float(data[i+1]) if i<=8 else data[i+1]

    if data[0]==b'wrn' or data[0]==b'wra':
        # DR reset reply
        ret={'type':'Reset DR reply'}
        ret['res']=str(data[0])

    return ret

if __name__=='__main__':
    if len(sys.argv)==1:
        ser = init_serial()
        pub_dvl = utils.publisher(zmq_topics.topic_dvl_port)
        pub_srange = utils.publisher(zmq_topics.topic_sonar_port)
        cnt=0
        last_time=None
        while 1:
            line=ser.readline()
            #print(line)
            try:
                d=parse_line(line)
                if d and d['type']=='deadreacon':
                    last_time = d['time']
                if d and cnt%51==0:
                    print('parsed ',cnt,'msgs')
                    print('d=',d)
                cnt+=1
                # Mock downward facing SONAR output
                if d and d['type']=='vel':
                    to_send=pickle.dumps({'ts':tic, 'sonar':[d['alt'], 1 if d['valid']==b'y' else 0]})
                    pub_srange.send_multipart([zmq_topics.topic_sonar,to_send])
            except Exception as e:
                print('-----------------------')
                traceback.print_exc(file=sys.stdout)
                print(e)
                #traceback.print_exc(file=sys.stdout)
            tic = time.time()
            pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps({'ts':tic,'dvl_raw':line})])
    else:
        fl = sys.argv[1]
        #fl = '../../data//220322-123731/viewer_data.pkl'
        drs=[]
        trs=[]
        vels=[]
        with open(fl,'rb') as fd:
            while 1:
                try:
                    data=pickle.load(fd)
                except EOFError:
                    break
                if data[0]==zmq_topics.topic_dvl_raw:
                    #line=pickle.loads(data[1])#['dvl_raw']
                    d = parse_line(data[1]['dvl_raw'])
                    if d is not None and d['type']=='deadreacon':
                        drs.append(d)
                    if d is not None and d['type']=='transducer':
                        trs.append(d)
                    if d is not None and d['type']=='vel':
                        vels.append(d)
                    print(data[1]['ts'],parse_line(data[1]['dvl_raw']))
                    #print(parse_line(line))
        ga = lambda x:[d[x] for d in drs]
        gt = lambda x:[d[x] for d in trs]
        gv = lambda x:[d[x] for d in vels if d['valid']==b'y']
        import numpy as np
        un = lambda x:np.rad2deg(np.unwrap(np.deg2rad(x)))
        import matplotlib.pyplot as plt
        plt.figure('deadreacon')
        plt.subplot(2,1,1)
        plt.plot(ga('time'),ga('x'))
        plt.plot(ga('time'),ga('y'))
        plt.plot(ga('time'),ga('z'))
        plt.legend(['x','y','z'])
        plt.subplot(2,1,2)
        plt.plot(ga('time'),un(ga('yaw')))
        plt.plot(ga('time'),ga('pitch'))
        plt.plot(ga('time'),ga('roll'))
        plt.legend(['yaw','pitch','roll'])

        plt.figure('transducer')
        plt.subplot(1,1,1)
        for i in range(4):                                                                                           
            d=[j['dist'][i] for j in trs]
            plt.plot(d,alpha=0.6)  

        plt.figure('vels')
        ax=plt.subplot(2,1,1)
        tt=range(len(gv('time'))) #fix me add time to each frame
        plt.plot(tt,gv('vx'))
        plt.plot(tt,gv('vy'))
        plt.plot(tt,gv('vz'))
        plt.legend(['vx','vy','vz'])
        plt.subplot(2,1,2,sharex=ax)
        plt.plot(tt,gv('alt'),'.-')
        plt.legend(['alt'])
        
        plt.show()


