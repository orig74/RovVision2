import time,sys
import os,zmq
sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
print('done import 2')
import zmq_topics
import asyncio,pickle

from brping import Ping1D
import detect_usb
dev=detect_usb.devmap['SONAR_USB']

myPing = Ping1D(dev, 115200)
if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)

pub_imu = utils.publisher(zmq_topics.topic_sonar_port)
cnt=0
while 1:
    time.sleep(0.1)
    data = myPing.get_distance()
    if cnt%10==0:
        print('sonar ',data)
    tosend = pickle.dumps([data['distance'],data['confidence']])
    pub_imu.send_multipart([zmq_topics.topic_sonar,tosend])
    cnt+=1
#
