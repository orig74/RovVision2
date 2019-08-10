#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys,os,time
sys.path.append('unreal_proxy')
sys.path.append('..')
sys.path.append('../utils')
import zmq
import struct
import cv2,os
import numpy as np
import pickle
import zmq_wrapper as utils
import ue4_zmq_topics
import zmq_topics
import config
topic_depth=ue4_zmq_topics.topic_unreal_depth%0

zmq_sub=utils.subscribe([topic_depth],ue4_zmq_topics.zmq_pub_unreal_proxy[1])

cvshow=1
#cvshow=False
test=1

print('start...')
def listener():
    cnt=0
    rgb=None
    while 1:
        while len(zmq.select([zmq_sub],[],[],0.001)[0])>0:
            data = zmq_sub.recv_multipart()
            topic=data[0]
            frame_cnt,shape=pickle.loads(data[1])
            
            if topic==topic_depth:
                img=np.frombuffer(data[2],'float16').reshape(shape)
                img=np.squeeze(img).copy()

                img_show=(img/10.0).clip(0,255).astype('uint8')

                if cvshow:
                    cv2.imshow(topic.decode(),img_show)
                    cv2.waitKey(1)

            ### test
        time.sleep(0.010)
        cnt+=1


       
if __name__ == '__main__':
    listener()
