#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys,os,time
sys.path.append('unreal_proxy')
sys.path.append('..')
import zmq
import struct
import cv2,os
import numpy as np
import pickle
import utils
import ue4_zmq_topics
import zmq_topics
import config
topicl=ue4_zmq_topics.topic_unreal_drone_rgb_camera%0+b'l'
topicr=ue4_zmq_topics.topic_unreal_drone_rgb_camera%0+b'r'
topicd=ue4_zmq_topics.topic_unreal_drone_depth%0

zmq_sub=utils.subscribe([topicl,topicr,topicd],ue4_zmq_topics.zmq_pub_unreal_proxy[1])
zmq_pub=utils.publisher(zmq_topics.topic_camera_port)
cvshow=1
#cvshow=False
test=1

print('start...')
def listener():
    cnt=0
    rgb=None
    while 1:
        while len(zmq.select([zmq_sub],[],[],0.001)[0])>0:
            topic, info, data = zmq_sub.recv_multipart()
            #topic=topic.decode()
            info=struct.unpack('llll',info)
            shape=info[:3]
            frame_cnt=info[3]
            if topic in [topicl,topicr]:
                img=np.fromstring(data,'uint8').reshape(shape)
                rgb=cv2.resize(img[...,::-1],(config.cam_resx,config.cam_resy))
                if cvshow:
                    #if 'depth' in topic:
                    #    cv2.imshow(topic,img)
                    #else:
                    #cv2.imshow(topic,cv2.resize(cv2.resize(img,(1920/2,1080/2)),(512,512)))
                    img_shrk = rgb[::2,::2]
                    cv2.imshow(topic.decode(),img_shrk)
                    cv2.waitKey(1)
            if topic==topicd:
                img=np.fromstring(data,'float16').reshape(shape)
                img=np.squeeze(img)
                img=img.clip(0,255).astype('uint8')
                if cvshow:
                    cv2.imshow(topic.decode(),img)
                    cv2.waitKey(1)

            topic_to_send=None
            if topic==topicl:
                topic_to_send=zmq_topics.topic_camera_left
            if topic==topicr:
                topic_to_send=zmq_topics.topic_camera_right
            if topic in [topicl,topicr]:
                zmq_pub.send_multipart([topic_to_send,pickle.dumps([frame_cnt,rgb],-1)])

                
        
            ### test
        time.sleep(0.010)
        if cnt%20==0 and rgb is not None:
            print('send...',cnt, rgb.shape)
        cnt+=1


       
if __name__ == '__main__':
    listener()
