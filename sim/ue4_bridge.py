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
topic_stereo=ue4_zmq_topics.topic_unreal_stereo_camera%0
topic_depth=ue4_zmq_topics.topic_unreal_depth%0

zmq_sub=utils.subscribe([topic_stereo,topic_depth],ue4_zmq_topics.zmq_pub_unreal_proxy[1])
zmq_pub=utils.publisher(zmq_topics.topic_camera_port)

pub_sonar = utils.publisher(zmq_topics.topic_sonar_port)
cvshow=0
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
            if topic==topic_stereo:
                imgr=np.frombuffer(data[2],'uint8').reshape(shape)
                imgl=np.frombuffer(data[3],'uint8').reshape(shape)
                    
                ch,cw,cl = shape
                ch = ch//2
                cw = cw//2

                if config.rov_type==2: #resizing for prev cameras
                    nh,nw = config.cam_res_rgby, config.cam_res_rgbx
                    imgl = imgl[ch-nh//2:ch+nh//2,cw-nw//2:cw+nw//2,:]
                    imgr = imgr[ch-nh//2:ch+nh//2,cw-nw//2:cw+nw//2,:]
                    shape = imgl.shape

                if 0 and cvshow:
                    #if 'depth' in topic:
                    #    cv2.imshow(topic,img)
                    #else:
                    #cv2.imshow(topic,cv2.resize(cv2.resize(img,(1920/2,1080/2)),(512,512)))
                    imgls = imgl[::2,::2]
                    imgrs = imgr[::2,::2]
                    cv2.imshow(topic.decode()+'l',imgls)
                    cv2.imshow(topic.decode()+'r',imgrs)
                    cv2.waitKey(1)
                zmq_pub.send_multipart([zmq_topics.topic_stereo_camera,pickle.dumps([frame_cnt,shape]),imgl.tostring(),imgr.tostring()])
                time.sleep(0.001) 
                zmq_pub.send_multipart([zmq_topics.topic_stereo_camera_ts,pickle.dumps((frame_cnt,time.time()))]) #for sync
            
            if topic==topic_depth:
                img=np.frombuffer(data[2],'float16').reshape(shape)
                min_range=img.min() 
                #import pdb;pdb.set_trace()
                img=np.squeeze(img).copy()
                #img.byteswap()

                img_show=(img/10.0).clip(0,255).astype('uint8')
                img[img>5000]=np.nan
                max_range=np.nanmax(img)
                print('sonar::',min_range,max_range)
                pub_sonar.send_multipart([zmq_topics.topic_sonar,pickle.dumps([min_range,max_range])])

                if cvshow:
                    cv2.imshow(topic.decode(),img_show)
                    cv2.waitKey(1)

            ### test
        time.sleep(0.010)
        if cnt%20==0 and rgb is not None:
            print('send...',cnt, rgb.shape)
        cnt+=1


       
if __name__ == '__main__':
    listener()
