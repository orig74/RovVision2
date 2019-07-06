# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys,os,time
from datetime import datetime
sys.path.append('../')
sys.path.append('../utils')
import zmq
import pickle
import select
import struct
import cv2,os
import signal
import argparse
import numpy as np
import zmq_topics
import config
from config import Joy_map as jm
from gst import init_gst_reader,get_imgs,set_files_fds,get_files_fds,save_main_camera_stream
from annotations import draw
import zmq_wrapper as utils
import image_enc_dec

parser = argparse.ArgumentParser()
parser.add_argument("--data_path", help="path for data" , default='../../data')
args = parser.parse_args()

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_controller_messages],zmq_topics.topic_controler_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_button, zmq_topics.topic_hat ], zmq_topics.topic_joy_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_imu ], zmq_topics.topic_imu_port) )
subs_socks.append(utils.subscribe([zmq_topics.topic_depth ], zmq_topics.topic_depth_port) )
subs_socks.append(utils.subscribe([zmq_topics.topic_sonar ], zmq_topics.topic_sonar_port) )

#socket_pub = utils.publisher(config.zmq_local_route)
socket_pub = utils.publisher(zmq_topics.topic_local_route_port,'0.0.0.0')
pub_record_state = utils.publisher(zmq_topics.topic_record_state_port)

if __name__=='__main__':
    init_gst_reader(2)
    sx,sy=config.cam_res_rgbx,config.cam_res_rgby
    join=np.zeros((sy,sx*2,3),'uint8')
    data_file_fd=None
    #main_camera_fd=None
    message_dict={}
    rcv_cnt=0
    record_state=False
    joy_buttons=[0]*16
    while 1:
        images=get_imgs()
        rcv_cnt+=1
        #if all(images):
        while 1:
            socks=zmq.select(subs_socks,[],[],0.001)[0]
            if len(socks)==0: #flush msg buffer
                break
            for sock in socks:
                ret = sock.recv_multipart()
                topic , data = ret
                data=pickle.loads(ret[1])
                
                message_dict[topic]=data
                
                if topic==zmq_topics.topic_button:
                    new_joy_buttons=pickle.loads(ret[1])
                    if new_joy_buttons[jm.record_bt]==1 and joy_buttons[jm.record_bt]==0:
                        #togel recording
                        if not record_state:
                            record_state=datetime.now().strftime('%y%m%d-%H%M%S') 
                        else:
                            record_state=False
                    joy_buttons=new_joy_buttons
                    pub_record_state.send_multipart([zmq_topics.topic_record_state,pickle.dumps(record_state)])
                    message_dict[zmq_topics.topic_record_state]=record_state

                if record_state:
                    if get_files_fds()[0] is None:
                        print('start recording...')
                        fds=[]
                        #datestr=sensor_gate_data['record_date_str']
                        datestr=record_state
                        save_path=args.data_path+'/'+datestr
                        if not os.path.isdir(save_path):
                            os.mkdir(save_path)
                        for i in [0,1]:
                            #datestr=datetime.now().strftime('%y%m%d-%H%M%S')
                            fds.append(open(save_path+'/vid_{}.mp4'.format('lr'[i]),'wb'))
                        set_files_fds(fds)
                        data_file_fd=open(save_path+'/viewer_data.pkl','wb')
                        pickle.dump([b'start_time',time.time()],data_file_fd,-1)
                else:
                    if get_files_fds()[0] is not None:
                        print('done recording...')
                        set_files_fds([None,None])
                        data_file_fd=None

                if data_file_fd is not None:
                    pickle.dump([topic,data],data_file_fd,-1)
                    #pickle.dump([b'viewer_data',{'rcv_cnt':rcv_cnt}],data_file_fd,-1)

        #print('-1-',main_data)

        if images[0] is not None and images[1] is not None:
            fmt_cnt_l=image_enc_dec.decode(images[0])
            fmt_cnt_r=image_enc_dec.decode(images[1])
            join[:,0:sx,:]=images[0]
            join[:,sx:,:]=images[1]
            images=[None,None]
            draw(join,message_dict,fmt_cnt_l,fmt_cnt_r)
            cv2.imshow('3dviewer',join)
            if data_file_fd is not None:
                pickle.dump([zmq_topics.topic_viewer_data,{'frame_cnt':(rcv_cnt,fmt_cnt_l,fmt_cnt_r),'ts':time.time()}],data_file_fd,-1)
            #cv2.imshow('left',images[0])
            #cv2.imshow('right',images[1])
        k=cv2.waitKey(10)
        if k==ord('q'):
            for p in gst_pipes:
                p.terminate()
                p.poll()
            break
