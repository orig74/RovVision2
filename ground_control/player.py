# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#
# to copy data.pkl
# sim:
# cd into data dir and then
# > scp -P 2222 oga13@localhost:projects/bluerov/data/$(basename $( pwd ))/data.pkl .

import sys,os,time,traceback
from datetime import datetime
sys.path.append('../')
sys.path.append('../utils')
import zmq
import pickle
import select
import struct
import cv2,os
import copy
import argparse
import numpy as np
import pandas as pd
import config
import gst2
import zmq_wrapper
import zmq_topics


parser = argparse.ArgumentParser()
parser.add_argument("--nowait",help="run all not wait for keyboard untill framenum or 0 till the end", default=0, type=int)
parser.add_argument("--nosingle",help="dont use the single files only the stream", action='store_true')
#parser.add_argument("--nosync", help="dont sync videos", action='store_true')
#parser.add_argument("--pub_data", help="publish data", action='store_true')
#parser.add_argument("--pub_camera", help="publish camera", action='store_true')
#parser.add_argument("--runtracker", help="run tracker on recorded vid", action='store_true')
parser.add_argument("--bs",help="history buff size",type=int ,default=1000)
parser.add_argument("--cc",help="color orrection matrix file",default=None)
parser.add_argument("--ccr",help="the efect of the color correction 0 to 1.0 (max 1.0)",type=float,default=1.0)
parser.add_argument("path",help="dir path")
args = parser.parse_args()

#socket_pub = zmq_wrapper.publisher(zmq_topics.topic_local_route_port,'0.0.0.0')

def read_pkl(pkl_file):
    fd = open(pkl_file,'rb')
    ret=[]
    while 1:
        try:
            ret.append(pickle.load(fd))
            #print('----',ret)
        except EOFError:
            break
        if ret[0]==zmq_topics.topic_viewer_data:
            msg=ret[1]
            if start==-1:
                start=msg['frame_cnt'][0]
            end=msg['frame_cnt'][0]
    return ret

def get_next_image(stream,rcnt):
    while 1:
        if stream.prevcnt>=rcnt:
            print('error stream',stream.prevcnt,rcnt)
            return None
        cnt,img=stream.get_img()
        #print('got image == ',cnt)
        if img is not None:
            if rcnt==cnt:
                #print('===',rcnt)
                return img
            if cnt<rcnt:
                print('error2 stream',cnt,rcnt)
                continue
        time.sleep(0.001)

zmq_pub_stereo=zmq_wrapper.publisher(zmq_topics.topic_camera_port)
zmq_pub_stereo_ts=zmq_wrapper.publisher(zmq_topics.topic_camera_ts_port)
zmq_pub_main_camera=zmq_wrapper.publisher(zmq_topics.topic_main_cam_port)
zmq_pub_main_camera_ts=zmq_wrapper.publisher(zmq_topics.topic_main_cam_ts_port)

map_port_topic = {}
map_port_topic[zmq_topics.topic_imu_port]=[zmq_topics.topic_imu]
map_port_topic[zmq_topics.topic_tracker_port]=[zmq_topics.topic_tracker]
map_port_topic[zmq_topics.topic_main_tracker_port]=[zmq_topics.topic_main_tracker]
map_port_topic[zmq_topics.topic_controller_port]=[zmq_topics.topic_thrusters_comand,zmq_topics.topic_system_state,zmq_topics.topic_lights]
map_port_topic[zmq_topics.topic_dvl_port]=[zmq_topics.topic_dvl_raw]
map_port_topic[zmq_topics.topic_depth_port]=[zmq_topics.topic_depth]
map_port_topic[zmq_topics.topic_gps_port]=[zmq_topics.topic_gps]
map_port_topic[zmq_topics.topic_pos_hold_port]=[zmq_topics.topic_pos_hold_pid_fmt%i for i in range(3)]
map_port_topic[zmq_topics.topic_att_hold_port]=[zmq_topics.topic_att_hold_yaw_pid,zmq_topics.topic_att_hold_pitch_pid,zmq_topics.topic_att_hold_roll_pid]
map_port_topic[zmq_topics.topic_depth_hold_port]=[zmq_topics.topic_depth_hold_pid]

map_topic_port={}
map_port_publisher={}
for p in map_port_topic:
    if p not in map_port_publisher:
        map_port_publisher[p]=zmq_wrapper.publisher(p)
    for t in map_port_topic[p]:
        map_topic_port[t]=p



if __name__=='__main__':
    vdata = read_pkl(args.path+'/viewer_data.pkl')
    keys=set()
    for v in vdata:
        keys.add(v[0])
    print('got keys',keys)
    vid_main_cnt=-1
    vid_main=gst2.FileReader(args.path+'/vid_main.mp4')
    vid_main_depth=gst2.FileReader(args.path+'/vid_main_depth.mp4')
    vid_stereo_cnt=-1
    vid_l=gst2.FileReader(args.path+'/vid_l.mp4',pad_lines=config.cam_res_gst_pad_lines)
    vid_r=gst2.FileReader(args.path+'/vid_r.mp4',pad_lines=config.cam_res_gst_pad_lines)
    time.sleep(4)
    
    start_time_rec=vdata[0][1]
    start_time_rt=time.time()
    for v in vdata[1:]:
        rec_ts=v[1]-start_time_rec
        rt_ts=time.time()-start_time_rt
        rt_delta=rec_ts-rt_ts
        #print('==rtd=',rt_ts,rec_ts,rt_delta,v[1],start_time_rec,vdata[0])
        if rt_delta>0:
            time.sleep(rt_delta)

        data=v[2]
        if v[0]==zmq_topics.topic_stereo_camera_ts:
            cnt=data[0]
            #print('==getting stereo',cnt)
            iml=get_next_image(vid_l,cnt)
            imr=get_next_image(vid_r,cnt)
            if imr is not None and iml is not None:
                print('sending stereo...',cnt)
                zmq_pub_stereo.send_multipart([zmq_topics.topic_stereo_camera,pickle.dumps([cnt,iml.shape]),iml.tostring(),imr.tostring()],copy=False)
        if v[0]==zmq_topics.topic_main_cam_ts:
            cnt=data[0]
            #print('==getting main',cnt)
            imm=get_next_image(vid_main,cnt)
            imd=get_next_image(vid_main_depth,cnt)
            if imm is not None and imd is not None:
                print('sending main...',cnt)
                zmq_pub_main_camera.send_multipart([zmq_topics.topic_main_cam_depth,pickle.dumps(
                    [cnt,None,imd.shape]),imd.tostring()],copy=False)
                zmq_pub_main_camera.send_multipart([zmq_topics.topic_main_cam,pickle.dumps([cnt,imm.shape]),imm.tostring()],copy=False)

        if v[0] in map_topic_port:
            map_port_publisher[map_topic_port[v[0]]].send_multipart([v[0],pickle.dumps(data)])

