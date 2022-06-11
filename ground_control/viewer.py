# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys,os,time
time.sleep(10)
from datetime import datetime
sys.path.append('../')
sys.path.append('../utils')
sys.path.append('../onboard')
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
from joy_mix import Joy_map
vid_zmq = config.is_sim and config.sim_type=='PB'
if not vid_zmq:
    from gst import init_gst_reader,get_imgs,set_files_fds,get_files_fds,save_main_camera_stream
from annotations import draw,draw_seperate,draw_mono
import zmq_wrapper as utils
import image_enc_dec
import camCalib as CC

parser = argparse.ArgumentParser()
parser.add_argument("--data_path", help="path for data" , default='../../data')
parser.add_argument("--pub_data", help="publish data", action='store_true')
parser.add_argument("--depth", help="Display Depth", action='store_true')
args = parser.parse_args()

resize_viewer = 'RESIZE_VIEWER' in os.environ
if resize_viewer:
    resize_width=int(os.environ['RESIZE_VIEWER'])

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand,zmq_topics.topic_system_state, zmq_topics.topic_lights],zmq_topics.topic_controller_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_button, zmq_topics.topic_hat], zmq_topics.topic_joy_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_imu], zmq_topics.topic_imu_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_dvl_raw], zmq_topics.topic_dvl_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_depth], zmq_topics.topic_depth_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_depth_hold_pid], zmq_topics.topic_depth_hold_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_sonar], zmq_topics.topic_sonar_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_sonar_hold_pid], zmq_topics.topic_sonar_hold_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_stereo_camera_ts], zmq_topics.topic_camera_ts_port)) #for sync perposes
#subs_socks.append(utils.subscribe([zmq_topics.topic_stereo_camera], zmq_topics.topic_camera_port)) #for sync perposes
subs_socks.append(utils.subscribe([zmq_topics.topic_tracker], zmq_topics.topic_tracker_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_telem], zmq_topics.topic_telem_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_hw_stats], zmq_topics.topic_hw_stats_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_gps], zmq_topics.topic_gps_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_stereo_camera_calib], zmq_topics.topic_camera_calib_port))

subs_socks.append(utils.subscribe([zmq_topics.topic_pos_hold_pid_fmt%i for i in range(3)], zmq_topics.topic_pos_hold_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_att_hold_yaw_pid,
                                   zmq_topics.topic_att_hold_pitch_pid,
                                   zmq_topics.topic_att_hold_roll_pid], zmq_topics.topic_att_hold_port))
    
sub_vid=utils.subscribe([zmq_topics.topic_stereo_camera], zmq_topics.topic_camera_port) #for sync perposes
#socket_pub = utils.publisher(config.zmq_local_route)
if args.pub_data:
    socket_pub = utils.publisher(zmq_topics.topic_local_route_port,'0.0.0.0')
pub_record_state = utils.publisher(zmq_topics.topic_record_state_port)

DEPTH_THESH = 10
DEPTH_MODULO = 4
valid_calib = False
if args.depth:
  rectifier = CC.CalibParams()
  num_disparity = 256
  left_matcher = cv2.StereoBM_create(numDisparities=num_disparity, blockSize=21)
  right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
  wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
  wls_filter.setLambda(1500)
  wls_filter.setSigmaColor(1.5)

if __name__=='__main__':
    if not vid_zmq:
        init_gst_reader(2)
    sx,sy=config.cam_res_rgbx,config.cam_res_rgby
    data_file_fd=None
    #main_camera_fd=None
    message_dict={}
    rcv_cnt=0
    record_state=False
    jm=Joy_map()
    bmargx,bmargy=config.viewer_blacks

    while 1:
        #join=np.zeros((sy,sx*2,3),'uint8')
        join=np.zeros((sy+bmargy,sx*2+bmargx,3),'uint8')
        if not vid_zmq:
            images = get_imgs()
        else:
            while 1:
                ret=sub_vid.recv_multipart()
                frame_cnt,shape = pickle.loads(ret[1])
                images = []
                for im in ret[2:]:
                    images.append(np.frombuffer(im,'uint8').reshape(shape))
                print('======',len(images),ret[0])
                if len(images)>0:
                    break
        
        rcv_cnt+=1
        #if all(images):
        while 1:
            socks=zmq.select(subs_socks,[],[],0.005)[0]
            if len(socks)==0: #flush msg buffer
                break
            for sock in socks:
                ret = sock.recv_multipart()
                topic = ret[0]
                if topic == zmq_topics.topic_stereo_camera:
                    print('errrrrrrooooorr')
                #    frame_cnt,shape = pickle.loads(ret[1])
                #    images = []
                #    for im in ret[2:]:
                #        images.append(np.frombuffer(im,'uint8').reshape(shape))
                #else:
                data = ret[1]
                data=pickle.loads(ret[1])
                
                message_dict[topic]=data
                
                if topic==zmq_topics.topic_button:
                    jm.update_buttons(data)
                    if jm.record_event(): 
                        #toggle recording
                        if not record_state:
                            record_state=datetime.now().strftime('%y%m%d-%H%M%S') 
                        else:
                            record_state=False
                    pub_record_state.send_multipart([zmq_topics.topic_record_state,pickle.dumps(record_state)])
                    message_dict[zmq_topics.topic_record_state]=record_state
                
                if args.depth and topic==zmq_topics.topic_stereo_camera_calib:
                  print("Camera params recieved!")
                  try:
                    rectifier.__dict__.update(data)
                    leftIntrinsics = rectifier.proj_mat_l
                    rightIntrinsics = rectifier.proj_mat_r
                    stereoTrns = rectifier.Trns
                    STEREO_FOCAL_LENGTH = leftIntrinsics[0, 0]
                    STEREO_BASELINE = np.linalg.norm(stereoTrns)
                    valid_calib = True
                  except:
                    print("Failed to get camera calibration")

                if args.pub_data:
                    socket_pub.send_multipart([ret[0],ret[1]])

                if not vid_zmq:
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
                            os.system("gst-launch-1.0 -v -e udpsrc port=17894  ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! qtmux ! filesink location=%s sync=false & "%(save_path+'/main_cam.mov'))
                    else:
                        if get_files_fds()[0] is not None:
                            print('done recording...')
                            os.system('pkill -2 -f "gst-launch-1.0 -v -e udpsrc port=17894"')
                            set_files_fds([None,None])
                            data_file_fd=None

                if data_file_fd is not None:
                    pickle.dump([topic,data],data_file_fd,-1)
                    #pickle.dump([b'viewer_data',{'rcv_cnt':rcv_cnt}],data_file_fd,-1)

        #print('-1-',main_data)

        if config.camera_setup == 'stereo':
            if images[0] is not None and images[1] is not None:
                fmt_cnt_l=image_enc_dec.decode(images[0])
                fmt_cnt_r=image_enc_dec.decode(images[1])
                if args.depth and fmt_cnt_l % DEPTH_MODULO == 0 and valid_calib:
                    gray_l = cv2.cvtColor(images[0], cv2.COLOR_BGR2GRAY)
                    gray_r = cv2.cvtColor(images[1], cv2.COLOR_BGR2GRAY)
                    left_disp = left_matcher.compute(gray_l, gray_r)
                    right_disp = right_matcher.compute(gray_r, gray_l)
                    filtered_disp = wls_filter.filter(left_disp, gray_l, disparity_map_right=right_disp).astype(np.float32)
                    depth_img = STEREO_BASELINE * STEREO_FOCAL_LENGTH / ((filtered_disp + 16.00001) / 16)
                    depth_img = (depth_img < DEPTH_THESH) * depth_img
                    cv2.imshow("Depth Image", depth_img / DEPTH_THESH)
                draw_seperate(images[0],images[1],message_dict)
                #join[:,0:sx,:]=images[0]
                #join[:,sx:,:]=images[1]
                join[bmargy//2:-bmargy//2,bmargx:sx+bmargx,:]=images[0]
                join[bmargy//2:-bmargy//2,sx+bmargx:,:]= images[1] #+ 0.5*images[0]
                images=[None,None]
                draw(join,message_dict,fmt_cnt_l,fmt_cnt_r)
        else:
            join = None
            if images[0] is not None:
                fmt_cnt_l=image_enc_dec.decode(images[0])
                draw_mono(images[0],message_dict,fmt_cnt_l)
                join=images[0]
        if join is not None:
            if resize_viewer:
                scale=resize_width/config.cam_resx
                sp0,sp1,_ = join.shape
                sp0=int(sp0*scale)
                sp1=int(sp1*scale)
                cv2.imshow('3dviewer',cv2.resize(join,(sp1,sp0)))
            else:
                cv2.imshow('3dviewer',join)
            if data_file_fd is not None:
                pickle.dump([zmq_topics.topic_viewer_data,{'frame_cnt':(rcv_cnt,fmt_cnt_l,fmt_cnt_r),'ts':time.time()}],data_file_fd,-1)
            
            #cv2.imshow('left',images[0])
            #cv2.imshow('right',images[1])

        k=cv2.waitKey(10)
        if k==ord('q'):
            #for p in gst_pipes:
            #    p.terminate()
            #    p.poll()
            break
