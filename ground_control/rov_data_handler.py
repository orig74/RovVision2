import zmq_topics
import zmq_wrapper as utils
import config
import zmq,pickle
import numpy as np
import image_enc_dec
import os
from annotations import draw_mono,draw_seperate
from joy_mix import Joy_map
from cyc_array import CycArr
from dvl import parse_line as dvl_parse_line
from datetime import datetime
import time
import cv2
import gst2

from zmq import select
print('===is sim ',config.is_sim)
print('===is sim zmq',config.is_sim_zmq)
vid_zmq = config.is_sim_zmq
if not vid_zmq:
    #from gst import init_gst_reader,get_imgs,set_files_fds,get_files_fds#,save_main_camera_stream
    stereo_cam_reader=[gst2.Reader('stereo_'+'lr'[i],config.gst_ports[i],config.cam_res_rgbx,config.cam_res_rgby,pad_lines=config.cam_res_gst_pad_lines)
            for i in [0,1]]
    main_cam_reader=gst2.Reader('main_cam',config.gst_cam_main_port,config.cam_main_sx,config.cam_main_sy)
    main_cam_reader_depth=gst2.Reader('main_cam_depth',config.gst_cam_main_depth_port,config.cam_main_dgui_sx,config.cam_main_dgui_sy)
    #init_gst_reader(2)


from utils import im16to8_22
class rovCommandHandler(object):
    def __init__(self):
        self.pub_sock=utils.publisher(zmq_topics.topic_remote_cmd_port)
        self.vertical_object_lock_state=False

    def armdisarm(self):
        self.pub({'cmd':'armdisarm'})
    
    def heartbit(self):
        self.pub({'cmd':'heartbit'})

    def depth_hold(self):
        self.pub({'cmd':'depth_hold'})

    def att_hold(self):#,yaw,pitch,roll):
        self.pub({'cmd':'att_hold'})

    def att_cmd(self,ypr_deg,relative=True):
        self.pub({'cmd':'att','ypr':ypr_deg,'rel':relative})

    def x_hold(self):
        self.pub({'cmd':'x_hold'})

    def y_hold(self):
        self.pub({'cmd':'y_hold'})

    def z_hold(self):
        self.pub({'cmd':'z_hold'})

    def go(self,p,relative=True):
        self.pub({'cmd':'go','point':p,'rel':relative})

    def depth_command(self,depth,relative=True):
        self.pub({'cmd':'depth','rel':relative,'depth':depth})

    def clear_freqs(self,val,relative=True):
        self.pub({'cmd':'clear_freqs','rel':relative,'data':val})

    def lock(self,x,y):
        self.pub({'cmd':'lock','click_pt':(x,y)})

    def main_track(self,pt):
        self.pub({'cmd':'main_track','click_pt':pt})

    def lock_max(self):
        self.pub({'cmd':'lock_max'})

    def vertical_object_lock(self,rng=0.32,Pxy=0.1):
        self.pub({'cmd':'tracker_vert_object_lock','range':rng,'Pxy':Pxy})
        self.vertical_object_lock_state=True

    def vertical_object_unlock(self):
        self.pub({'cmd':'tracker_vert_object_unlock'})
        self.vertical_object_lock_state=False

    def calib_dvl(self):
        self.pub({'cmd':'dvl_calib'})

    def reset_dvl(self):
        self.pub({'cmd':'dvl_reset'})

    def lights_inc(self,):
        self.pub({'cmd':'lights+'})

    def lights_dec(self):
        self.pub({'cmd':'lights-'})

    def set_rope_tracker_to_hsv(self,n=0):
        self.pub({'cmd':'track_conf','rope_grey_func':'hsv','n':n})

    def set_rope_tracker_to_grey(self,chan):
        self.pub({'cmd':'track_conf','rope_grey_func':'grey','chan':chan})

    def set_gripper(self,val):
        self.pub({'cmd':'gripper','val': val})

    def set_manual_control_limit(self,val):
        self.pub({'cmd':'manual_control_limit','value':val})

    def set_thruster_limit(self,val):
        self.pub({'cmd':'thruster_limit','value':val})
#    def start_recording(self):
#        self.pub({'cmd':'start_recording'})
#
#    def start_recording(self):
#        self.pub({'cmd':'stop_recording'})

    def update_pid(self,pid_type,target,mult):
        print('updateing pid ',pid_type,target,mult)
        pid_type=pid_type.lower()

        if pid_type in ['yaw','pitch','roll']:
            self.pub({'cmd':'exec','script':'att_hold_plugin.py',
                'torun':f'pid_{pid_type[0]}.{target}*={mult}; '+f'printer(pid_{pid_type[0]}.{target})'})

        xy_pids= ['x_hold','y_hold']
        if pid_type in ['x_hold','y_hold']:
            ind=xy_pids.index(pid_type)
            self.pub({'cmd':'exec','script':'pos_hold_dvl_plugin.py',
                'torun':f'pids[{ind}].{target}*={mult};'+f'printer(pids[{ind}].{target})'})

        if pid_type == 'depth':
            self.pub({'cmd':'exec','script':'depth_hold_plugin.py',
                'torun':f'pid.{target}*={mult};'+f'printer(pid.{target})'})

    def save_pid(self,pid_type):
        pid_type=pid_type.lower()
        if pid_type in ['yaw','pitch','roll']:
            self.pub({'cmd':'exec','script':'att_hold_plugin.py',
                'torun':f'pid.dump("att_{pid_type[0]}_pid.json")'})
        
        xy_pids= ['x_hold','y_hold']
        if pid_type in ['x_hold','y_hold']:
            ind=xy_pids.index(pid_type)
            self.pub({'cmd':'exec','script':'pos_hold_dvl_plugin.py',
                'torun':f'pids[{ind}].dump("{pid_type}_pid.json")'})

        if pid_type == 'depth':
            self.pub({'cmd':'exec','script':'depth_hold_plugin.py',
                'torun':'pid.dump("depth_pid.json")'})

           
    def pub(self,data):
        #print('sending command ...',data)
        self.pub_sock.send_multipart([zmq_topics.topic_remote_cmd,pickle.dumps(data,protocol=3)])
    #pickle.dump([time.time()-start_time,topic,data],joy_log,-1)

class rovDataHandler(object):
    def __init__(self, rovViewer,printer,args):
        self.subs_socks=[]
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand,zmq_topics.topic_system_state, zmq_topics.topic_lights],zmq_topics.topic_controller_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_button, zmq_topics.topic_hat], zmq_topics.topic_joy_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_imu], zmq_topics.topic_imu_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_dvl_raw], zmq_topics.topic_dvl_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_depth], zmq_topics.topic_depth_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_depth_hold_pid], zmq_topics.topic_depth_hold_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_sonar], zmq_topics.topic_sonar_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_sonar_hold_pid], zmq_topics.topic_sonar_hold_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_stereo_camera_ts], zmq_topics.topic_camera_ts_port)) #for sync perposes
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_main_cam_ts], zmq_topics.topic_main_cam_ts_port)) #for sync perposes
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_tracker], zmq_topics.topic_tracker_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_main_tracker], zmq_topics.topic_main_tracker_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_telem], zmq_topics.topic_telem_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_hw_stats], zmq_topics.topic_hw_stats_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_gps], zmq_topics.topic_gps_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_stereo_camera_calib], zmq_topics.topic_camera_calib_port))

        self.subs_socks.append(utils.subscribe([zmq_topics.topic_pos_hold_pid_fmt%i for i in range(3)], zmq_topics.topic_pos_hold_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_att_hold_yaw_pid,
                                          zmq_topics.topic_att_hold_pitch_pid,
                                          zmq_topics.topic_att_hold_roll_pid], zmq_topics.topic_att_hold_port))
            
        self.sub_vid=[]
        self.sub_vid.append(
                utils.subscribe([zmq_topics.topic_stereo_camera], zmq_topics.topic_camera_port)) #for sync perposes
        self.sub_vid.append(
                utils.subscribe([zmq_topics.topic_main_cam,
                    zmq_topics.topic_main_cam_depth,
                    zmq_topics.topic_main_cam_ts], zmq_topics.topic_main_cam_port)) #for sync perposes

        self.printer_sink = utils.pull_sink(zmq_topics.printer_sink_port)
        self.subs_socks.append(self.printer_sink)
        #self.subs_socks=[]
        self.syncedImages = [None,None] 
        self.curFrameId = -1
        self.curExposure = -1
        
        self.pubData = True
        self.socket_pub = None
        if self.pubData:
            self.socket_pub = utils.publisher(zmq_topics.topic_local_route_port,'0.0.0.0')
        self.rovViewer = rovViewer
        self.record_state=False
        self.telemtry = {}
        self.data_file_fd=None
        self.jm=Joy_map()
        self.pub_record_state = utils.publisher(zmq_topics.topic_record_state_port)
        self.args=args

        self.plot_buffers = {
            zmq_topics.topic_depth_hold_pid: CycArr(),
            zmq_topics.topic_att_hold_yaw_pid: CycArr(),
            zmq_topics.topic_att_hold_roll_pid: CycArr(),
            zmq_topics.topic_att_hold_pitch_pid: CycArr(),
            }
        for i in [0,1,2]:
            self.plot_buffers[zmq_topics.topic_pos_hold_pid_fmt%i]=CycArr()
        self.printer=printer
        self.main_image=None
        self.main_image_depth=None
        
    def getSincedImages(self,ind):
        ret = [self.curFrameId, self.syncedImages[ind]]
        if self.syncedImages[ind] is not None:
            self.syncedImages[ind] = None
        else:
            pass
        return ret

    def getNumSyncedImages(self):
        return len(self.syncedImages)

    def getMainImage(self):
        ret=self.main_image
        self.main_image=None
        return ret

    def getMainImageDepth(self):
        ret=self.main_image_depth
        self.main_image_depth=None
        return ret
     
    def getTelemtry(self):
        if self.telemtry is not None:
            return self.telemtry.copy()
        return None
   
    def process_video(self):
        images = [None, None]
        #sx,sy=config.cam_res_rgbx,config.cam_res_rgby
        bmargx,bmargy=config.viewer_blacks
        if not vid_zmq:
            #images = get_imgs()
            images = [stereo_cam_reader[i].get_img() for i in [0,1]]
            main_cam_img = main_cam_reader.get_img()
            main_cam_img_depth = main_cam_reader_depth.get_img()
            if self.main_image is None:
                self.main_image=main_cam_img
            if self.main_image_depth is None:
                self.main_image_depth=main_cam_img_depth
        else:
            while True:
                socks=select(self.sub_vid,[],[],0.003)[0]
                if len(socks)==0:
                    break
                for sock in socks:
                    ret=sock.recv_multipart()
                    if ret[0]==zmq_topics.topic_stereo_camera:
                        frame_cnt,shape = pickle.loads(ret[1])
                        images = []
                        for im in ret[2:]:
                            images.append(np.frombuffer(im,'uint8').reshape(shape).copy())
                        #print('======',len(images),ret[0])

                    if ret[0]==zmq_topics.topic_main_cam:
                        frame_main_cnt,shape = pickle.loads(ret[1])
                        self.main_image=np.frombuffer(ret[2],'uint8').reshape(shape).copy()
                        #print('got main image',self.main_image.shape)
                    if ret[0]==zmq_topics.topic_main_cam_depth:
                        _,scale_to_mm,shape = pickle.loads(ret[1])
                        if scale_to_mm is not None:
                            self.main_image_depth=im16to8_22(np.frombuffer(ret[2],'uint16').reshape(shape).astype('float32')*scale_to_mm)
                        else: #already converted
                            self.main_image_depth=np.frombuffer(ret[2],'uint8').reshape(shape).copy()


                #print('got main image depth',shape)

        if not vid_zmq:
            if self.record_state:
                if stereo_cam_reader[0].get_save_fd() is None:
                    print('start recording...')
                    #fds=[]
                    #datestr=sensor_gate_data['record_date_str']
                    datestr=self.record_state
                    save_path=self.args.data_path+'/'+datestr
                    if not os.path.isdir(save_path):
                        os.mkdir(save_path)
                    for i in [0,1]:
                        stereo_cam_reader[i].set_save_fd(open(save_path+'/vid_{}.mp4'.format('lr'[i]),'wb'))
                        #datestr=datetime.now().strftime('%y%m%d-%H%M%S')
                    main_cam_reader.set_save_fd(open(save_path+'/vid_main.mp4','wb'))   
                    main_cam_reader_depth.set_save_fd(open(save_path+'/vid_main_depth.mp4','wb'))   
                        #fds.append(open(save_path+'/vid_{}.mp4'.format('lr'[i]),'wb'))
                        
                    #set_files_fds(fds)
                    self.data_file_fd=open(save_path+'/viewer_data.pkl','wb')
                    pickle.dump([b'start_time',time.time()],self.data_file_fd,-1)
                    #os.system("gst-launch-1.0 -v -e udpsrc port=17894  ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! qtmux ! filesink location=%s sync=false & "%(save_path+'/main_cam.mov'))
            else:
                #if get_files_fds()[0] is not None:
                if stereo_cam_reader[0].get_save_fd() is not None:
                    print('done recording...')
                    #os.system('pkill -2 -f "gst-launch-1.0 -v -e udpsrc port=17894"')
                    #set_files_fds([None,None])
                    for i in [0,1]:
                        stereo_cam_reader[i].set_save_fd(None)
                    main_cam_reader.set_save_fd(None)
                    main_cam_reader_depth.set_save_fd(None)
                    self.data_file_fd=None

        if len(images)>0 and images[0] is not None:
            fmt_cnt_l=image_enc_dec.decode(images[0])
            for i in range(len(images)):
                if images[i] is not None:
                    self.syncedImages[i]=images[i]
            if images[0] is not None:
                draw_mono(images[0],self.telemtry,fmt_cnt_l)
                if len(images)>1 and images[1] is not None:
                    draw_seperate(images[0],images[1],self.telemtry)
                   
    def get_pos_xy(self):
        ldata=self.telemtry['dvl_deadrecon'] 
        ret=(ldata['x'],ldata['y'])
        return ret

    def get_pos_xy2(self): #shuld be the same as above but some how not BUG
        target_xy=[0,0]
        for i in [0,1]:
            pb=self.plot_buffers[zmq_topics.topic_pos_hold_pid_fmt%i]
            target_xy[i]=pb.get_last('N')
            if target_xy[i] is None:
                target_xy[i]=0
        return target_xy


    def get_target_xy(self):
        target_xy=[0,0]
        for i in [0,1]:
            pb=self.plot_buffers[zmq_topics.topic_pos_hold_pid_fmt%i]
            target_xy[i]=pb.get_last('T')
            if target_xy[i] is None:
                target_xy[i]=0
        return target_xy

    def get_track_range(self):
        if zmq_topics.topic_tracker in self.telemtry:
            trdata = self.telemtry[zmq_topics.topic_tracker]
            if trdata['valid']:
                return trdata['range']

    def get_track_dy(self):
        if zmq_topics.topic_tracker in self.telemtry:
            trdata = self.telemtry[zmq_topics.topic_tracker]
            if trdata['valid']:
                return trdata['dy']

    def get_main_track_pt(self):
        if zmq_topics.topic_main_tracker in self.telemtry:
            trdata = self.telemtry[zmq_topics.topic_main_tracker]
            #print('==trdata==',trdata)
            return trdata

    def get_depth(self):
        return self.telemtry[zmq_topics.topic_depth]['depth']

    def get_target_depth(self):
        return self.telemtry.get(zmq_topics.topic_depth_hold_pid,{}).get('T',0)

    def get_target_yaw(self):
        return self.telemtry.get(zmq_topics.topic_att_hold_yaw_pid,{}).get('T',0)

    def toggle_recording(self):
        if not self.record_state:
            self.record_state=datetime.now().strftime('%y%m%d-%H%M%S') 
        else:
            self.record_state=False
        self.pub_record_state.send_multipart([zmq_topics.topic_record_state,pickle.dumps(self.record_state)])
        self.telemtry[zmq_topics.topic_record_state]=self.record_state

        self.printer('record'+str(self.record_state))

    def process_telem(self):
        message_dict={}

        while True:
            socks = zmq.select(self.subs_socks,[],[],0.001)[0]
            if len(socks)==0: #flush msg buffer
                break
            for sock in socks:
                if sock==self.printer_sink:
                    data=sock.recv_pyobj()
                    print('got...',data)
                    self.printer(data['txt'],data['c'])
                else:
                    ret = sock.recv_multipart()
                    topic, data = ret
                    data_receive_ts=time.time()
                    data = pickle.loads(ret[1])
                    message_dict[topic] = data
                    self.telemtry.update(message_dict.copy())
                    if self.pubData:
                        self.socket_pub.send_multipart([ret[0],ret[1]])
                    #if zmq_topics.topic_tracker_result == topic:
                    #    #print('trck data res:', data)
                    #    try:
                    #        if data[1][0] < 0:
                    #            message_dict.pop(zmq_topics.topic_tracker_result)
                    #    except:
                    #        import traceback
                    #        traceback.print_exc()

                    if topic==zmq_topics.topic_button:
                        self.jm.update_buttons(data)
                        if self.jm.record_event(): 
                            #toggle recording
                            if not self.record_state:
                                self.record_state=datetime.now().strftime('%y%m%d-%H%M%S') 
                            else:
                                self.record_state=False
                        self.pub_record_state.send_multipart([zmq_topics.topic_record_state,pickle.dumps(self.record_state)])
                        message_dict[zmq_topics.topic_record_state]=self.record_state
                    
                    if topic==zmq_topics.topic_stereo_camera_calib:
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

                    if topic in self.plot_buffers:
                        self.plot_buffers[topic].add(data)

                    if topic==zmq_topics.topic_dvl_raw:
                        dvl_data = dvl_parse_line(data['dvl_raw']) 
                        if dvl_data is not None and dvl_data['type']=='deadreacon':
                            self.telemtry['dvl_deadrecon']=dvl_data
            

                    if self.data_file_fd is not None:
                        pickle.dump([topic,data_receive_ts,data],self.data_file_fd,-1)


    def next(self):
        self.process_video()
        self.process_telem()
            

           

