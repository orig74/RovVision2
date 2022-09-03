import zmq_topics
import zmq_wrapper as utils
import config

vid_zmq = config.is_sim and config.sim_type=='PB'
if not vid_zmq:
    from gst import init_gst_reader,get_imgs,set_files_fds,get_files_fds,save_main_camera_stream

class rovDataHandler(object):
    def __init__(self, rovViewer):
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
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_tracker], zmq_topics.topic_tracker_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_telem], zmq_topics.topic_telem_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_hw_stats], zmq_topics.topic_hw_stats_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_gps], zmq_topics.topic_gps_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_stereo_camera_calib], zmq_topics.topic_camera_calib_port))

        self.subs_socks.append(utils.subscribe([zmq_topics.topic_pos_hold_pid_fmt%i for i in range(3)], zmq_topics.topic_pos_hold_port))
        self.subs_socks.append(utils.subscribe([zmq_topics.topic_att_hold_yaw_pid,
                                          zmq_topics.topic_att_hold_pitch_pid,
                                          zmq_topics.topic_att_hold_roll_pid], zmq_topics.topic_att_hold_port))
            
        self.sub_vid=utils.subscribe([zmq_topics.topic_stereo_camera], zmq_topics.topic_camera_port) #for sync perposes
        #self.subs_socks=[]
        self.image = None 
        self.curFrameId = -1
        self.curExposure = -1
        
        self.pubData = True
        self.socket_pub = None
        if self.pubData:
            self.socket_pub = utils.publisher(zmq_topics.topic_local_route_port,'0.0.0.0')
        self.rovViewer = rovViewer
        self.record_state=False
        self.telemtry = {}
        
        
    def getNewImage(self):
        ret = [self.curFrameId, None]
        if self.image is not None:
            ret = [self.curFrameId, np.copy(self.image)]
            self.image = None
            #print("---image---", time.time())
        else:
            pass
            #print(time.time(), "no image")
            #print('--->', ret[0])
        return ret
    
    def getTelemtry(self):
        if self.telemtry is not None:
            return self.telemtry.copy()
        return None
   
    def process_video(self):
        images = [None, None]
        sx,sy=config.cam_res_rgbx,config.cam_res_rgby
        bmargx,bmargy=config.viewer_blacks
        if not vid_zmq:
            images = get_imgs()
        else:
            while len(select([self.sub_vid],[],[],0.003)[0]) > 0:
                ret=sub_vid.recv_multipart()
                frame_cnt,shape = pickle.loads(ret[1])
                images = []
                for im in ret[2:]:
                    images.append(np.frombuffer(im,'uint8').reshape(shape).copy())
                print('======',len(images),ret[0])
                if len(images)>0:
                    break
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

        if images[0] is not None:
            fmt_cnt_l=image_enc_dec.decode(images[0])
            draw_mono(images[0],message_dict,fmt_cnt_l)
 
    def process_telem(self):
        message_dict={}
        rcv_cnt=0
        

        while True:
            socks = zmq.select(self.subs_socks,[],[],0.001)[0]
            if len(socks)==0: #flush msg buffer
                break
            for sock in socks:
                ret = sock.recv_multipart()
                topic, data = ret
                data = pickle.loads(ret[1])
                message_dict[topic] = data
                self.telemtry = message_dict.copy()
                
                
                if self.pubData:
                    self.socket_pub.send_multipart([ret[0],ret[1]])
                
                if zmq_topics.topic_tracker_result == topic:
                    #print('trck data res:', data)
                    try:
                        if data[1][0] < 0:
                            message_dict.pop(zmq_topics.topic_tracker_result)
                    except:
                        import traceback
                        traceback.print_exc()

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

        

            if data_file_fd is not None:
                pickle.dump([topic,data],data_file_fd,-1)



    def next(self):
        self.process_video(self)
        self.process_telem(self)
            

           

