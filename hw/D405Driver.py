"""Fuse 1000 RGB-D images from the 7-scenes dataset into a TSDF voxel volume with 2cm resolution.
"""
import sys,os
sys.path.append('../')
sys.path.append('../utils')
import time
import cv2
import pickle
import numpy as np
import pyrealsense2 as rs
import zmq
import signal
import zmq_topics
import zmq_wrapper as utils
subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_record_state],zmq_topics.topic_record_state_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))
socket_pub = utils.publisher(zmq_topics.topic_main_cam_port)
socket_pub_ts=utils.publisher(zmq_topics.topic_main_cam_ts_port)


#WRITE_DIR = '/local/D405/' #'/home/uav/data/D405/'
#time.sleep(6)
FRAME_MOD = 1

RES_X = 848
RES_Y = 480

KEEP_STROBE_FRAMES = 1 #1 keep strob 2 dont keep strob 3 keepall

FPS = 15 if KEEP_STROBE_FRAMES==1 else 15#90
MIN_GAP_BETWEEN_KEEPS = 1 *FPS//15 #dectates maxmial keep frequency
MAX_GAP_BETWEEN_KEEPS = 10 * FPS//15 #dectates minimal keep frequency
SAVE_RATIO = 3 *FPS//15
SEND_RATIO = 1 *FPS//15
SAVE = False
IMSHOW = False
DEPTH_THRESH = 1.0
fd_frame_data=None

class BrightDetector(object):
    def __init__(self,ws=FPS):
        self.buf=[0] * ws

    def add(self,val):
        self.buf.pop(0)
        self.buf.append(val)

    def is_bright(self,val):
        min_g=abs(val-np.min(self.buf))
        max_g=abs(val-np.max(self.buf))
        #return abs(val-np.min(self.buf))>abs(val-np.max(self.buf))
        return min_g>max_g

if __name__ == "__main__":
    record_state=None
    pipeline = rs.pipeline()
    keep_run=True

    def handler(signum, frame):
        global keep_run
        msg = "Ctrl-c was pressed"
        print(msg, flush=True)
        time.sleep(1)
        keep_run=False
     
    signal.signal(signal.SIGINT, handler)

    config = rs.config()
    config.enable_stream(rs.stream.color, RES_X, RES_Y, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, RES_X, RES_Y, rs.format.z16, FPS)
    #config.enable_stream(rs.stream.infrared, 1, RES_X, RES_Y, rs.format.y8, FPS)
    #config.enable_stream(rs.stream.infrared, 2, RES_X, RES_Y, rs.format.y8, FPS)
    #config.enable_all_streams()


    profile = pipeline.start(config)
    intrinsics_depth=profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    intrinsics_color=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

    depth_sensor = profile.get_device().first_depth_sensor()

    sens=pipeline.get_active_profile().get_device().query_sensors()[0]
    sens.set_option(rs.option.enable_auto_exposure, False)
    time.sleep(1.0)
    sens.set_option(rs.option.exposure, int(1e6//FPS))  # Set exposure to inter-frame time
    time.sleep(1.0)
    sens.set_option(rs.option.gain, 10)
    time.sleep(1.0)
    
    depth_scale = depth_sensor.get_depth_scale()

    frame_cnt = -1
    keep_frame_cnt = -1
    avg_val = 0
    #last_kept_ts = time.time()
    last_kept_num = -1
    bd=BrightDetector()

    while keep_run:
        socks=zmq.select(subs_socks,[],[],0)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])
            if topic==zmq_topics.topic_record_state:
                new_record_state_str=data
                #if not record_state and new_record_state_str:
                    #switch to recording
                    #os.mkdir('/media/data/'+new_record_state_str)
                    #calibrator.ParamsUpdateFlag = True
                record_state=('/media/data/'+new_record_state_str+'/') if new_record_state_str else None
                if record_state and os.path.isdir(record_state):
                    fd_frame_data=open(record_state+'d405_frame_data.txt','w')
                    open(record_state+'camera_main.txt','w').write(str(intrinsics_color))
                    open(record_state+'camera_depth.txt','w').write(str(intrinsics_depth))
            if topic==zmq_topics.topic_remote_cmd:
                if data['cmd']=='strob_mode':
                    print('setting strob mode',data)
                    KEEP_STROBE_FRAMES=data['keep_mode'] 
                if data['cmd']=='d405param':
                    if 'exposure' in data:
                        print('setting setting d405 exposure',data)
                        exp=min(data['exposure']*1000,int(1e6//FPS)) 
                        sens.set_option(rs.option.exposure, exp)  # Set exposure to inter-frame time
 
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        #print(1000*time.time() - color_frame.get_timestamp())

        depth_frame = frames.get_depth_frame()
        #grey_l_frame = frames.get_infrared_frame(1)
        #grey_r_frame = frames.get_infrared_frame(2)

        frame_cnt += 1
        if frame_cnt % FRAME_MOD != 0:
            continue

        depth_frame_raw=np.array(depth_frame.get_data())
        depth_float_raw = depth_frame_raw.astype(np.float32)
        depth_img_m = depth_float_raw * depth_scale
        col_img = np.array(color_frame.get_data())
        #grey_l = np.array(grey_l_frame.get_data())
        time_stamp=time.time()
        #grey_r = np.array(grey_r_frame.get_data())

        if frame_cnt%(2*FPS)==0 and frame_cnt>100:
            print(f'frame_cnt={frame_cnt},keep_frame_cnt={keep_frame_cnt},realfps={FPS*keep_frame_cnt/frame_cnt}')
            print('::',list(map(int,bd.buf)),np.min(bd.buf),np.max(bd.buf))
            print('::',[1 if bd.is_bright(i) else 0 for i in bd.buf])
            #print(color_frame.get_frame_metadata(rs.frame_metadata_value.actual_exposure))

        val = float(col_img.mean())
        bd.add(val)
        keep_gap = frame_cnt-last_kept_num
        if KEEP_STROBE_FRAMES==1:
            if keep_gap<MIN_GAP_BETWEEN_KEEPS:
                continue
            if not bd.is_bright(val) and keep_gap<MAX_GAP_BETWEEN_KEEPS:
                continue
        if KEEP_STROBE_FRAMES==2: #keep dark
            if bd.is_bright(val) and keep_gap<MAX_GAP_BETWEEN_KEEPS:
                continue
        last_kept_num = frame_cnt
        keep_frame_cnt += 1
        #print('===',bd.is_bright(val))

        #avg_val = avg_val * 0.8 + val * 0.2
        #elapsed_time = time.time() - last_kept_ts
        #if elapsed_time < 0.1:
        #    continue
        ##if KEEP_STROBE_FRAMES and val < 1.0 * avg_val and elapsed_time < 0.14:
        #    continue
        #last_kept_ts = time.time()
        if record_state and keep_frame_cnt%SAVE_RATIO==0:
            if os.path.isdir(record_state) and fd_frame_data is not None:
                fd_frame_data.write(f'{keep_frame_cnt},{time_stamp},{depth_scale}\n')
                cv2.imwrite(record_state + f'{keep_frame_cnt:06d}.jpg',col_img)
                open(record_state+f'd{keep_frame_cnt:06d}.bin','wb').write(depth_frame_raw.tobytes())
                fd_frame_data.flush()
            else:
                print("Write directory doesnt exist!")

        if keep_frame_cnt%SEND_RATIO==0:
            socket_pub.send_multipart([zmq_topics.topic_main_cam,
                pickle.dumps((keep_frame_cnt,col_img.shape)),col_img.tobytes()])
            scale_to_mm=depth_scale*1000
            socket_pub.send_multipart([zmq_topics.topic_main_cam_depth,
                pickle.dumps((keep_frame_cnt,scale_to_mm,depth_frame_raw.shape)),depth_frame_raw.tostring()])
            socket_pub_ts.send_multipart([zmq_topics.topic_main_cam_ts,pickle.dumps((
                keep_frame_cnt,time_stamp,depth_frame.get_timestamp(),color_frame.get_timestamp()))])
            #cv2.imwrite(record_state + 'greyl_' + str(keep_frame_cnt) + '.jpeg', grey_l)
            #cv2.imwrite(record_state + 'greyr_' + str(keep_frame_cnt) + '.jpeg', grey_r)
            #depth_img_U8 = (np.clip(depth_img_m, 0, 1.0) * 255).astype(np.uint8)
            #cv2.imwrite(record_state + 'depthU8_' + str(keep_frame_cnt) + '.jpeg', depth_img_U8)
            #np.save(record_state + 'depthF32_' + str(keep_frame_cnt) + '.npy', depth_img_m)

        #print('===',col_img.shape,depth_img_m.shape)

        if IMSHOW:
            depth_img_m[depth_img_m > DEPTH_THRESH] = 0
            cv2.imshow("Depth cam image", depth_img_m)
            cv2.imshow("Colour", col_img)
            cv2.imshow("Grey l", grey_l)
            #cv2.imshow("Grey r", grey_r)
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
    pipeline.stop()
