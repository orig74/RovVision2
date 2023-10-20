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


RES_X = 848
RES_Y = 480
FPS = 15

EXPOSURE = 600
GAIN = 20

SAVE_RATIO = 3 * FPS // 15
SEND_RATIO = 1 * FPS // 15
SAVE = False
IMSHOW = False
fd_frame_data=None


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
    intrinsics_depth = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    intrinsics_color = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    
    intel_cammtx = np.array([[intrinsics_depth.fx, 0, intrinsics_depth.ppx],
                                     [0, intrinsics_depth.fy, intrinsics_depth.ppy],
                                     [0, 0, 1]])
    print(intel_cammtx)


    depth_sensor = profile.get_device().first_depth_sensor()

    sens=pipeline.get_active_profile().get_device().query_sensors()[0]
    sens.set_option(rs.option.enable_auto_exposure, False)
    time.sleep(0.5)
    sens.set_option(rs.option.exposure, EXPOSURE)
    time.sleep(0.5)
    sens.set_option(rs.option.gain, GAIN)
    time.sleep(0.5)
    # 0: High Density, 1: Medium Density, 2: High Accuracy, 3: Hand, 4: Left Imager Color w/o IR Pattern, 5: Default
    sens.set_option(rs.option.visual_preset, 2)
    time.sleep(0.5)
    
    depth_scale = depth_sensor.get_depth_scale()

    frame_cnt = -1
    keep_frame_cnt = -1
    last_kept_num = -1

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
                record_state = ('/media/data/'+new_record_state_str+'/') if new_record_state_str else None
                if record_state and os.path.isdir(record_state):
                    fd_frame_data = open(record_state+'d405_frame_data.txt', 'w')
                    open(record_state+'camera_main.txt','w').write(str(intrinsics_color))
                    open(record_state+'camera_depth.txt','w').write(str(intrinsics_depth))
            if topic==zmq_topics.topic_remote_cmd:
                if data['cmd']=='d405param':
                    if 'exposure' in data:
                        print('setting setting d405 exposure',data)
                        #exp=max(500, min(data['exposure']*1000, 10e3)) 
                        #todo change it to gain
                        sens.set_option(rs.option.gain, max(0,min(data['exposure'], 1000))) 
                        # sens.set_option(rs.option.exposure, exp)  # Set exposure to inter-frame time
 
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        depth_frame = frames.get_depth_frame()
        #grey_l_frame = frames.get_infrared_frame(1)
        #grey_r_frame = frames.get_infrared_frame(2)

        frame_cnt += 1
        keep_frame_cnt = frame_cnt

        depth_frame_raw=np.array(depth_frame.get_data())
        depth_float_raw = depth_frame_raw.astype(np.float32)
        depth_img_m = depth_float_raw * depth_scale
        col_img = np.array(color_frame.get_data())
        #grey_l = np.array(grey_l_frame.get_data())
        time_stamp=time.time()
        #grey_r = np.array(grey_r_frame.get_data())

        if frame_cnt%(2*FPS)==0 and frame_cnt>100:
            print(f'frame_cnt={frame_cnt},keep_frame_cnt={keep_frame_cnt},realfps={FPS*keep_frame_cnt/frame_cnt}')

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
            depth_img_m[depth_img_m > 2.0] = 0
            cv2.imshow("Depth cam image", depth_img_m)
            cv2.imshow("Colour", col_img)
            # cv2.imshow("Grey l", grey_l)
            #cv2.imshow("Grey r", grey_r)
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()
    pipeline.stop()
