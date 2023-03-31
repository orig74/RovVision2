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
import zmq_topics
import zmq_wrapper as utils
subs_socks=[]
subs_socks.append(utils.subscribe([ zmq_topics.topic_record_state ],zmq_topics.topic_record_state_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))
socket_pub = utils.publisher(zmq_topics.topic_camera_port)
socket_pub_ts = utils.publisher(zmq_topics.topic_camera_ts_port)
socket_pub_telem = utils.publisher(zmq_topics.topic_camera_telem_port)


#WRITE_DIR = '/local/D405/' #'/home/uav/data/D405/'

FRAME_MOD = 1

FPS = 90
RES_X = 640
RES_Y = 480

KEEP_STROBE_FRAMES = True
SAVE_RATIO = 3
SEND_RATIO = 3
SAVE = False
IMSHOW = False
DEPTH_THRESH = 1.0

if __name__ == "__main__":
    record_state=None
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, RES_X, RES_Y, rs.format.bgr8, FPS)
    config.enable_stream(rs.stream.depth, RES_X, RES_Y, rs.format.z16, FPS)
    #config.enable_stream(rs.stream.infrared, 1, RES_X, RES_Y, rs.format.y8, FPS)
    #config.enable_stream(rs.stream.infrared, 2, RES_X, RES_Y, rs.format.y8, FPS)
    #config.enable_all_streams()
    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.enable_auto_exposure, False)
    depth_sensor.set_option(rs.option.exposure, 11111)  # Set exposure to inter-frame time
    depth_sensor.set_option(rs.option.gain, 16)
    depth_scale = depth_sensor.get_depth_scale()

    frame_cnt = -1
    keep_frame_cnt = -1
    avg_val = 0
    last_kept_ts = time.time()
    while True:
        socks=zmq.select(subs_socks,[],[],0)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])
            if topic==zmq_topics.topic_record_state:
                new_record_state_str=data
                if not record_state and new_record_state_str:
                    #switch to recording
                    os.mkdir('/media/data/'+new_record_state_str)
                    #calibrator.ParamsUpdateFlag = True
                record_state=('/media/data/'+new_record_state_str+'/') if new_record_state_str else None
                if record_state:
                    fd_frame_data=open(record_state+'/frame_data.txt','w')
 
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        #print(1000*time.time() - color_frame.get_timestamp())
        #print()

        depth_frame = frames.get_depth_frame()
        grey_l_frame = frames.get_infrared_frame(1)
        #grey_r_frame = frames.get_infrared_frame(2)

        frame_cnt += 1
        if frame_cnt % FRAME_MOD != 0:
            continue

        depth_frame_raw=depth_frame.get_data()
        depth_float_raw = np.array(depth_frame_raw).astype(np.float32)
        depth_img_m = depth_float_raw * depth_scale
        col_img = np.array(color_frame.get_data())
        grey_l = np.array(grey_l_frame.get_data())
        #grey_r = np.array(grey_r_frame.get_data())

        val = float(grey_l.mean())
        avg_val = avg_val * 0.8 + val * 0.2
        elapsed_time = time.time() - last_kept_ts
        if elapsed_time < 0.09:
            continue
        if KEEP_STROBE_FRAMES and val < 1.0 * avg_val and elapsed_time < 0.12:
            continue
        last_kept_ts = time.time()
        keep_frame_cnt += 1

        if record_state and keep_frame_cnt%SAVE_RATIO==0:
            fd_frame_data.write(f'{keep_frame_cnt},{time.time()},{depth_scale}')
            cv2.imwrite(record_state + f'{keep_frame_cnt:06d}.jpg',col_img)
            open(record_state+'d{keep_frame_cnt:06d}.bin','wb').write(depth_frame_raw)

        if keep_frame_cnt%SEND_RATIO==0:
            socket_pub.send_multipart([zmq_topics.topic_main_cam,
                pickle.dumps((keep_frame_cnt,col_img.shape)),col_img.tobytes()])
            scale_to_mm=depth_scale
            socket_pub_ts.send_multipart([zmq_topics.topic_main_cam_depth,
                pickle.dumps((keep_frame_cnt,scale_to_mm,depth_frame.shape)),depth_frame_raw])

            #cv2.imwrite(record_state + 'greyl_' + str(keep_frame_cnt) + '.jpeg', grey_l)
            #cv2.imwrite(record_state + 'greyr_' + str(keep_frame_cnt) + '.jpeg', grey_r)
            #depth_img_U8 = (np.clip(depth_img_m, 0, 1.0) * 255).astype(np.uint8)
            #cv2.imwrite(record_state + 'depthU8_' + str(keep_frame_cnt) + '.jpeg', depth_img_U8)
            #np.save(record_state + 'depthF32_' + str(keep_frame_cnt) + '.npy', depth_img_m)

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
