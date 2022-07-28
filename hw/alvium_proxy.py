import argparse,sys,os,time,pickle
sys.path.append('../')
sys.path.append('../utils')
import config
import copy
import threading
import queue
import time
from vimba import *
import numpy as np
import random
import cv2
import sys
from datetime import datetime
from typing import Optional
import math
import gc
import zmq
import zmq_topics
import zmq_wrapper as utils
subs_socks=[]
subs_socks.append(utils.subscribe([ zmq_topics.topic_record_state ],zmq_topics.topic_record_state_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_system_state],zmq_topics.topic_controller_port))
socket_pub = utils.publisher(zmq_topics.topic_camera_port)
socket_pub_ts = utils.publisher(zmq_topics.topic_camera_ts_port)
socket_pub_telem = utils.publisher(zmq_topics.topic_camera_telem_port)

parser = argparse.ArgumentParser()
parser.add_argument("--debug", help="Show frames with opencv", action='store_true')
args = parser.parse_args()

CAM_IDS = ['DEV_000F315DAB37', 'DEV_000F315DB084', 'DEV_000F315DAB68'] # 
MASTER_CAM_ID = CAM_IDS[0]
NUM_CAMS = len(CAM_IDS)

# Min exposure time: 32us
CAM_EXPOSURE_US = None #10000
CAM_EXPOSURE_MAX = 1000    # us
CAM_EXPOSURE_MIN = 32

IMG_SIZE_BYTES = 5065984

# If changing need to power cycle cameras, no vertical binning, cannot change while streaming
BIN_HORIZONTAL = 1
DEC_HORIZONTAL = 1
DEC_VERTICAL = 1
BINDEC_IMG_SIZE = IMG_SIZE_BYTES / (BIN_HORIZONTAL * DEC_HORIZONTAL * DEC_VERTICAL)

MAX_BANDWIDTH_BYTES_P_S = 110e6
FRAME_TRANSFER_TIME = NUM_CAMS * BINDEC_IMG_SIZE / MAX_BANDWIDTH_BYTES_P_S

FRAME_QUEUE_SIZE = 11

ACTION_DEV_KEY = 1
ACTION_GROUP_KEY = 1
ACTION_GROUP_MASK = 1

def fast_bayer_shrink(dest,img):
    dest[:,:,2]=np.squeeze(img[::2,::2])
    dest[:,:,1]=np.squeeze(img[1::2,0::2])
    dest[:,:,0]=np.squeeze(img[1::2,1::2])

# --------------------------------------- Alvium Multicam Driver ---------------------------------------------

class AlviumMultiCam(threading.Thread):
    def __init__(self):
        super().__init__()
        self.loop_fps = 500
        self.producers = {}
        self.frame_queue = queue.Queue(maxsize=FRAME_QUEUE_SIZE)
        self.producers_lock = threading.Lock()
        self.cached_images=None
        self.debug_cam_idx=0

    def __call__(self, cam: Camera, event: CameraEvent):
        # An existing camera was disconnected, stop associated FrameProducer.
        if event == CameraEvent.Missing:
            with self.producers_lock:
                producer = self.producers.pop(cam.get_id())
                producer.stop()
                producer.join()
        # New camera was detected. Create FrameProducer, add it to active FrameProducers
        elif event == CameraEvent.Detected:
            with self.producers_lock:
                self.producers[cam.get_id()] = FrameProducer(cam, self.frame_queue)
                self.producers[cam.get_id()].start()

    def run(self):
        system_state = 'INIT'
        frame = None

        if args.debug:
            def on_change(val):
                self.debug_cam_idx = val
            cv2.namedWindow("DEBUG WINDOW", cv2.WINDOW_GUI_NORMAL)
            cv2.createTrackbar("Cam IDX", "DEBUG WINDOW", 0, 3, on_change)

        while system_state != 'STOP':
            vimba_ctx = Vimba.get_instance()
            with vimba_ctx:
                # Construct FrameProducer threads for all detected cameras
                for cam in vimba_ctx.get_all_cameras():
                    self.producers[cam.get_id()] = FrameProducer(cam, self.frame_queue)

                # Start FrameProducer threads
                with self.producers_lock:
                    for producer in self.producers.values():
                        producer.start()

                # Start and respond to state changes
                vimba_ctx.register_camera_change_handler(self)

                # start trigger thread only once all cameras are setup
                start_ts = time.time()
                while system_state == 'INIT':
                    all_streaming = True
                    for k, prod in self.producers.items():
                        all_streaming &= prod.cam.is_streaming()
                        if not prod.is_alive():
                            system_state = 'RESTART'
                            break
                    if all_streaming:
                        system_state = 'RUN'
                    time.sleep(0.1)
                    if time.time() - start_ts > 5:
                        system_state = 'STOP'

                if system_state == 'RUN':
                    trigger_thread = TriggerThread(fps=config.cam_fps)
                    trigger_thread.start()
                    print()

                    max_rx_delay = 0
                    max_rx_frame = 0
                    rx_delay_sum = 0
                    proc_delay_sum = 0
                    num_frames = 0
                    max_proc_delay = 0

                    prev_loop_time = time.time()
                    loop_cnt = 0
                    blank_img = np.zeros((200, 200, 3), dtype=np.uint8)
                    current_frames = dict(zip(CAM_IDS, NUM_CAMS*[blank_img]))
                    current_frame_ts = dict(zip(CAM_IDS, [i for i in range(NUM_CAMS)]))
                    prev_syncd_trig_ts = time.time()

                    total_syncd_frames = 0
                    total_frames_through_buff = 0
                    while system_state == 'RUN':
                        frames_left = self.frame_queue.qsize()
                        total_frames_through_buff += frames_left
                        assert frames_left < FRAME_QUEUE_SIZE
                        while frames_left:
                            try:
                                cam_id, rx_ts, frame = self.frame_queue.get_nowait()

                                proc_delay = time.time() - rx_ts
                                prev_trigger_ts_l = trigger_thread.prev_trigger_ts.copy()
                                for prev_trig_ts in prev_trigger_ts_l:
                                    rx_delay = rx_ts - prev_trig_ts
                                    if rx_delay > 0.75*FRAME_TRANSFER_TIME:
                                        break
                                assert rx_delay > 0
                                rx_delay_sum += rx_delay
                                proc_delay_sum += proc_delay
                                num_frames += 1
                                if (rx_delay > max_rx_delay):
                                    max_rx_delay = rx_delay
                                    max_rx_frame = num_frames
                                max_proc_delay = max(max_proc_delay, proc_delay) if system_state == 'RUN' else max_proc_delay

                                current_frame_ts[cam_id] = prev_trig_ts
                                current_frames[cam_id] = frame

                                # Syncd frames check
                                ts_vals = list(current_frame_ts.values())
                                if all(ts == ts_vals[0] for ts in ts_vals):
                                    if ts_vals[0] != prev_syncd_trig_ts:
                                        #print(ts_vals[0] - prev_syncd_trig_ts)
                                        prev_syncd_trig_ts = ts_vals[0]
                                        total_syncd_frames += 1
                                        imgl,imgr = [frm for c_id, frm in current_frames.items()][:2]
                                        if self.cached_images is None:
                                            self.cached_images = \
                                                [np.zeros((imgl.shape[0]//2,imgl.shape[1]//2,3),dtype='uint8') for _ in [0,1]]
                                        rgbl,rgbr = self.cached_images
                                        fast_bayer_shrink(rgbl,imgl)
                                        fast_bayer_shrink(rgbr,imgr)
                                        RS_DIV = 4
                                        rgbl_rs = cv2.resize(rgbl, (imgl.shape[1]//RS_DIV, imgl.shape[0]//RS_DIV))
                                        rgbr_rs = cv2.resize(rgbr, (imgl.shape[1]//RS_DIV, imgl.shape[0]//RS_DIV))
                                        #print('shape sent is..:',rgbl_rs.shape) 
                                        socket_pub.send_multipart([zmq_topics.topic_stereo_camera,
                                            pickle.dumps((total_syncd_frames,rgbl_rs.shape)),rgbl_rs.tobytes(),rgbr_rs.tobytes()])
                                        socket_pub_ts.send_multipart([zmq_topics.topic_stereo_camera_ts,
                                            pickle.dumps((total_syncd_frames,prev_syncd_trig_ts,time.time()))])
                                        if args.debug and total_syncd_frames%1==0:
                                            if self.debug_cam_idx == 3:
                                                frame_bgr_0 = cv2.cvtColor(current_frames[CAM_IDS[0]], cv2.COLOR_BayerBG2BGR)
                                                frame_bgr_1 = cv2.cvtColor(current_frames[CAM_IDS[1]], cv2.COLOR_BayerBG2BGR)
                                                frame_bgr_2 = cv2.cvtColor(current_frames[CAM_IDS[2]], cv2.COLOR_BayerBG2BGR)
                                                frame_bgr = frame_bgr_0.astype(np.float32) + frame_bgr_1.astype(np.float32) + frame_bgr_2.astype(np.float32)
                                                frame_bgr = (255 * frame_bgr / np.max(frame_bgr)).astype(np.uint8)
                                            else:
                                                debug_cam_key = CAM_IDS[min(self.debug_cam_idx, NUM_CAMS-1)]
                                                frame_bgr = cv2.cvtColor(current_frames[debug_cam_key], cv2.COLOR_BayerBG2BGR)
                                            cv2.imshow("DEBUG WINDOW", frame_bgr)
                                            if cv2.waitKey(1) == ord('q'):
                                                trigger_thread.alive = False
                                                system_state = 'STOP'
                                                time.sleep(1.0)
                                                cv2.destroyAllWindows()
                                        if record_state is not None:
                                            cur_rec_state = record_state
                                            for cam_key, cam_frame in current_frames.items():
                                                cv2.imwrite(cur_rec_state + str(total_syncd_frames)
                                                            + '_' + cam_key + '.pgm', cam_frame)
                                    else:
                                        print("Duplicate frame detected!")

                            except queue.Empty:
                                break
                            frames_left -= 1

                        # Get device stats
                        if loop_cnt % 400 == 0:
                            cam_key = random.choice(self.producers.keys())
                            cam = self.producers[cam_key]
                            try:
                                temp = cam.get_feature_by_name('DeviceTemperature').get()
                                exposure = cam.get_feature_by_name('ExposureTimeAbs').get()
                                gain = cam.get_feature_by_name('Gain').get()
                                balance = cam.get_feature_by_name('BalanceRatioAbs').get()
                                print("{} Temp: {} degrees C".format(k, round(temp, 2)))
                                to_send = {'Temp': temp, 'Exp_us': exposure, 'Gain': gain, 'WB': balance}
                                socket_pub_telem.send_multipart([zmq_topics.topic_camera_telem,
                                                                 pickle.dumps((time.time(),to_send))])
                            except:
                                print("Get cam telem failed!")

                        while (time.time() - prev_loop_time < 1 / self.loop_fps):
                            time.sleep(0.001)
                        prev_loop_time = time.time()

                        loop_cnt += 1

                    print("AVG frame rx delay: {}".format(rx_delay_sum / num_frames))
                    print("MAX frame rx delay: {} on frame {}".format(max_rx_delay, max_rx_frame))
                    print("AVG processing delay: {}".format(proc_delay_sum / num_frames))
                    print("MAX processing delay: {}".format(max_proc_delay))
                    print("Total frames recieved: {}".format(total_frames_through_buff))
                    print("Total synced frames: {}".format(total_syncd_frames * NUM_CAMS))
                    print("Total trigger signals: {}".format(trigger_thread.total_num_trigs * NUM_CAMS))

                    trigger_thread.alive = False
                    trigger_thread.join()

                vimba_ctx.unregister_camera_change_handler(self)

                # Stop all FrameProducer threads
                with self.producers_lock:
                    producer_keys = list(self.producers.keys()).copy()
                    for prod_key in producer_keys:
                        self.producers[prod_key].stop()
                        self.producers[prod_key].join()
                        self.producers.pop(prod_key)

            del vimba_ctx
            gc.collect()


class TriggerThread(threading.Thread):
    def __init__(self, fps):
        super().__init__()
        self.fps = fps
        self.alive = True
        self.prev_trigger_ts = 4 * [0.0]
        self.total_num_trigs = 0

    def print_usage(self):
        print('Usage:')
        print('    python action_commands.py <camera_id> <interface_id>')
        print('    python action_commands.py [/h] [-h]')
        print()
        print('Parameters:')
        print('    camera_id      ID of the camera to be used')
        print('    interface_id   ID of network interface to send out Action Command')
        print('                   \'ALL\' enables broadcast on all interfaces')
        print()

    def abort(self, reason: str, return_code: int = 1, usage: bool = False):
        print(reason + '\n')
        if usage:
            self.print_usage()
        sys.exit(return_code)

    def get_command_sender(self, interface_id):
        # If given interface_id is ALL, ActionCommand shall be sent from all Ethernet Interfaces.
        # This is achieved by run ActionCommand on the Vimba instance.
        if interface_id == 'ALL':
            return Vimba.get_instance()
        with Vimba.get_instance() as vimba:
            # A specific Interface was given. Lookup via given Interface id and verify that
            # it is an Ethernet Interface. Running ActionCommand will be only send from this Interface.
            try:
                inter = vimba.get_interface_by_id(interface_id)
            except VimbaInterfaceError:
                self.abort('Failed to access Interface {}. Abort.'.format(interface_id))
            if inter.get_type() != InterfaceType.Ethernet:
                self.abort('Given Interface {} is no Ethernet Interface. Abort.'.format(interface_id))
        return inter

    def run(self):
        TOA_INTERFACE_ID = 'ALL'
        sender = self.get_command_sender(TOA_INTERFACE_ID)
        with sender:
            sender.ActionDeviceKey.set(ACTION_DEV_KEY)
            sender.ActionGroupKey.set(ACTION_GROUP_KEY)
            sender.ActionGroupMask.set(ACTION_GROUP_MASK)
            while self.alive:
                if time.time() - self.prev_trigger_ts[0] > 1 / self.fps:
                    #print(1 / (time.time() - prev_trig_time))
                    self.prev_trigger_ts[3] = self.prev_trigger_ts[2]
                    self.prev_trigger_ts[2] = self.prev_trigger_ts[1]
                    self.prev_trigger_ts[1] = self.prev_trigger_ts[0]
                    self.prev_trigger_ts[0] = time.time()
                    sender.ActionCommand.run()
                    self.total_num_trigs += 1
                time.sleep(0.002)


# ----------------------------------Alvium Frame Producer ----------------------------------------------

class FrameProducer(threading.Thread):
    def __init__(self, cam: Camera, frame_queue: queue.Queue):
        super().__init__()
        self.cam = cam
        self.cam_id = cam.get_id()
        self.frame_queue = frame_queue
        self.killswitch = threading.Event()

    def try_put_frame(self, frame: Optional[Frame], time_stamp):
        try:
            self.frame_queue.put_nowait((self.cam_id, time_stamp, frame))

        except queue.Full:
            pass

    def __call__(self, cam: Camera, frame: Frame):
        try:
            if frame.get_status() == FrameStatus.Complete:
                ts = time.time()
                frame_cpy = copy.deepcopy(frame)
                cv_frame_bay = frame_cpy.as_numpy_ndarray()
                #cv_frame = cv2.cvtColor(cv_frame, cv2.COLOR_BAYER_RG2RGB)
                #cv_frame = cv2.rotate(cv_frame, cv2.ROTATE_90_CLOCKWISE)
                #cv2.putText(cv_frame, str(datetime.now()), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 255), 3, cv2.LINE_AA)
                if all(cv_frame_bay.shape):
                    self.try_put_frame(cv_frame_bay, ts)
            cam.queue_frame(frame)
        except Exception as e:
            print(e)
            self.stop()

    def stop(self):
        self.killswitch.set()

    def setup_camera(self):
        if (config.cam_fps > 1 / FRAME_TRANSFER_TIME):
            print("WARNING! Requested FPS exceeds data transfer speeds")
        self.cam.get_feature_by_name('StreamBytesPerSecond').set(MAX_BANDWIDTH_BYTES_P_S // NUM_CAMS)

        self.cam.get_feature_by_name('BinningHorizontal').set(BIN_HORIZONTAL)
        self.cam.get_feature_by_name('DecimationHorizontal').set(DEC_HORIZONTAL)
        self.cam.get_feature_by_name('DecimationVertical').set(DEC_VERTICAL)
        
        if CAM_EXPOSURE_US:
            self.cam.get_feature_by_name('ExposureAuto').set('Off')
            self.cam.get_feature_by_name('ExposureTimeAbs').set(CAM_EXPOSURE_US)
        
        else:
            self.cam.get_feature_by_name('ExposureAuto').set('Continuous')
            self.cam.get_feature_by_name('ExposureAutoMax').set(CAM_EXPOSURE_MAX)
            self.cam.get_feature_by_name('ExposureAutoMin').set(CAM_EXPOSURE_MIN)
        self.cam.get_feature_by_name('GainAuto').set('Continuous')
        self.cam.get_feature_by_name('BalanceWhiteAuto').set('Continuous')

        self.cam.set_pixel_format(PixelFormat.BayerRG8)  # PixelFormat.Bgr8
        self.cam.get_feature_by_name('AcquisitionMode').set('Continuous')

        self.cam.get_feature_by_name('TriggerSelector').set('FrameStart')
        self.cam.get_feature_by_name('TriggerSource').set('Action0')
        self.cam.get_feature_by_name('TriggerMode').set('On')
        self.cam.get_feature_by_name('ActionDeviceKey').set(ACTION_DEV_KEY)
        self.cam.get_feature_by_name('ActionGroupKey').set(ACTION_GROUP_KEY)
        self.cam.get_feature_by_name('ActionGroupMask').set(ACTION_GROUP_MASK)

        print("Alvium Producer Cam ID: " + self.cam_id)

    def run(self):
        try:
            with self.cam:
                try:
                    self.setup_camera()
                    # if self.cam_id == MASTER_CAM_ID:
                    #     print(self.cam.__dict__.keys())
                    #     self.print_features()
                except:
                    print('Camera setup failed, restarting...')
                    #self.cam.DeviceReset.run()
                try:
                    self.cam.start_streaming(self)
                    self.killswitch.wait()
                finally:
                    self.cam.stop_streaming()
        except Exception as e:
            print(e)
            pass
        print('Thread \'FrameProducer({})\' terminated.'.format(self.cam_id))

    def print_features(self):
        for feature in self.cam.get_all_features():
            self.print_feature(feature)

    def print_feature(self, feature):
        try:
            value = feature.get()
        except (AttributeError, VimbaFeatureError):
            value = None
        if "Binning" in feature.get_name():
            print('/// Feature name   : {}'.format(feature.get_name()))
            print('/// Display name   : {}'.format(feature.get_display_name()))
            print('/// Tooltip        : {}'.format(feature.get_tooltip()))
            print('/// Description    : {}'.format(feature.get_description()))
            print('/// SFNC Namespace : {}'.format(feature.get_sfnc_namespace()))
            print('/// Unit           : {}'.format(feature.get_unit()))
            print('/// Value          : {}\n'.format(str(value)))


if __name__ == '__main__':
    record_state=None
    multicam_handler_thread = AlviumMultiCam()
    multicam_handler_thread.start()
    while multicam_handler_thread.is_alive():
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
            if topic==zmq_topics.topic_system_state:
                _,system_state=data
            time.sleep(0.001)
    print('done running thread exited')
 
    multicam_handler_thread.join()
