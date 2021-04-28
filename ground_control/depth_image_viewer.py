import zmq
import select
import struct
import cv2,os
import numpy as np
from gst import init_gst_reader,get_imgs,set_files_fds,get_files_fds,save_main_camera_stream
import zmq_wrapper as utils
import image_enc_dec
 
DEPTH_THESH = 5
CALIB_DIR = "/home/stereo/Desktop/CalibParamsAir/"

leftIntrinsics = np.load(CALIB_DIR + "Left_Cam_Matrix.npy")
rightIntrinsics = np.load(CALIB_DIR + "Right_Cam_Matrix.npy")
stereoRot = np.load(CALIB_DIR + "Stereo_Rot.npy")
stereoTrns = np.load(CALIB_DIR + "Stereo_Trans.npy")
STEREO_FOCAL_LENGTH = leftIntrinsics[0, 0]
FOCAL_DIFF_THRESH = 5
assert (abs(leftIntrinsics[0, 0] - leftIntrinsics[1, 1]) < FOCAL_DIFF_THRESH and
        abs(rightIntrinsics[0, 0] - rightIntrinsics[1, 1]) < FOCAL_DIFF_THRESH and
        abs(leftIntrinsics[0, 0] - rightIntrinsics[1, 1]) < FOCAL_DIFF_THRESH)
print(STEREO_FOCAL_LENGTH)
STEREO_BASELINE = np.linalg.norm(stereoTrns)
print(STEREO_BASELINE)

num_disparity = 128
left_matcher = cv2.StereoBM_create(numDisparities=num_disparity, blockSize=21)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

sigma = 1
lmbda = 100.0
wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)

if __name__=='__main__':
    init_gst_reader(2)
    sx,sy=config.cam_res_rgbx,config.cam_res_rgby

    while 1:
        images=get_imgs()
        if images[0] is not None and images[1] is not None:
            fmt_cnt_l=image_enc_dec.decode(images[0])
            fmt_cnt_r=image_enc_dec.decode(images[1])

            gray_l = cv2.cvtColor(images[0].copy(), cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(images[1].copy(), cv2.COLOR_BGR2GRAY)
            left_disp = left_matcher.compute(gray_l, gray_r)
            right_disp = right_matcher.compute(gray_r, gray_l)
            filtered_disp = wls_filter.filter(left_disp, gray_l, disparity_map_right=right_disp).astype(np.float32)

            depth_img = STEREO_BASELINE * STEREO_FOCAL_LENGTH / ((filtered_disp + 16.00001) / 16)

            cv2.imshow("calibrated ROI", np.hstack([gray_l, gray_r]))
            raw_disp = left_disp.astype(np.float32)
            raw_disp += np.min(raw_disp)
            raw_disp /= np.max(raw_disp)
            cv2.imshow("Raw Disparity", raw_disp)
            depth_img = (depth_img < DEPTH_THESH) * depth_img
            cv2.imshow("Depth", depth_img / DEPTH_THESH)
            images=[None,None]

        k=cv2.waitKey(10)
        if k==ord('q'):
            break

cv2.destroyAllWindows()
