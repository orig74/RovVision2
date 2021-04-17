import numpy as np
import cv2
import time


CHKBD_DIMS = (4, 11)
CHKBD_SCALE = 0.03
CALIB_DIR = "/home/stereo/CalibParams/"

ALPHA = 1


class Calibrator():
    def __init__(self):
        self.imgPnts_l = []
        self.imgPnts_r = []
        self.objPnts = []
        self.imshape = None
        self.ValidCalib = False
        self.LoadCalib()
        self.objp = np.zeros((CHKBD_DIMS[0] * CHKBD_DIMS[1], 3), np.float32)
        idx = 0
        for i in range(CHKBD_DIMS[1]):
            for j in range(CHKBD_DIMS[0]):
                self.objp[idx, :] = ((i * CHKBD_SCALE, (2 * j + i % 2) * CHKBD_SCALE, 0))
                idx += 1
        # plt.scatter(objp[:, 0], objp[:, 1], s=400, color='black')
        # plt.show()

    def LoadCalib(self):
        try:
            self.LeftStereoMapX = np.load(CALIB_DIR + "Left_Stereo_Map_x.npy")
            self.LeftStereoMapY = np.load(CALIB_DIR + "Left_Stereo_Map_y.npy")
            self.RoiL = np.load(CALIB_DIR + "RoiL.npy")
            self.RightStereoMapX = np.load(CALIB_DIR + "Right_Stereo_Map_x.npy")
            self.RightStereoMapY = np.load(CALIB_DIR + "Right_Stereo_Map_y.npy")
            self.RoiR = np.load(CALIB_DIR + "RoiR.npy")
            self.ValidCalib = True
        except:
            print("Unable to load Calib params!")
            self.ValidCalib = False

    def ResetCalibration(self):
        self.imgPnts_l = []
        self.imgPnts_r = []
        self.objPnts = []

    def AddImgPnts(self, l_img, r_img, drawCHKBD=False):
        gray_l = cv2.cvtColor(l_img, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(r_img, cv2.COLOR_BGR2GRAY)
        self.imshape = gray_r.shape[:2]
        ret_l, centers_l = cv2.findCirclesGrid(gray_l, CHKBD_DIMS, flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
        ret_r, centers_r = cv2.findCirclesGrid(gray_r, CHKBD_DIMS, flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
        if ret_l and ret_r:
            self.imgPnts_l.append(centers_l)
            self.imgPnts_r.append(centers_r)
            self.objPnts.append(self.objp)
            if drawCHKBD:
                cv2.drawChessboardCorners(l_img, CHKBD_DIMS, centers_l, ret_l)
                cv2.drawChessboardCorners(r_img, CHKBD_DIMS, centers_r, ret_r)
        print("Num Calib Imgs: {}".format(len(self.objPnts)))
        return l_img, r_img

    def CalcMonoError(self, img_pnts, obj_pnts, rvecs, tvecs, mtx, dist):
        mean_error = 0
        for i in range(len(obj_pnts)):
            imgpoints2, _ = cv2.projectPoints(obj_pnts[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(img_pnts[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        return mean_error / len(obj_pnts)

    def CalibMono(self, img_points, obj_points):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, self.imshape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, self.imshape[::-1], ALPHA, self.imshape[::-1])
        print(roi)
        error = self.CalcMonoError(img_points, obj_points, rvecs, tvecs, mtx, dist)
        print("Total Error: {}".format(error))
        return mtx, dist, newcameramtx, roi, error

    def RunStereoCalibration(self, calIdxStep=1):
        print("Calibrating on {} imgs...".format(len(self.imgPnts_l[::calIdxStep])))
        print("Left:")
        mtx_l, dist_l, newcameramtx_l, roi_l, err_l = self.CalibMono(self.imgPnts_l[::calIdxStep], self.objPnts[::calIdxStep])
        print("Right:")
        mtx_r, dist_r, newcameramtx_r, roi_r, err_r = self.CalibMono(self.imgPnts_r[::calIdxStep], self.objPnts[::calIdxStep])

        # cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_TILTED_MODEL + 
        flags = cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 + cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6
        criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(self.objPnts[::calIdxStep],
                                                                                            self.imgPnts_l[::calIdxStep],
                                                                                            self.imgPnts_r[::calIdxStep],
                                                                                            newcameramtx_l, dist_l, newcameramtx_r, dist_r,
                                                                                            self.imshape[::-1], flags=flags, criteria=criteria_stereo)
        print("Stereo Relative Rotation: {}".format(Rot))
        print("Stereo Relative Translation: {}".format(Trns))
        rectify_scale = 1
        rect_l, rect_r, proj_mat_l, proj_mat_r, Q, self.RoiL, self.RoiR = cv2.stereoRectify(new_mtxL, distL, new_mtxR, distR, self.imshape[::-1], Rot, Trns, rectify_scale, (0, 0), alpha=ALPHA)
        self.LeftStereoMapX, self.LeftStereoMapY = cv2.initUndistortRectifyMap(new_mtxL, distL, rect_l, proj_mat_l, self.imshape[::-1], cv2.CV_16SC2)
        self.RightStereoMapX, self.RightStereoMapY = cv2.initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r, self.imshape[::-1], cv2.CV_16SC2)

        if self.RoiL[2] == 0 or self.RoiR[2] == 0 or self.RoiL[3] == 0 or self.RoiR[3] == 0:
            print("Calib Failed!")
            print(self.RoiL)
            print(self.RoiR)
            self.LoadCalib()
        else:
            print("Saving parameters ......")
            with open(CALIB_DIR + "calib_output.txt", 'w') as out_file:
                out_file.write("Error Left: {}\n".format(err_l))
                out_file.write("Stereo ROI Left: {}\n".format(self.RoiL))
                out_file.write("Error Right: {}\n".format(err_r))
                out_file.write("Stereo ROI Right: {}\n".format(self.RoiR))
            out_file.close()
            np.save(CALIB_DIR + "Stereo_Trans", Trns)
            np.save(CALIB_DIR + "Stereo_Rot", Rot)
            np.save(CALIB_DIR + "Q", Q)
            np.save(CALIB_DIR + "ProjMatLeft", proj_mat_l)
            np.save(CALIB_DIR + "ProjMatRight", proj_mat_r)
            np.save(CALIB_DIR + "Left_Cam_Matrix", new_mtxL)
            np.save(CALIB_DIR + "Left_Stereo_Map_x", self.LeftStereoMapX)
            np.save(CALIB_DIR + "Left_Stereo_Map_y", self.LeftStereoMapY)
            np.save(CALIB_DIR + "RoiL", self.RoiL)
            np.save(CALIB_DIR + "Right_Cam_Matrix", new_mtxR)
            np.save(CALIB_DIR + "Right_Stereo_Map_x", self.RightStereoMapX)
            np.save(CALIB_DIR + "Right_Stereo_Map_y", self.RightStereoMapY)
            np.save(CALIB_DIR + "RoiR", self.RoiR)
            self.ValidCalib = True

    def StereoRectify(self, l_img, r_img):
        l_img_sr = cv2.remap(l_img, self.LeftStereoMapX, self.LeftStereoMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 0)
        r_img_sr = cv2.remap(r_img, self.RightStereoMapX, self.RightStereoMapY, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, 0)
        return l_img_sr, r_img_sr
