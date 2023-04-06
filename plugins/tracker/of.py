import cv2
import numpy as np

class OF(object):
    def __init__(self):
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                          maxLevel = 2,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.old_gray=None    
# Take first frame and find corners in it
    def set(self,gray_img,pt):
        self.old_gray=gray_img
        self.p0=np.array(pt[:2],dtype='float32').reshape((-1,1,2))

    def track(self,frame_gray):
        if self.old_gray is None:
            return None
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
        good_new = p1[st==1]
        if len(good_new)>0:
            x,y=good_new[0].ravel()
            self.p0 = good_new.reshape(-1,1,2)
            self.old_gray=frame_gray
            return (x,y)
        else:
            self.old_gray=None

       

if __name__=='__main__':
    clk_x,clk_y=0,0
    of = OF()
    frame_gray=None
    def click_event(event, x, y, flags, param):
        global clk_x,clk_y
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, ', ', y)
            clk_x,clk_y=x,y
            if frame_gray is not None:
                of.set(frame_gray,(x,y))

    cap = cv2.VideoCapture(0)
    cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
    cv2.setMouseCallback('frame', click_event)
    while True:
        ret,frame = cap.read()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret=of.track(frame_gray)
        if ret is not None:
            frame = cv2.circle(frame,ret,5,(255,0,0),-1)

        cv2.imshow('frame',frame)
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        # calculate optical flow
    cv2.destroyAllWindows()
    cap.release()
