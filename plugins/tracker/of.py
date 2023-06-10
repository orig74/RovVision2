import cv2
import numpy as np

class OF(object):
    def __init__(self):
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                          maxLevel = 2,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.old_gray=None    
        self.last_good=None
        self.bad_count=0
# Take first frame and find corners in it
    def set(self,gray_img,pt):
        self.old_gray=gray_img
        self.p0=np.array(pt[:2],dtype='float32').reshape((-1,1,2))
        self.last_good=self.p0

    def reset(self):
        self.old_gray=None 
        self.bad_count=0


    def track(self,frame_gray):
        if self.bad_count>1:
            self.reset()
        if self.old_gray is None:
            return None
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
        p0r, st, err = cv2.calcOpticalFlowPyrLK(frame_gray,self.old_gray,p1,None,**self.lk_params)
        d = abs(self.p0-p0r).reshape(-1, 2).max(-1)

        good_new = p1[st==1]
        if d<3 and len(good_new)>0:
            x,y=good_new[0].ravel()
            #if self.last_good is not None:
            #    xp,yp=self.last_good[0].ravel()
            #    tr=50
            #    if abs(x-xp)>tr or abs(y-yp)>tr:
            #        good_new=self.last_good
            #        self.bad_count+=1
            #        print('track fail...1')
            self.last_good=good_new
            self.p0 = good_new.reshape(-1,1,2)
            self.old_gray=frame_gray
            return (x,y)
        else:
            print('track fail...',self.bad_count,d)
            self.old_gray=frame_gray
            self.p0=self.last_good.reshape(-1,1,2)
            self.bad_count+=1
            if self.last_good is not None:
                return self.last_good[0].ravel()
        #else:
        #    self.old_gray=None

class OF2(object):
    def __init__(self):
        # Parameters for lucas kanade optical flow
        self.lk_params = dict( winSize  = (15,15),
                          maxLevel = 2,
                          criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.old_gray=None    
        self.last_good=None
        self.bad_count=0
# Take first frame and find corners in it
    def set(self,gray_img,pt):
        self.old_gray=gray_img
        mgx=10
        mgy=10
        temp=gray_img[pt[1]-mgx:pt[1]+mgx,pt[0]-mgy:pt[0]+mgy]
        self.p0=cv2.goodFeaturesToTrack(temp,1, qualityLevel=0.01,minDistance=3)
        #cv2.imshow('ggg',temp)
        #cv2.waitKey(0)
        rel_pt=self.p0[0][0]
        new_pt=(pt[0]-mgy+rel_pt[0],pt[1]-mgx+rel_pt[1])
        self.p0=np.array(new_pt[:2],dtype='float32').reshape((-1,1,2))
        #print('=====',pt,new_pt,self.p0)
        #self.p0=np.array(new_pt,dtype='float32').reshape((-1,1,2))
        self.last_good=self.p0

    def reset(self):
        self.old_gray=None 
        self.bad_count=0


    def track(self,frame_gray):
        if self.bad_count>1:
            self.reset()
        if self.old_gray is None:
            return None
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
        p0r, st, err = cv2.calcOpticalFlowPyrLK(frame_gray,self.old_gray,p1,None,**self.lk_params)
        d = abs(self.p0-p0r).reshape(-1, 2).max(-1)

        good_new = p1[st==1]
        if d<3 and len(good_new)>0:
            x,y=good_new[0].ravel()
            #if self.last_good is not None:
            #    xp,yp=self.last_good[0].ravel()
            #    tr=50
            #    if abs(x-xp)>tr or abs(y-yp)>tr:
            #        good_new=self.last_good
            #        self.bad_count+=1
            #        print('track fail...1')
            self.last_good=good_new
            self.p0 = good_new.reshape(-1,1,2)
            self.old_gray=frame_gray
            return (x,y)
        else:
            print('track fail...',self.bad_count,d)
            self.old_gray=frame_gray
            self.p0=self.last_good.reshape(-1,1,2)
            self.bad_count+=1
            if self.last_good is not None:
                return self.last_good[0].ravel()
       

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
