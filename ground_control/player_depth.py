# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#
# to copy data.pkl
# sim:
# cd into data dir and then
# > scp -P 2222 oga13@localhost:projects/bluerov/data/$(basename $( pwd ))/data.pkl .

import sys,os,time,traceback
from datetime import datetime
sys.path.append('../')
sys.path.append('../utils')
import zmq
import pickle
import select
import struct
import cv2,os
import copy
import argparse
import numpy as np
import pandas as pd
import config
import gst2
import zmq_wrapper
import zmq_topics
import glob

font = cv2.FONT_HERSHEY_SIMPLEX

def torgb(im):
    imret = cv2.applyColorMap(cv2.convertScaleAbs(im, alpha=0.46), cv2.COLORMAP_JET)[:,:,::-1]
    imret[im==0]=(0,0,0)
    return imret

parser = argparse.ArgumentParser()
parser.add_argument("-s","--start_frame",help="start frame",default=0,type=int)
parser.add_argument("path",help="dir path")
args = parser.parse_args()

cv2.namedWindow('depth',cv2.WINDOW_NORMAL)
cv2.namedWindow('rgb',cv2.WINDOW_NORMAL)
df=pd.read_csv(args.path+'/d405_frame_data.txt',delimiter=',',header=None)

def read_pkl(pkl_file):
    fd = open(pkl_file,'rb')
    ret=[]
    while 1:
        try:
            d=pickle.load(fd)
            if d[0] in [zmq_topics.topic_main_cam_ts , zmq_topics.topic_stereo_camera_ts]:
                ret.append(d)
        except EOFError:
            break
    return ret

try:
    pkl_data = read_pkl(args.path+'/viewer_data.pkl')
except FileNotFoundError:
    pkl_data = None
    print('no pickle data file')

#import ipdb;ipdb.set_trace()
depth_scale=df[2][0]
scale_to_mm=depth_scale*1000
#fnums=[int(i) for i in df[0] if not np.isnan(i)]
fnums=[int(l) for l in os.popen(f'cd {args.path} && ls -1 *.jpg |cut -d"." -f1').readlines() if l.strip()]
fnums.sort()
i=args.start_frame

def rope_detect_depth(depth_img,start_row=150,nrows=100):
    #np.save('/tmp/imgd.npy',depth_img)
    marg=100
    imt=depth_img[start_row:start_row+nrows,:].sum(axis=0).flatten()/nrows
    #prioritizing center
    r=600
    imtp=imt+np.abs(np.linspace(-r,r,len(imt)))
    #blur line
    imtp=np.convolve(imtp,np.ones(20)/20,mode='same')
    col=np.argmin(imtp[marg:-marg])+marg
    return imt[col],col,imtp


refPt=None
last_click_data=None
delta_click=None
def click(event, x, y, flags, param):
    global delta_click,last_click_data
    if event == cv2.EVENT_LBUTTONDOWN:
        d=depth_img[y,x]*config.water_scale
        print('---click',x,y,depth_img[y,x])
        click_data=(np.linalg.inv(np.array(config.cam_main_int)) @ np.array([[x*d,y*d,d]]).T).flatten()
        xw,yw,s=click_data
        if last_click_data is not None:
            delta_click=click_data-last_click_data
        last_click_data=click_data
        print('===+===',xw,yw,s,delta_click)
    if event == cv2.EVENT_MBUTTONDOWN:
        k=cv2.pollKey()
        print('===',k)

class ClickWin(object):
    def __init__(self,wname):
        self.wname=wname
        self.draw_state=False
    def click(self,event, x, y, flags, param):
        if self.draw_state:
            print('====',self.wname,x,y,event)
        if event == cv2.EVENT_MBUTTONDOWN:
            self.draw_state=True
        if event == cv2.EVENT_MBUTTONUP:
            self.draw_state=False

cv2.setMouseCallback("depth", click)
cv2.setMouseCallback("rgb", click)
shape=[100,100]
wins={}
cobjs=[]
while 1:
    if pkl_data is not None:
        i=np.clip(i,0,len(pkl_data)-1) 
        if pkl_data[i][0]==zmq_topics.topic_main_cam_ts:
            fnum=pkl_data[i][2][0]
            is_depth_image=True
        else:
            #print('main image')
            fmt=args.path+f'/{pkl_data[i][2][0]:06d}'+'*.pgm'
            #print('====',fmt)
            img_files=glob.glob(args.path+f'/{pkl_data[i][2][0]:06d}'+'*.pgm')
            #print(img_files)
            for img_fl in img_files:
                wname=img_fl.strip().split('_DEV_')[1].split('.pgm')[0]
                if wname not in wins:
                    cv2.namedWindow(wname,cv2.WINDOW_NORMAL)
                    cw=ClickWin(wname)
                    cv2.setMouseCallback(wname,cw.click)
                    cobjs.append(cw)
                    #wins[wname]=None
                pgm=cv2.imread(img_fl,cv2.IMREAD_GRAYSCALE)
                wins[wname]={'img':cv2.cvtColor(pgm,cv2.COLOR_BAYER_BG2BGR)}
                #cv2.imshow(wname,cv2.cvtColor(pgm,cv2.COLOR_BAYER_BG2BGR))

            #cv2.imread()
            is_depth_image=False

    else:
        i=np.clip(i,0,len(fnums)-1)
        fnum=fnums[i]
        is_depth_image=True

    if is_depth_image:
        rgb_img=cv2.imread(args.path+f'/{fnum:06d}.jpg')
        print('===',args.path+f'/{fnum:06d}.jpg')
        posx=0
        if rgb_img is not None:
            shape=rgb_img.shape[:2]
            depth_img=np.frombuffer(open(args.path+f'/d{fnum:06d}.bin','rb').read(),'uint16').astype('float').reshape(shape)*scale_to_mm*config.water_scale
            depth_img[depth_img<1]=10000 #10 meters

            ret=rope_detect_depth(depth_img)
            posx=ret[1]
            #cv2.imshow('depth',torgb(depth_img))
            wins['depth']={'img':torgb(depth_img)}
            print('maxmin',depth_img.max(),depth_img.min(),ret[0],ret[1])
        else:
            #rgb_img=np.zeros(shape,'uint8')
            i+=1
            continue
        cv2.line(rgb_img,(posx,30),(posx,40),(255,255,255),thickness=4)
        line=''
        if last_click_data is not None:
            x,y,z=last_click_data
            line=f'xyr {x:.1f},{y:.1f},{z:.1f}' 
            cv2.putText(rgb_img,line,(10,400), font, 0.7,(255,0,0),2,cv2.LINE_AA)
            if delta_click is not None:
                x,y,z=delta_click
                line=f'delta xyr {x:.1f},{y:.1f},{z:.1f}' 
                cv2.putText(rgb_img,line,(10,420), font, 0.7,(255,0,0),2,cv2.LINE_AA)
            
        wins['rgb']={'img':rgb_img}
        #cv2.imshow('rgb',rgb_img)

    for wname in wins:
        if wins[wname]:
            cv2.imshow(wname,wins[wname]['img'])
    k=cv2.waitKey(100)%0xff
    #k=cv2.pollKey()%0xff
    if k in [ord(','),81,52]:
        i=i-1
    elif k in [ord('<')]:
        i=i-10
    elif k in [ord('.'),54,83]:
        i=i+1
    elif k in [ord('>')]:
        i=i+10
    elif k in [ord('q')]:
        break
    else:
        time.sleep(0.5)
        print('i',i)
cv2.destroyAllWindows()

