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

#import ipdb;ipdb.set_trace()
depth_scale=df[2][0]
scale_to_mm=depth_scale*1000
#fnums=[int(i) for i in df[0] if not np.isnan(i)]
fnums=[int(l) for l in os.popen(f'cd {args.path} && ls -1 *.jpg |cut -d"." -f1').readlines() if l.strip()]
fnums.sort()
i=args.start_frame

refPt=None
def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('---click',x,y,depth_img[y,x])

cv2.setMouseCallback("depth", click)
cv2.setMouseCallback("rgb", click)

while 1:
    i=np.clip(i,0,len(fnums)-1)
    fnum=fnums[i]
    rgb_img=cv2.imread(args.path+f'/{fnum:06d}.jpg')
    if rgb_img is not None:
        shape=rgb_img.shape[:2]
        depth_img=np.frombuffer(open(args.path+f'/d{fnum:06d}.bin','rb').read(),'uint16').astype('float').reshape(shape)*scale_to_mm
        print('maxmin',depth_img.max(),depth_img.min())
    else:
        rgb_img=np.zeros(shape,'uint8')
    cv2.imshow('rgb',rgb_img)
    cv2.imshow('depth',torgb(depth_img))
    k=cv2.waitKey(0)%0xff
    if k in [ord(','),81,52]:
        i=i-1
    if k in [ord('<')]:
        i=i-10
    if k in [ord('.'),54,83]:
        i=i+1
    if k in [ord('>')]:
        i=i+10
    if k in [ord('q')]:
        break
    print('i',i)
cv2.destroyAllWindows()

