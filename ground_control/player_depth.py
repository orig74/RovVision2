# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#
# to copy data.pkl
# sim:
# cd into data dir and then
# > scp -P 2222 oga13@localhost:projects/bluerov/data/$(basename $( pwd ))/data.pkl .
#test data for rope end:
#python player_depth.py -s 0 ../../data2/230719-103213 # false at frame ~11447
#python player_depth.py -s 0 ../../data2/230719-104817 # end of the rope ~2673
#230719-105708 #false 1533 #end 1777 #close to false 4512
#230720-093250 #false 8444 maybe tether
#230720-100957 #false 12719 ~300mm
#230720-104714 #false 7536
#230720-110859 #rope end 8191 before climb
#230720-114948 #false end ~ 8986
#230720-121540 # false 13841 24211
#230720-134041 # end rope event 678 #false 11889 12491 #15230 end not detected!

import sys,os,time,traceback
from datetime import datetime
sys.path.append('../')
sys.path.append('../utils')
sys.path.append('../plugins')
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

from tracker.of import OF
tcv = None 
from tracker.rope_detect import rope_detect_depth

font = cv2.FONT_HERSHEY_SIMPLEX

def torgb(im):
    #import ipdb;ipdb.set_trace()
    im_=im.copy()
    clip=5000
    im_[im_>clip]=clip
    med=im_.flatten()
    med=med.take(np.nonzero(np.bitwise_and(med>0 , med<clip))[0])#.mean()
    if 0:
        from matplotlib import pyplot as plt
        plt.hist(med,bins=500)
        plt.show()
        sys.exit()
    im_[im==0]=med.mean()
    imret = cv2.applyColorMap(cv2.convertScaleAbs(im_/10, alpha=0.46), cv2.COLORMAP_JET)[:,:,::-1]
    #imret = cv2.applyColorMap(cv2.convertScaleAbs(np.log(im_)*100, alpha=0.46), cv2.COLORMAP_JET)[:,:,::-1]
    imret[im==0]=(0,0,0)
    return imret

parser = argparse.ArgumentParser()
parser.add_argument("-s","--start_frame",help="start frame",default=0,type=int)
parser.add_argument("-v","--vision_cameras",help="show vision cameras",default=False,action='store_true')
parser.add_argument("-d","--detector_view",help="show detector",default=False,action='store_true')
parser.add_argument("-t","--tracker_mode",help="tracker or mark mode (default)",default=False,action='store_true')
parser.add_argument("path",help="dir path")
args = parser.parse_args()

cv2.namedWindow('depth',cv2.WINDOW_NORMAL)
cv2.namedWindow('rgb',cv2.WINDOW_NORMAL)

if args.detector_view:
    cv2.namedWindow('detect',cv2.WINDOW_NORMAL)
    from tracker.mussel_detector import detect

def read_pkl(pkl_file):
    fd = open(pkl_file,'rb')
    ret=[]
    telem_data=[]
    telem={}
    while 1:
        try:
            d=pickle.load(fd)
            if d[0]==zmq_topics.topic_depth:
                telem['depth']=d[2]['depth']
            if d[0] in [
                    zmq_topics.topic_main_cam_ts , 
                    zmq_topics.topic_stereo_camera_ts,
                    b'printer_sink']:
                ret.append(d)
                telem_data.append(telem.copy())
        except EOFError:
            break
    return ret,telem_data

try:
    pkl_data,telem_data = read_pkl(args.path+'/viewer_data.pkl')
except FileNotFoundError:
    pkl_data = None
    telem_data = None
    print('no pickle data file')

print('done reading pickle')
#import ipdb;ipdb.set_trace()
try:
    df=pd.read_csv(args.path+'/d405_frame_data.txt',delimiter=',',header=None)
    depth_scale=df[2][0]
except:
    print('failed loading /d405_frame_data.txt')
    depth_scale=1e-4
scale_to_mm=depth_scale*1000
#fnums=[int(i) for i in df[0] if not np.isnan(i)]
fnums=[int(l) for l in os.popen(f'cd {args.path} && ls -1 *.jpg |cut -d"." -f1').readlines() if l.strip()]
fnums.sort()
if pkl_data is None:
    i=args.start_frame
else:
    fnum=-1
    i=0
    while fnum<args.start_frame:
        if pkl_data[i][0]==zmq_topics.topic_main_cam_ts:
            fnum=pkl_data[i][2][0]
        if i>=len(pkl_data):
            print('error cant find frame (or close to) frame',args.start_frame)
            sys.exit(0)
        i+=1


print('done reading fnames')
#def rope_detect_depth(depth_img,start_row=150,nrows=100):
#    #np.save('/tmp/imgd.npy',depth_img)
#    marg=100
#    imt=depth_img[start_row:start_row+nrows,:].sum(axis=0).flatten()/nrows
#    #prioritizing center
#    r=600
#    imtp=imt+np.abs(np.linspace(-r,r,len(imt)))
#    #blur line
#    imtp=np.convolve(imtp,np.ones(20)/20,mode='same')
#    col=np.argmin(imtp[marg:-marg])+marg
#    return imt[col],col,imtp


refPt=None
last_click_data=None
delta_click=None
wins={}
of=OF()
def click(event, x, y, flags, param):
    global delta_click,last_click_data,tcv
    if event == cv2.EVENT_LBUTTONDOWN:
        d=depth_img[y,x]*config.water_scale*scale_to_mm
        print('---click',x,y,depth_img[y,x])
        click_data=(np.linalg.inv(np.array(config.cam_main_int)) @ np.array([[x*d,y*d,d]]).T).flatten()
        xw,yw,s=click_data
        if last_click_data is not None:
            delta_click=click_data-last_click_data
        last_click_data=click_data
        wins['rgb']['redraw']=True
        if rgb_img is not None:
            of.set(gray_img,(x,y))
            mg=5
            bbox=(x-mg,y-mg,x+mg,y+mg)
            #tcv=cv2.TrackerGOTURN_create()
            #tcv.init(gray_img,bbox) 
            print('setting track',x,y)
        #print('===+===',xw,yw,s,delta_click)
    #if event == cv2.EVENT_MBUTTONDOWN:
        #k=cv2.pollKey()
        #print('===',k)
mark_mode = True
mark_sz=0
show_text=True

class ClickWin(object):
    def __init__(self,wname):
        self.wname=wname
        self.draw_state=False
        #print('=====xxx===',wname)

    def click(self,event, x, y, flags, param):
        if self.draw_state:
            if 'mask' in wins[self.wname]:
                mim=wins[self.wname]['mask']
                #mg=1 if self.wname=='rgb' else 3
                mg=mark_sz
                #mim[y-mg:y+mg,x-mg:x+mg]=0 if mark_mode else 1
                cv2.circle(mim,(x,y),mg,0 if mark_mode else 1 ,mg)

                wins[self.wname]['redraw']=True
                #print('====',self.wname,x,y,event,self.draw_state)

        if event == cv2.EVENT_MBUTTONDOWN:
            self.draw_state=True
            #print('==ds==down==',self.wname,x,y,event)
        if event == cv2.EVENT_MBUTTONUP:
            #print('==ds==up==',self.wname,x,y,event)
            self.draw_state=False

cv2.setMouseCallback("depth", click)
shape=[100,100]
cobjs=[]

if not args.tracker_mode:
    cw=ClickWin('rgb')
    cv2.setMouseCallback('rgb',cw.click)
    cobjs.append(cw)
else:
    cv2.setMouseCallback("rgb", click)

def get_mask_stats(mask):
    output = cv2.connectedComponentsWithStats(1-mask,ltype = cv2.CV_16U)
    (numLabels, labels, stats, centroids) = output
    n_componnets=len([s for s in stats if 30<s[-1]<100000])
    return (numLabels,stats, centroids,n_componnets)
    
is_depth_image=False
prev_i=-1
while 1:
    if pkl_data is not None:
        i=np.clip(i,0,len(pkl_data)-2) 
        if pkl_data[i][0].startswith(b'printer'):
            print('printer:',pkl_data[i][2]['txt'])
        if pkl_data[i][0]==zmq_topics.topic_main_cam_ts:
            fnum=pkl_data[i][2][0]
            is_depth_image=True
        elif args.vision_cameras:
            main_ind=pkl_data[i][2][0]
            fmt=args.path+f'/{main_ind:06d}'+'*.pgm'
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
                    wins[wname]={'img':None,'file_path':None,'redraw':True}

                if wins[wname]['file_path']!=img_fl:
                    print('main image',main_ind)
                    pgm=cv2.imread(img_fl,cv2.IMREAD_GRAYSCALE)
                    if pgm is not None:
                        mask_fname=img_fl[:-3]+'mask.npy'
                        if os.path.isfile(mask_fname):
                            mask=np.load(mask_fname)
                        else:
                            mask=np.ones_like(pgm)
                        wins[wname]={
                                'img':cv2.cvtColor(pgm,cv2.COLOR_BAYER_BG2BGR),
                                'file_path':img_fl,
                                'mask':mask,
                                'redraw':True}
                    #cv2.imshow(wname,cv2.cvtColor(pgm,cv2.COLOR_BAYER_BG2BGR))

            #cv2.imread()
            is_depth_image=False

    else:
        i=np.clip(i,0,len(fnums)-1)
        fnum=fnums[i]
        is_depth_image=True

    if is_depth_image and (prev_i!=i or args.tracker_mode):
        fname = args.path+f'/{fnum:06d}.jpg'
        if not ('rgb' in wins and wins['rgb'].get('file_path')==fname) or wins['rgb'].get('redraw'):
            rgb_img=None
            if os.path.isfile(fname):
                try:
                    rgb_img=cv2.imread(args.path+f'/{fnum:06d}.jpg')
                    gray_img=cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
                    if args.detector_view:
                        detected_img=detect(rgb_img,annotate=True)['img']
                    _ret=of.track(gray_img)
                    #if tcv is not None:
                    #    print('hhh',tcv.update(gray_img))
                    if _ret is not None:
                        x,y=map(int,_ret)
                        cv2.circle(rgb_img, (x,y), 3, (0, 0, 255), -1)
                except Exception as E:
                    print('Error reading image',E)
                    rgb_img=None
            #print('===',args.path+f'/{fnum:06d}.jpg')
            posx=0
            if rgb_img is not None:
                shape=rgb_img.shape[:2]
                depth_fl = args.path+f'/d{fnum:06d}.bin'
                depth_img=np.frombuffer(open(depth_fl,'rb').read(),'uint16').astype('float').reshape(shape)#*scale_to_mm*config.water_scale
                #depth_img[depth_img<1]=10000 #10 meters

                ret=rope_detect_depth(depth_img,scale_to_mm,config.water_scale)
                d,posx,up_validation,down_validation,_=ret
                #cv2.imshow('depth',torgb(depth_img))
                wins['depth']={'img':torgb(depth_img),'redraw':True}
                diff=down_validation-up_validation
                line=f'range {d:.1f},U{up_validation:.1f},D{down_validation:.1f}'
                if show_text:
                    cv2.putText(rgb_img,line,(10,40), font, 0.7,(255,0,0),2,cv2.LINE_AA)
                    cv2.putText(rgb_img,f'Df{diff:.1f}',(10,60),font, 0.7,(255,0,0) if diff<150 else (0,0,255),
                            2,cv2.LINE_AA)
                    cv2.putText(rgb_img,f'{fnum:06d}',(10,90), font, 0.7,(255,0,0),2,cv2.LINE_AA)

                mrg=50
                vrow=50
                nrow=100
                srow=150
                for rrr in [srow-vrow,srow,srow+nrow,srow+nrow+vrow]:
                    cv2.line(rgb_img,(posx-mrg,rrr),(posx+mrg,rrr),(255,255,255),thickness=1)
                mask_fname=fname[:-3]+'mask.npy'
                #print('mask file...',mask_fname)
                if os.path.isfile(mask_fname):
                    mask=np.load(mask_fname)
                else:
                    mask=np.ones(rgb_img.shape[:2],dtype='uint8')
            else:
                #rgb_img=np.zeros(shape,'uint8')
                i+=1
                continue
            cv2.line(rgb_img,(posx,30),(posx,40),(255,255,255),thickness=2)
            line=''
            if last_click_data is not None:
                x,y,z=last_click_data
                line=f'xyr {x:.1f},{y:.1f},{z:.1f}' 
                cv2.putText(rgb_img,line,(10,400), font, 0.7,(255,0,0),2,cv2.LINE_AA)
                if delta_click is not None:
                    x,y,z=delta_click
                    line=f'delta xyr {x:.1f},{y:.1f},{z:.1f}' 
                    cv2.putText(rgb_img,line,(10,420), font, 0.7,(255,0,0),2,cv2.LINE_AA)
            if telem_data is not None:
                td=telem_data[i]
                depth=td.get('depth',-1)
                if show_text:
                    cv2.putText(rgb_img,f'Depth{depth:.1f}',(10,20), font, 0.7,(255,0,0),2,cv2.LINE_AA)
            
            wins['rgb']={'img':rgb_img,'redraw':True,'file_path':fname, 'mask':mask}

            if args.detector_view:
                wins['detect']={'img':detected_img, 'redraw':True}
            #cv2.imshow('rgb',rgb_img)

    for wname in wins:
        if wins[wname] and wins[wname].get('redraw'):
            img=wins[wname]['img']
            if img is not None:
                if 'mask' in wins[wname]:
                    #print('===',wname)
                    img=img.copy()
                    #chs=[1] if wname=='rgb' else [1]
                    ch=1
                    if wname=='rgb':
                        img[:,:,ch]=img[:,:,ch]*wins[wname]['mask']
                    else:
                        #img[:,:,1]=img[:,:,1]*wins[wname]['mask']
                        img[wins[wname]['mask']==0,2]=255
                        #img[:,:,2]=img[:,:,0]*(1-wins[wname]['mask'])


                cv2.imshow(wname,img)
                    #cv2.imshow(wname,wins[wname]['mask']*255)
                wins[wname]['redraw']=False
            #print('current index',i)

    k=cv2.waitKey(100)%0xff
    #k=cv2.pollKey()%0xff
    prev_i=i
    if k in [ord(','),81,52]:
        i=i-6
    elif k in [ord('<')]:
        print('===',i)
        i=i-20
    elif k in [ord('.'),54,83]:
        i=i+1
    elif k in [ord('>')]:
        i=i+20
    elif k in [ord('q')]:
        break
    elif k in [ord('m')]:
        mark_mode=not mark_mode
    elif k in [ord(x) for x in '1234']:
        mark_sz=k-ord('1')
        print('setting mark size to ',mark_sz)
    elif k in [ord('c')]:
        for wname in wins:
            w=wins[wname]
            if 'mask' in w and w['mask'].min()==0:

                print('wname=',wname)
                #print('number of objects is :',centroids)
                print('number of objects=',get_mask_stats(w['mask'])[3])
                    
    elif k in [ord('n')]:
        show_text = not show_text
                #cv2.imshow('kkkk',labels)
    if k!=254 and not args.tracker_mode:
        for wname in wins:
            w=wins[wname]
            if 'mask' in w:
                mask_fname=w['file_path'][:-3]+'mask.npy'
                n_objs=get_mask_stats(w['mask'])[3]
                if n_objs>0: #means active mask
                    print('saving mask',mask_fname)
                    np.save(mask_fname,w['mask'])
                else:
                    if os.path.isfile(mask_fname):
                        print('deleting mask file',mask_fname)
                        os.remove(mask_fname)




cv2.destroyAllWindows()

