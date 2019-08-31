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
import config
from gst import gst_file_reader
from annotations import draw,draw_seperate
import zmq_wrapper as utils
import explore
import zmq_topics as topics


parser = argparse.ArgumentParser()
parser.add_argument("--nowait",help="run all not wait for keyboard untill framenum or 0 till the end", default=0, type=int)
parser.add_argument("--nosingle",help="dont use the single files only the stream", action='store_true')
parser.add_argument("--nosync", help="dont sync videos", action='store_true')
parser.add_argument("--pub_data", help="publish data", action='store_true')
parser.add_argument("--runtracker", help="run tracker on recorded vid", action='store_true')
parser.add_argument("--path",help="dir path")
parser.add_argument("--bs",help="history buff size",type=int ,default=1000)
parser.add_argument("--cc",help="color orrection matrix file",default=None)
parser.add_argument("--ccr",help="the efect of the color correction 0 to 1.0 (max 1.0)",type=float,default=1.0)
args = parser.parse_args()

file_path_fmt=args.path+'/{}{:08d}'#.ppm'

if args.cc is not None:
    c_mat = pickle.load(open(args.cc,'rb'))*args.ccr+(1.0-args.ccr)*np.eye(3)
    def apply_colorcc(img):
        return  (img.reshape((-1,3)) @ c_mat.T).clip(0,255).reshape(img.shape).astype('uint8')

if args.pub_data:
    socket_pub = utils.publisher(config.zmq_local_route,'0.0.0.0')


base_name = os.path.basename(args.path)

imbuff=[None for i in range(50)]


def get_first_and_last(pkl_file):
    fd = open(pkl_file,'rb')
    start = -1
    end = -1
    while 1:
        try:
            ret=pickle.load(fd)
            #print('----',ret)
        except EOFError:
            break
        if ret[0]==topics.topic_viewer_data:
            msg=ret[1]
            if start==-1:
                start=msg['frame_cnt'][0]
            end=msg['frame_cnt'][0]
    return start,end

if __name__=='__main__':
    print('nosync',args.nosync)
    reader = gst_file_reader(args.path,nosync = args.nosync)

    start_frame,end_frame = get_first_and_last(args.path+'/viewer_data.pkl')
    print('start_frame,end_frame',start_frame,end_frame)

    fd = open(args.path+'/viewer_data.pkl','rb')
    
    sx,sy=config.cam_res_rgbx,config.cam_res_rgby
    bmargx,bmargy=config.viewer_blacks
    messages={}
    messages_hist = []
    fcnt=-1
    from_buff=False
    save_avi = None
    
    while 1:
        join=np.zeros((sy+bmargy,sx*2+bmargx,3),'uint8')
        hist_buff_ind=fcnt%len(imbuff)
        if imbuff[hist_buff_ind]!=None and imbuff[hist_buff_ind][0]==fcnt:
            fcnt,images,messages=imbuff[hist_buff_ind]
            from_buff=True
        else:
            images,fcnt=reader.__next__()
            from_buff=False
            #print('fnum in image',fcnt)
            while fcnt>-1:
                try:
                    ret=pickle.load(fd)
                    print('got',ret[0])
                except EOFError:
                    print('No more data')
                    break
                if args.pub_data:
                    socket_pub.send_multipart([ret[0],pickle.dumps(ret[1])])
                if ret[0]==topics.topic_viewer_data:
                    print('got',ret)
                    msg=ret[1]
                    if msg['frame_cnt'][1]>=fcnt:
                        break
                else:
                    messages[ret[0]]=ret[1]
            if fcnt>0:
                hist_buff_ind=fcnt%len(imbuff)
                imbuff[hist_buff_ind]=(fcnt,images,copy.deepcopy(messages))

        imgs_raw=[None,None]
        if fcnt >0 and images[0] is not None and images[1] is not None:
            #if 1 or not  from_buff:
            for i in [0,1]:
                if not args.nosingle and args.nowait <= fcnt:
                    for ext in ['.ppm','.png','.webp','.pgm']:
                        fname=file_path_fmt.format('lr'[i],fcnt)+ext
                        if os.path.isfile(fname):
                            try:
                                frame=cv2.imread(fname)
                                if ext=='.pgm':
                                    frame=cv2.cvtColor(frame[:,:,0].copy(), cv2.COLOR_BAYER_BG2RGB_EA)[:,:,::-1]
                                if args.cc is not None:
                                    frame=apply_colorcc(frame)
                                imgs_raw[i]=frame
                                images[i]=imgs_raw[i][::2,::2,:].copy()
                            except Exception:
                                print('Failed read frame',fname)
                                traceback.print_exc(file=sys.stdout)
                            break

                    #images[i]=cv2.imread(fname)[:,:,::-1].copy()
                if imgs_raw[i] is None:
                    imgs_raw[i]=images[i].copy()#[:,:,::-1].copy()
                imgs_raw[i]=imgs_raw[i][:,:,::-1]
            draw_seperate(images[0],images[1],messages)
            #join[:,0:sx,:]=images[0]
            #join[:,sx:,:]=images[1]
            join[bmargy//2:-bmargy//2,bmargx:sx+bmargx,:]=images[0]
            join[bmargy//2:-bmargy//2,sx+bmargx:,:]=images[1]
            draw(join,messages,fcnt,fcnt)
            if explore.is_data_file(args.path,fcnt):
                cv2.circle(join,(0,0), 15, (0,0,255), -1)

            if save_avi is not None:
                save_avi.write(join)

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(join,'Play {:.1f} tot {:.1f} sec'.format((fcnt-start_frame)/10 ,(end_frame-start_frame)/10)
                ,(20,32), font, 0.4,(255,0,255),1,cv2.LINE_AA)

            cv2.imshow('3dviewer '+base_name,join)
            #cv2.imshow('left',images[0])
            #cv2.imshow('right',images[1])
        if args.nowait > fcnt:
            k=cv2.waitKey(1)
            if images is not None:
                fcnt+=1
        else:
            k=cv2.waitKey(0)
        if k%256==ord('q'):
            break
        if k%256==ord('i'):
            explore.plot_raw_images(imgs_raw,args.path,fcnt)
        if k%256==8:
            fcnt-=1
        if k%256==ord(' '):
            fcnt+=1
        if k%258==ord('x'):
            cv2.imwrite('out{:08d}.png'.format(fcnt),join)
        if k%256==ord('s'):
            #import pdb;pdb.set_trace()
            save_sy,save_sx=images[0].shape[:2]
            save_sx*=2
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            #fourcc = cv2.VideoWriter_fourcc(
            save_avi = cv2.VideoWriter('./output.avi', fourcc , 20.0, (save_sx,save_sy))
        if k%256==ord('t'):
            track.debug=fcnt
    if save_avi is not None:
        save_avi.release()
        print('to convert to webm run:')
        print('ffmpeg -i output.avi -cpu-used 2 -b:v 1M output.webm')


### fmt_cnt_l,imgl,imgr=imgget.__next__()
###                fmt_cnt_r=fmt_cnt_l
###                img=imgl
