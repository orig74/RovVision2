# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import cv2
import time
import config
import zmq_topics
import sys
sys.path.append('../plugins')
sys.path.append('../plugins/tracker')
from tracker import tracker


fps_time=time.time()
frame_start_time=None
frame_start_number=None
fps_last_num=0
fps=None
def draw_seperate(imgl,imgr,message_dict):
    if zmq_topics.topic_tracker in message_dict:
        tracker.draw_track_rects(message_dict[zmq_topics.topic_tracker],imgl,imgr)

def draw(img,message_dict,fmt_cnt_l,fmt_cnt_r):
    global fps_time,fps,fps_last_num,frame_start_time,frame_start_number
    font = cv2.FONT_HERSHEY_SIMPLEX
    #print('-2-',md)
    #line1='{:08d}'.format(fmt_cnt_l)
    #cv2.putText(img,line1,(10,50), font, 0.5,(0,0,255),1,cv2.LINE_AA)


    #if vd.get('record_state',False):
    #    cv2.putText(img,'REC '+vd['disk_usage'],(10,200),font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if frame_start_time is None:
        frame_start_time=time.time()
        frame_start_number=fmt_cnt_l
    if fmt_cnt_l is not None:
        line=' {:>8}'.format(fmt_cnt_l)
        if fmt_cnt_l%100==0 and fmt_cnt_l!=fps_last_num:
            fps=100.0/(time.time()-fps_time)
            #print('---',time.time()-fps_time)
            fps_time=time.time()
            fps_last_num=fmt_cnt_l
        if fps is not None:
            line+=' {:>.2f}fps'.format(fps)
            line+=' {:>05.2f}delay'.format(\
                (fmt_cnt_l-frame_start_number)-\
                config.fps*(time.time()-frame_start_time))
        cv2.putText(img,line,(10,550), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_imu in message_dict:
        m=message_dict[zmq_topics.topic_imu]
        yaw,pitch,roll=m['yaw'],m['pitch'],m['roll']
        draw_compass(img,1000,500,yaw,pitch,roll)
    if zmq_topics.topic_depth in message_dict:
        target_depth = message_dict.get(zmq_topics.topic_depth_hold_pid,{}).get('T',0)
        draw_depth(img,0,0,message_dict[zmq_topics.topic_depth]['depth'],target_depth)
    if zmq_topics.topic_sonar in message_dict:
        sonar_rng = message_dict[zmq_topics.topic_sonar]
        line=' {:>.2f},{:>.2f}Rng'.format(*sonar_rng)
        cv2.putText(img,line,(350,550), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.record_state in message_dict:
        if message_dict[zmq_topics.record_state]:
            cv2.putText(img,'REC',(10,15), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_system_state in message_dict:
        ss = message_dict[zmq_topics.topic_system_state][1]
        cv2.putText(img,'ARM' if ss['arm'] else 'DISARM' \
                ,(50,15), font, 0.5,(0,0,255) if ss['arm'] else (0,255,0),1,cv2.LINE_AA)
        modes = sorted(ss['mode'])
        if len(modes)==0:
            modes_str='MANUAL'
        else:
            modes_str=' '.join(modes)
        cv2.putText(img, modes_str\
                ,(140,15), font, 0.5,(0,255,0),1,cv2.LINE_AA)

from math import cos,sin,pi
def draw_compass(img,x,y,heading,pitch,roll):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,str(int(heading%360)),(x,y+30), font, 0.5,(0,000,255),1,cv2.LINE_AA)

    r=50.0
    #mt=r-3


    cv2.circle(img, (x,y), int(r), (0,0,255), 1)
    for i in range(36):
        t=i*10/180.0*pi
        if i%9==0:
            mt=10
        elif i%3==0:
            mt=5
        else:
            mt=2
        cs=cos(t)
        si=sin(t)
        cv2.line(img,
                (int(x+cs*(r-mt)),int(y+si*(r-mt))),
                (int(x+cs*r),int(y+si*r)),(0,0,255),1)

    t=(heading-90)/180.0*pi
    cs=cos(t)
    si=sin(t)
    mt=3
    cv2.line(img,
        (int(x+cs*(r-mt)),int(y+si*(r-mt))),
        (int(x+cs*r),int(y+si*r)),(0,255,255),3)

    r=30
    mt=5
    hfov=40/180.0*pi/2.0*config.cam_resy/2.0
    pt=pitch/180.0*pi
    xx=x
    yy=y-sin(pt)*hfov
    rl=-roll/180.0*pi
    cs=cos(rl)
    si=sin(rl)
    cv2.line(img,
        (int(xx+cs*(r-mt)),int(yy+si*(r-mt))),
        (int(xx+cs*r),int(yy+si*r)),(0,255,255),2)
    cv2.line(img,
        (int(xx-cs*(r-mt)),int(yy-si*(r-mt))),
        (int(xx-cs*r),int(yy-si*r)),(0,255,255),2)
    cv2.putText(img,'Y:'+str(int(heading)),(x-3,y+60), font, 0.5,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(img,'P:'+str(int(pitch)),(x-3,y+75), font, 0.5,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(img,'R:'+str(int(roll)),(x-3,y+90), font, 0.5,(0,255,255),1,cv2.LINE_AA)




def draw_depth(img,x,y,depth,tdepth):
    l=450
    s=15
    cv2.line(img,(x,y),(x,y+l),(0,0,255))
    for i in range(0,l+1,s):
        if (i//s)%5==0:
            mt=10
        else:
            mt=3
        cv2.line(img,(x,y+i),(x+mt,y+i),(0,0,255))

    d=int(depth*s)
    dt=int(tdepth*s)
    cv2.line(img,(x,y+dt),(x+10,y+dt),(0,100,100),thickness=3)
    cv2.line(img,(x,y+d),(x+10,y+d),(0,255,255),thickness=2)
    cv2.line(img,(x,y+d+1),(x+10,y+d+1),(255,0,255))

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,'%.2f'%depth \
            ,(x,l+y+20), font, 0.5,(255,0,255),1,cv2.LINE_AA)


