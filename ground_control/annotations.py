# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import cv2
import time
import config
import zmq_topics
import sys
sys.path.append('../plugins')
sys.path.append('../plugins/tracker')
if config.tracker=='rope':
    from tracker import rope_tracker as tracker
elif config.tracker=='local':
    from tracker import tracker
import hw_stats_tools

prev_fps_time=time.time()
prev_frame_cnt = None
cfps = 0

if config.rov_type==2:
    sy = lambda n: int(n*1280/1920)
    sx = lambda n: int(n*1024/1200)
else:
    sx = lambda x:x
    sy = lambda y:y

def draw_seperate(imgl,imgr,message_dict):
    if zmq_topics.topic_tracker in message_dict:
        tracker.draw_track_rects(message_dict[zmq_topics.topic_tracker],imgl,imgr)

def draw(img,message_dict,fmt_cnt_l,fmt_cnt_r):
    global prev_fps_time, prev_frame_cnt, cfps
    font = cv2.FONT_HERSHEY_SIMPLEX
    #print('-2-',md)
    #line1='{:08d}'.format(fmt_cnt_l)
    #cv2.putText(img,line1,(10,50), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    voff=113
    #if vd.get('record_state',False):
    #    cv2.putText(img,'REC '+vd['disk_usage'],(10,200),font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if prev_frame_cnt is None:
        prev_fps_time=time.time()
        prev_frame_cnt=fmt_cnt_l
    if fmt_cnt_l is not None:
        line=' {:>8}'.format(fmt_cnt_l)
        if fmt_cnt_l >= prev_frame_cnt+50:
            cfps = (fmt_cnt_l - prev_frame_cnt) / (time.time() - prev_fps_time)
            prev_frame_cnt = fmt_cnt_l
            prev_fps_time = time.time()
        line+=' {:>.2f}Cfps, {:>.2f}Rfps'.format(cfps, cfps / config.save_modulo)
        print('fpsline',line)
        cv2.putText(img,line,(sy(10),sx(560+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_imu in message_dict:
        m=message_dict[zmq_topics.topic_imu]
        yaw,pitch,roll=m['yaw'],m['pitch'],m['roll']
        draw_compass(img,sy(1000+100),sx(485+voff),yaw,pitch,roll)
    if zmq_topics.topic_depth in message_dict:
        target_depth = message_dict.get(zmq_topics.topic_depth_hold_pid,{}).get('T',0)
        draw_depth(img,0,0,message_dict[zmq_topics.topic_depth]['depth'],target_depth)
    if zmq_topics.topic_sonar in message_dict and 'sonar' in message_dict[zmq_topics.topic_sonar]:
        sonar_rng = message_dict[zmq_topics.topic_sonar]['sonar']
        line=' {:>.2f},{:>.2f} SRng'.format(*sonar_rng)
        cv2.putText(img,line,(sy(450),sx(560+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_record_state in message_dict:
        if message_dict[zmq_topics.topic_record_state]:
            cv2.putText(img,'REC',(sy(10),sx(15)), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_system_state in message_dict:
        ss = message_dict[zmq_topics.topic_system_state][1]
        cv2.putText(img,'ARM' if ss['arm'] else 'DISARM' \
                ,(sy(50),sx(15)), font, 0.5,(0,0,255) if ss['arm'] else (0,255,0),1,cv2.LINE_AA)
        modes = sorted(ss['mode'])
        if len(modes)==0:
            modes_str='MANUAL'
        else:
            modes_str=' '.join(modes)
        cv2.putText(img, modes_str\
                ,(sy(140),sx(15)), font, 0.5,(255,255,255),1,cv2.LINE_AA)
    if zmq_topics.topic_tracker in message_dict:
        rng  = message_dict[zmq_topics.topic_tracker].get('range_f',-1.0)
        line='{:>.2f} TRng'.format(rng)
        cv2.putText(img,line,(sy(350),sx(580+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    if zmq_topics.topic_volt in message_dict:
        v=message_dict[zmq_topics.topic_volt]['V']
        i=message_dict[zmq_topics.topic_volt]['I']
        line='{:>.2f}V {:>.2f}I'.format(v,i)
        cv2.putText(img,line,(sy(50),sx(580+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    if zmq_topics.topic_hw_stats in message_dict:
        line=hw_stats_tools.get_hw_str(message_dict[zmq_topics.topic_hw_stats][1])
        cv2.putText(img,line,(sy(670+500),sx(580+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)

def draw_mono(img,message_dict,fmt_cnt_l):
    global prev_fps_time, prev_frame_cnt, cfps
    font = cv2.FONT_HERSHEY_SIMPLEX
    #print('-2-',md)
    #line1='{:08d}'.format(fmt_cnt_l)
    #cv2.putText(img,line1,(10,50), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    voff=0#113
    #if vd.get('record_state',False):
    #    cv2.putText(img,'REC '+vd['disk_usage'],(10,200),font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if prev_frame_cnt is None:
        prev_fps_time = time.time()
        prev_frame_cnt = fmt_cnt_l
    if fmt_cnt_l is not None:
        line=' {:>8}'.format(fmt_cnt_l)
        if fmt_cnt_l >= prev_frame_cnt+50:
            cfps = (fmt_cnt_l - prev_frame_cnt) / (time.time() - prev_fps_time)
            prev_frame_cnt = fmt_cnt_l
            prev_fps_time = time.time()
        line+=' {:>.2f}Cfps, {:>.2f}Rfps'.format(cfps, cfps / config.save_modulo)
        print('fpsline',line)
        cv2.putText(img,line,(sy(10),sx(560+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_imu in message_dict:
        m=message_dict[zmq_topics.topic_imu]
        yaw,pitch,roll=m['yaw'],m['pitch'],m['roll']
        draw_compass(img,img.shape[1]//2,img.shape[0]//2,yaw,pitch,roll,rr=150.0)
    if zmq_topics.topic_depth in message_dict:
        target_depth = message_dict.get(zmq_topics.topic_depth_hold_pid,{}).get('T',0)
        draw_depth(img,0,0,message_dict[zmq_topics.topic_depth]['depth'],target_depth)
    if zmq_topics.topic_sonar in message_dict:
        sonar_rng = message_dict[zmq_topics.topic_sonar]
        line=' {:>.2f},{:>.2f} SRng'.format(*sonar_rng)
        cv2.putText(img,line,(sy(450),sx(560+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_record_state in message_dict:
        if message_dict[zmq_topics.topic_record_state]:
            cv2.putText(img,'REC',(sy(10),sx(15)), font, 0.5,(0,0,255),1,cv2.LINE_AA)
    if zmq_topics.topic_system_state in message_dict:
        ss = message_dict[zmq_topics.topic_system_state][1]
        cv2.putText(img,'ARM' if ss['arm'] else 'DISARM' \
                ,(sy(50),sx(15)), font, 0.5,(0,0,255) if ss['arm'] else (0,255,0),1,cv2.LINE_AA)
        modes = sorted(ss['mode'])
        if len(modes)==0:
            modes_str='MANUAL'
        else:
            modes_str=' '.join(modes)
        cv2.putText(img, modes_str\
                ,(sy(140),sx(15)), font, 0.5,(255,255,255),1,cv2.LINE_AA)
    if zmq_topics.topic_tracker in message_dict:
        rng  = message_dict[zmq_topics.topic_tracker].get('range_f',-1.0)
        line='{:>.2f} TRng'.format(rng)
        cv2.putText(img,line,(sy(350),sx(580+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    if zmq_topics.topic_volt in message_dict:
        v=message_dict[zmq_topics.topic_volt]['V']
        i=message_dict[zmq_topics.topic_volt]['I']
        line='{:>.2f}V {:>.2f}I'.format(v,i)
        cv2.putText(img,line,(sy(50),sx(580+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    if zmq_topics.topic_hw_stats in message_dict:
        line=hw_stats_tools.get_hw_str(message_dict[zmq_topics.topic_hw_stats][1])
        cv2.putText(img,line,(sy(670+500),sx(580+voff)), font, 0.5,(0,0,255),1,cv2.LINE_AA)

from math import cos,sin,pi
def draw_compass(img,x,y,heading,pitch,roll,rr=50.0):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,str(int(heading%360)),(x,y+30), font, 0.5,(0,000,255),1,cv2.LINE_AA)

    cv2.circle(img, (x,y), int(rr), (0,0,255), 1)
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
                (int(x+cs*(rr-mt)),int(y+si*(rr-mt))),
                (int(x+cs*rr),int(y+si*rr)),(0,0,255),1)

    t=(heading-90)/180.0*pi
    cs=cos(t)
    si=sin(t)
    mt=3
    cv2.line(img,
        (int(x+cs*(rr-mt)),int(y+si*(rr-mt))),
        (int(x+cs*rr),int(y+si*rr)),(0,255,255),3)

    r=30
    mt=5
    hfov=40/180.0*pi/2.0*config.cam_resy/2.0
    pt=pitch/180.0*pi
    xx=x
    yy=y-sin(pt)*hfov
    rl=roll/180.0*pi
    cs=cos(rl)
    si=sin(rl)
    cv2.line(img,
        (int(xx+cs*(r-mt)),int(yy+si*(r-mt))),
        (int(xx+cs*r),int(yy+si*r)),(0,255,255),2)
    cv2.line(img,
        (int(xx-cs*(r-mt)),int(yy-si*(r-mt))),
        (int(xx-cs*r),int(yy-si*r)),(0,255,255),2)
    cv2.putText(img,'Y:'+str(int(heading)),(x-3,y+int(rr)+10), font, 0.5,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(img,'P:'+str(int(pitch)),(x-3,y+int(rr)+25), font, 0.5,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(img,'R:'+str(int(roll)),(x-3,y+int(rr)+40), font, 0.5,(0,255,255),1,cv2.LINE_AA)




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
