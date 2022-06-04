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
elif config.rov_type==4:
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
        line+=' {:>.2f}Cfps'.format(cfps)
        cv2.putText(img,line,(sy(10),sx(575+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_imu in message_dict:
        m=message_dict[zmq_topics.topic_imu]
        yaw,pitch,roll=m['yaw'],m['pitch'],m['roll']
        draw_compass(img,sy(1000+155),sx(438+voff),yaw,pitch,roll)
    if zmq_topics.topic_depth in message_dict:
        target_depth = message_dict.get(zmq_topics.topic_depth_hold_pid,{}).get('T',0)
        draw_depth(img,0,0,message_dict[zmq_topics.topic_depth]['depth'],target_depth)
    if zmq_topics.topic_sonar in message_dict and 'sonar' in message_dict[zmq_topics.topic_sonar]:
        sonar_rng = message_dict[zmq_topics.topic_sonar]['sonar']
        line=' {:>.2f},{:>.2f} SRng'.format(*sonar_rng)
        cv2.putText(img,line,(sy(450),sx(575+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_record_state in message_dict:
        if message_dict[zmq_topics.topic_record_state]:
            cv2.putText(img,'REC',(sy(25),sx(20)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_system_state in message_dict:
        ss = message_dict[zmq_topics.topic_system_state][1]
        cv2.putText(img,'ARM' if ss['arm'] else 'DISARM' \
                ,(sy(100),sx(20)), font, 0.7,(0,0,255) if ss['arm'] else (0,255,0),2,cv2.LINE_AA)
        modes = sorted(ss['mode'])
        if len(modes)==0:
            modes_str='MANUAL'
        else:
            modes_str=' '.join(modes)
        cv2.putText(img, modes_str\
                ,(sy(240),sx(20)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_tracker in message_dict:
        rng  = message_dict[zmq_topics.topic_tracker].get('range_f',-1.0)
        line='{:>.2f} TRng'.format(rng)
        cv2.putText(img,line,(sy(400),sx(600+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_telem in message_dict:
        v=message_dict[zmq_topics.topic_telem]['V']
        i=message_dict[zmq_topics.topic_telem]['I']
        leak=message_dict[zmq_topics.topic_telem]['leak']
        if leak:
            pos=(90, 90)
            cv2.circle(img, pos, 52, (0, 0, 255), -1)
            cv2.putText(img,"LEAK!",(pos[0]-42, pos[1]+10), font, 1.0,(255,255,255),3,cv2.LINE_AA)
        line='{:>.2f}V {:>.2f}I'.format(v,i)
        cv2.putText(img,line,(sy(50),sx(600+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_hw_stats in message_dict:
        line=hw_stats_tools.get_hw_str(message_dict[zmq_topics.topic_hw_stats][1])
        cv2.putText(img,line,(sy(670+500),sx(580+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_thrusters_comand in message_dict:
        thrst_cmnd = message_dict[zmq_topics.topic_thrusters_comand][1]
        draw_thrusters(img, (650, 350), thrst_cmnd)
    if zmq_topics.topic_lights in message_dict:
        lights=message_dict[zmq_topics.topic_lights]
        cv2.putText(img,'L'+str(lights),(sy(670+360),sx(580+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)

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
        cv2.putText(img,line,(sy(10),sx(560+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
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
        cv2.putText(img,line,(sy(450),sx(560+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_record_state in message_dict:
        if message_dict[zmq_topics.topic_record_state]:
            cv2.putText(img,'REC',(sy(10),sx(20)), font, 0.7,(0,0,255),2,cv2.LINE_AA)
    if zmq_topics.topic_system_state in message_dict:
        ss = message_dict[zmq_topics.topic_system_state][1]
        cv2.putText(img,'ARM' if ss['arm'] else 'DISARM' \
                ,(sy(50),sx(20)), font, 0.7,(0,0,255) if ss['arm'] else (0,255,0),2,cv2.LINE_AA)
        modes = sorted(ss['mode'])
        if len(modes)==0:
            modes_str='MANUAL'
        else:
            modes_str=' '.join(modes)
        cv2.putText(img, modes_str\
                ,(sy(140),sx(15)), font, 0.7,(255,255,255),2,cv2.LINE_AA)
    if zmq_topics.topic_tracker in message_dict:
        rng  = message_dict[zmq_topics.topic_tracker].get('range_f',-1.0)
        line='{:>.2f} TRng'.format(rng)
        cv2.putText(img,line,(sy(350),sx(580+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)

    if zmq_topics.topic_volt in message_dict:
        v=message_dict[zmq_topics.topic_volt]['V']
        i=message_dict[zmq_topics.topic_volt]['I']
        line='{:>.2f}V {:>.2f}I'.format(v,i)
        cv2.putText(img,line,(sy(50),sx(580+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)

    if zmq_topics.topic_hw_stats in message_dict:
        line=hw_stats_tools.get_hw_str(message_dict[zmq_topics.topic_hw_stats][1])
        cv2.putText(img,line,(sy(670+500),sx(580+voff)), font, 0.7,(255,255,255),2,cv2.LINE_AA)

from math import cos,sin,pi
def draw_compass(img,x,y,heading,pitch,roll,rr=50.0):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,str(int(heading%360)),(x-12,y+26), font, 0.5,(0,000,255),2,cv2.LINE_AA)

    cv2.circle(img, (x,y), int(rr), (0,0,255), 2)
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
                (int(x+cs*rr),int(y+si*rr)),(0,0,255),2)

    t=(heading-90)/180.0*pi
    cs=cos(t)
    si=sin(t)
    mt=3
    cv2.line(img,
        (int(x+cs*(rr-mt)),int(y+si*(rr-mt))),
        (int(x+cs*rr),int(y+si*rr)),(255,255,255),10)

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
        (int(xx+cs*r),int(yy+si*r)),(0,255,255),6)
    cv2.line(img,
        (int(xx-cs*(r-mt)),int(yy-si*(r-mt))),
        (int(xx-cs*r),int(yy-si*r)),(0,255,255),5)
    cv2.putText(img,'Y:'+str(int(heading)),(x+int(rr)+5,y-15), font, 0.6,(0,0,0),6,cv2.LINE_AA)
    cv2.putText(img,'P:'+str(int(pitch)),(x+int(rr)+5,y+5), font, 0.6,(0,0,0),6,cv2.LINE_AA)
    cv2.putText(img,'R:'+str(int(roll)),(x+int(rr)+5,y+25), font, 0.6,(0,0,0),6,cv2.LINE_AA)
    cv2.putText(img,'Y:'+str(int(heading)),(x+int(rr)+5,y-15), font, 0.6,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(img,'P:'+str(int(pitch)),(x+int(rr)+5,y+5), font, 0.6,(255,255,255),2,cv2.LINE_AA)
    cv2.putText(img,'R:'+str(int(roll)),(x+int(rr)+5,y+25), font, 0.6,(255,255,255),2,cv2.LINE_AA)


from math import radians
import numpy as np
cosd = lambda d: cos(radians(d))
sind = lambda d: sin(radians(d))
rot = lambda ang: np.array([[cosd(ang), sind(ang)],
                            [-sind(ang), cosd(ang)]])
def draw_thrusters(img, pos, thrst_cmnds):
    t_commands = np.array(thrst_cmnds)
    DS = 1.0
    DISP_S = 240
    s_img = np.zeros((DISP_S, DISP_S, 3), dtype=np.uint8)

    t_commands_uint8 = (20 + 215 * abs(t_commands)).astype(np.uint8)
    t_cols = cv2.applyColorMap(t_commands_uint8, cv2.COLORMAP_TURBO).astype(int)
    
    # Draw vertical thrusters
    U_H = 100
    U_W = 160
    for t_i in range(4):
        comm_str = str(round(t_commands[t_i], 1))
        comm_str = ' ' + comm_str if len(comm_str) == 3 else comm_str
        c_x = int((DISP_S-U_W) // 2 + U_W * ((t_i % 2) if t_i < 2 else (t_i % 2 == 0)))
        c_y = int((DISP_S-U_H) // 2 + U_H * (t_i > 1))
        cv2.circle(s_img, (c_x, c_y), 20, t_cols[t_i][0].tolist(), -1)
        if t_commands[t_i] > 0:
            cv2.putText(s_img, 'x', (c_x - 14, c_y + 11), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (1, 1, 1), 10)
        else:
            cv2.circle(s_img, (c_x, c_y), 15, (1, 1, 1), -1)
        cv2.putText(s_img, comm_str, (c_x - 18, c_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 6)
        cv2.putText(s_img, comm_str, (c_x - 18, c_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (1, 1, 1), 2)

    # Draw horizontal thrusters
    L_H = 140
    L_W = 60
    T_ANGLES = [60, -240, -300, 120]
    for t_i in range(4):
        comm_str = str(round(t_commands[t_i+4], 1))
        comm_str = ' ' + comm_str if len(comm_str) == 3 else comm_str
        c_x = int((DISP_S-L_W) // 2 + L_W * ((t_i % 2) if t_i < 2 else (t_i % 2 == 0)))
        c_y = int((DISP_S-L_H) // 2 + L_H * (t_i > 1))
        M, m = (20, 13)
        contours = np.array([[-m, -M], [m, -M],
                             [m, M], [-m, M]])
        arrow_p = np.array([[0, -8],
                            [25 * abs(t_commands[t_i + 4]), -8],
                            [25 * abs(t_commands[t_i + 4]), -13],
                            [40 * abs(t_commands[t_i + 4]), 0],
                            [25 * abs(t_commands[t_i + 4]), 13],
                            [25 * abs(t_commands[t_i + 4]), 8],
                            [0, 8]])
        arrow_p[:, 0] += m
        arrow_p[:, 0] *= np.sign(t_commands[t_i + 4])
        contours = np.matmul(rot(T_ANGLES[t_i]), contours.T).T.astype(int)
        arrow = np.matmul(rot(T_ANGLES[t_i]), arrow_p.T).T.astype(int)
        contours[:, 0] += c_x
        arrow[:, 0] += c_x
        contours[:, 1] += c_y
        arrow[:, 1] += c_y
        cv2.fillPoly(s_img, [contours], t_cols[t_i+4][0].tolist())
        cv2.fillPoly(s_img, [arrow], t_cols[t_i+4][0].tolist())
        cv2.putText(s_img, comm_str, (c_x - 18, c_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 6)
        cv2.putText(s_img, comm_str, (c_x - 18, c_y + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (1, 1, 1), 2)

    img_ds = cv2.resize(s_img, (int(DISP_S/DS), int(DISP_S//DS)))
    img_ds_mask = np.sum(img_ds, axis=-1) > 0
    s_img = img[pos[1]:pos[1]+img_ds.shape[1], pos[0]:pos[0]+img_ds.shape[0]]
    s_img[img_ds_mask] = img_ds[img_ds_mask]


def draw_depth(img,x,y,depth,tdepth):
    l=450
    s=15
    cv2.line(img,(x,y),(x,y+l),(0,0,255), 2)
    for i in range(0,l+1,s):
        if (i//s)%5==0:
            mt=15
        else:
            mt=5
        cv2.line(img,(x,y+i),(x+mt,y+i),(0,0,255), 2)

    d=int(depth*s)
    dt=int(tdepth*s)
    cv2.line(img,(x,y+dt),(x+15,y+dt),(0,100,100),thickness=5)
    cv2.line(img,(x,y+d),(x+15,y+d),(0,255,255),thickness=4)
    cv2.line(img,(x,y+d+1),(x+15,y+d+1),(255,0,255))

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img,'%.2f'%depth \
            ,(x,l+y+20), font, 0.7,(255,0,255),2,cv2.LINE_AA)
