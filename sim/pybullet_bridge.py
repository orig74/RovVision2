#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys,os,time
import pybullet as pb
import pybullet_data
sys.path.append('..')
sys.path.append('../utils')
sys.path.append('../hw')
sys.path.append('unreal_proxy')
import zmq
from dvl import parse_line
import struct
import cv2,os
import numpy as np
import pickle
import zmq_wrapper as utils
import ue4_zmq_topics
import zmq_topics
import config
import dill
from numpy import sin,cos

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_dvl_cmd],zmq_topics.topic_controller_port))

zmq_pub=utils.publisher(zmq_topics.topic_camera_port)
zmq_pub_ts = utils.publisher(zmq_topics.topic_camera_ts_port)
pub_imu = utils.publisher(zmq_topics.topic_imu_port)
pub_depth = utils.publisher(zmq_topics.topic_depth_port)
pub_dvl = utils.publisher(zmq_topics.topic_dvl_port)

pub_sonar = utils.publisher(zmq_topics.topic_sonar_port)
cvshow=0
#cvshow=False
test=1

dill.settings['recurse'] = True
lamb=dill.load(open('../sim/lambda.pkl','rb'))
current_command=[0 for _ in range(8)] # 8 thrusters
dt=1/60.0
#pybullet init
render = pb.DIRECT # pb.GUI
#physicsClient = pb.connect(pb.GUI)#or p.DIRECT for non-graphical version
physicsClient = pb.connect(render)#or p.DIRECT for non-graphical version
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pb.setGravity(0,0,-0)
print('start...')
planeId = pb.loadURDF("plane.urdf")
pb.resetBasePositionAndOrientation(planeId,(0,0,-20),pb.getQuaternionFromEuler((0,0,0,)))
import random
robj=[]
def set_random_objects():
    random.seed(0)
    for _ in range(400):
        urdf = "random_urdfs/{0:03d}/{0:03d}.urdf".format(random.randint(1,1000)) 
        x = (random.random()-0.5)*5
        y = (random.random()-0.5)*5
        z = (random.random())*-10
        obj = pb.loadURDF(urdf,basePosition=[x,y,z],globalScaling=3,baseOrientation=[random.random() for _ in range(4)])
        robj.append(obj)
        print('---loading--',urdf,x,y)
        #pb.resetBasePositionAndOrientation(obj,(x,y,z),pb.getQuaternionFromEuler((0,0,0,)))

set_random_objects()
keep_running = True

def getrov():

    shift = [0, -0.0, 0]
    meshScale = np.array([0.01, 0.01, 0.01])*0.3
    vfo=pb.getQuaternionFromEuler(np.deg2rad([90, 0, 0]))
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                        fileName="br1.obj",
                                        rgbaColor=[1, 0, 0, 1],
                                        specularColor=[0.4, .4, 0],
                                        visualFramePosition=[0,0,0],
                                        visualFrameOrientation=vfo,
                                        meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    boxId=pb.createMultiBody(baseMass=1,
                          baseInertialFramePosition=[0, 0, 0],
                          baseCollisionShapeIndex=1,
                          baseVisualShapeIndex=visualShapeId,
                          basePosition=[0,0,0],
                          useMaximalCoordinates=True)
    return boxId

def _get_next_state(curr_q,curr_u,control,dt,lamb):
    forces=control
    #print('forces = ',forces)
    u_dot_f=lamb(curr_q,curr_u,*forces).flatten()
    next_q=curr_q+curr_u*dt
    next_u=curr_u+u_dot_f*dt
    return next_q,next_u

from scipy.interpolate import interp1d
pts_16v_t200 = [(1100,-4.07) , (1184, -2.94), (1308, -1.35) , (1472,0) , (1528,0), 
                (1624,0.87) , (1728,2.22), (1840,4.25), (1864,4.71), (1900,5.25)]
pwm,thrust = zip(*pts_16v_t200)
thrust=np.array(thrust)/0.101972 #kgf to newton
pwm_to_thrust=interp1d(pwm, thrust)
def scale_thrust(control):
    return np.array([pwm_to_thrust(c*400+1500)/10 for c in control])

def get_next_state(curr_q,curr_u,control,dt,lamb):
    control = np.clip(control,-1,1)
    forces=scale_thrust(control)
    currents_vector = [0,0.1,0]
    u_dot_f=lamb(curr_q,curr_u,*forces,*currents_vector).flatten()
    next_q=curr_q+curr_u*dt
    next_u=curr_u+u_dot_f*dt
    return next_q,next_u


def translateM(M,dx,dy,dz):
    T = np.zeros((4,4),dtype=float)
    T[np.diag_indices(4)]=1.0
    T[0,3]=dx
    T[1,3]=dy
    T[2,3]=dz
    #VM = T @ np.array(M).reshape((4,4)) 
    VM =  np.array(M).reshape((4,4)) @ T.T
    VM = VM.flatten().tolist()
    return VM

def resize(img,factor):
    h,w = img.shape[:2]
    return cv2.resize(img,(int(w*factor),int(h*factor)))

def main():
    cnt=0
    frame_cnt=0
    frame_ratio=6 # for 6 sim cycles 1 video frame
    resize_fact=0.5
    mono=False
    imgl = None
    curr_q = np.zeros(6)
    curr_u = np.zeros(6)
    current_command = np.zeros(8)

    dvl_offset=np.zeros(3)
    dvl_angle_offsets=np.zeros(3)
    dvl_cmd=None

    if render==pb.GUI:
        boxId = getrov()

    while keep_running:
        tic_cycle = time.time()
        socks = zmq.select(subs_socks,[],[],0.001)[0]
        for sock in socks:
            data = sock.recv_multipart()
            topic=data[0]
            if topic==zmq_topics.topic_thrusters_comand:
                _,current_command=pickle.loads(data[1])
                current_command=[i*1.3 for i in current_command]
            if topic==zmq_topics.topic_dvl_cmd:
                dvl_cmd=data[1]

        next_q,next_u=get_next_state(curr_q,curr_u,current_command,dt,lamb)
        next_q,next_u=next_q.flatten(),next_u.flatten()
        curr_q,curr_u=next_q,next_u

        ps={}
        #print('dsim {:4.2f} {:4.2f} {:4.2f} {:3.1f} {:3.1f} {:3.1f}'.format(*curr_q),current_command)
        ps['posx'],ps['posy'],ps['posz']=curr_q[:3]
        yaw,roll,pitch = curr_q[3:]
        ps['yaw'],ps['roll'],ps['pitch']=np.rad2deg(curr_q[3:])
        #ps['yaw']=-ps['yaw']
        #ps['posy']=-ps['posy']
    #ps['pitch']=-ps['pitch'] 
        #ps['roll']=-ps['roll']
        #pub_pos_sim.send_multipart([xzmq_topics.topic_sitl_position_report,pickle.dumps((time.time(),curr_q))])
        zmq_pub.send_multipart([ue4_zmq_topics.topic_sitl_position_report,pickle.dumps(ps)])
        px,py,pz=ps['posx'],ps['posy'],ps['posz']

        if cnt%frame_ratio==0:
            #print('====',yaw,pitch,roll)
            #first camera
            yawd,pitchd,rolld=ps['yaw'],ps['roll'],ps['pitch']
            VM = pb.computeViewMatrixFromYawPitchRoll((py,px,-pz),-0.1,-yawd,pitchd,rolld,2)
            if not mono:
                VML=translateM(VM,0.1,0,0.0)#left camera 0.2 for left 
            else:
                VML=VM
            PM = pb.computeProjectionMatrixFOV(fov=60.0,aspect=1.0,nearVal=0.1,farVal=1000)
            w = int(config.cam_res_rgbx*resize_fact)
            h = int(config.cam_res_rgby*resize_fact)
            width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=w, 
                height=h,
                viewMatrix=VML,
                projectionMatrix=PM,renderer = pb.ER_BULLET_HARDWARE_OPENGL)
                    #get images from py bullet
            imgl=resize(rgbImg[:,:,:3],1/resize_fact)#inly interested in rgb
            #print('===',imgl.shape)

            #second camera
            if not mono:
                #VM = pb.computeViewMatrixFromYawPitchRoll((py,px,-pz),1.0,-yawd,pitchd,rolld,2)
                VMR=translateM(VM,-0.10,0.00,0.0)#left camera 0.2 for left 
                PM = pb.computeProjectionMatrixFOV(fov=60.0,aspect=1.0,nearVal=0.1,farVal=1000)
                width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                    width=w, 
                    height=h,
                    viewMatrix=VMR,
                    projectionMatrix=PM,renderer = pb.ER_BULLET_HARDWARE_OPENGL)
                imgr=resize(rgbImg[:,:,:3],1/resize_fact) #todo...
                        

            if cvshow:
                #if 'depth' in topic:
                #    cv2.imshow(topic,img)
                #else:
                #cv2.imshow(topic,cv2.resize(cv2.resize(img,(1920/2,1080/2)),(512,512)))
                imgls = imgl[::2,::2]
                imgrs = imgr[::2,::2]
                cv2.imshow('l',imgls)
                cv2.imshow('r',imgrs)
                cv2.waitKey(1)
            if mono:
                zmq_pub.send_multipart([zmq_topics.topic_stereo_camera,pickle.dumps([frame_cnt,imgl.shape]),imgl.tostring()])
            else:
                zmq_pub.send_multipart([zmq_topics.topic_stereo_camera,pickle.dumps([frame_cnt,imgl.shape]),imgl.tostring(),imgr.tostring()])
            time.sleep(0.001) 
            zmq_pub_ts.send_multipart([zmq_topics.topic_stereo_camera_ts,pickle.dumps((frame_cnt,time.time(),time.time()))]) #for sync
                
            #get depth image
            depthImg=depthImg[::4,::4]
            min_range=depthImg.min() 
            #import pdb;pdb.set_trace()

            img_show=(depthImg/10.0).clip(0,255).astype('uint8')
            depthImg[depthImg>5000]=np.nan
            max_range=np.nanmax(depthImg)
            #print('sonar::',min_range,max_range)
            tosend = pickle.dumps({'ts':time.time(), 'sonar':[(min_range + max_range) / 2, 1.0]})
            pub_sonar.send_multipart([zmq_topics.topic_sonar,tosend])

            if cvshow:
                cv2.imshow('depth',img_show)
                cv2.waitKey(1)
            frame_cnt+=1

            #print('====',px,py,pz,roll,pitch,yaw)
            #pb.resetBasePositionAndOrientation(boxId,(px,py,pz),pb.getQuaternionFromEuler((roll,pitch,yaw)))
            if render==pb.GUI:
                pb.resetBasePositionAndOrientation(boxId,(py,px,-pz),pb.getQuaternionFromEuler((roll,-pitch,-yaw+np.radians(0))))
            ### test
            tic=time.time()
            imu={'ts':tic}
            imu['yaw'],imu['pitch'],imu['roll']=np.rad2deg(curr_q[3:])
            #rates from dsym notebook
            #R.ang_vel_in(R).express(R).to_matrix(R)
            #good video in https://www.youtube.com/watch?v=WZEFoWP0Tzs
            q3,q4,q5=curr_q[3:]
            u3,u4,u5=curr_u[3:]
            imu['rates']=(
                    -u3*sin(q4) + u5,
                    u3*sin(q5)*cos(q4) + u4*cos(q5),
                    u3*cos(q4)*cos(q5) - u4*sin(q5))
            pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])
            pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':curr_q[2]})])

            if cnt%1==0:
                vx,vy,vz = curr_u[:3]
                #simulate dvl messgaes
                yaw_off=-dvl_angle_offsets[0]#-np.pi/2
                c,s = np.cos(yaw_off),np.sin(yaw_off)
                vx,vy = vx*c-vy*s,vx*s+vy*c
                vel_msg='wrz,{},{},{},y,1.99,0.006,3.65e-05;3.39e-06;7.22e-06;3.39e-06;2.46e-06;-8.5608e-07;7.223e-06;-8.560e-07;3.2363e-06,1550139816188624,1550139816447957,188.80,0*XX\r\n'.format(vx,vy,vz).encode()
                pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps({'ts':tic,'dvl_raw':vel_msg})])
                d=parse_line(vel_msg)
                pub_dvl.send_multipart([zmq_topics.topic_dvl_vel, pickle.dumps(d)])

            if cnt%1==0:
                x,y,z=curr_q[:3]-dvl_offset
                yaw_off=-dvl_angle_offsets[0]#-np.pi/2
                c,s = np.cos(yaw_off),np.sin(yaw_off)
                x,y = x*c-y*s,x*s+y*c

                pos_msg='wrp,1550139816.178,{},{},{},{},2.5,-3.7,-62.5,0*XX\r\n'.\
                        format(x,y,z,100).encode()
                pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps({'ts':tic,'dvl_raw':pos_msg})])
        
        if dvl_cmd is not None:
            if dvl_cmd==b'wcr\n':
                print('got dvl reset')
                dvl_offset=curr_q[:3]
                dvl_angle_offsets=curr_q[3:]
            dvl_cmd=None


        time.sleep(0.010)
        if cnt%20==0 and imgl is not None:
            print('send...',cnt, imgl.shape)
        cnt+=1


       
if __name__ == '__main__':
    main()
