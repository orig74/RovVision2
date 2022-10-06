#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys,os,time
import pybullet as pb
import pybullet_data
sys.path.append('..')
sys.path.append('../utils')
sys.path.append('unreal_proxy')
import zmq
import struct
import cv2,os
import numpy as np
import pickle
import zmq_wrapper as utils
import ue4_zmq_topics
import zmq_topics
import config
import dill
import cv2
from dvl_sim import DVLSim
from numpy import sin,cos
from scipy.spatial.transform import Rotation as R

from mussels_scene import getscene

zmq_controller=utils.subscribe([zmq_topics.topic_dvl_cmd],zmq_topics.topic_controller_port)
zmq_sub=utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port)
zmq_pub=utils.publisher(zmq_topics.topic_camera_port)
pub_imu = utils.publisher(zmq_topics.topic_imu_port)
pub_depth = utils.publisher(zmq_topics.topic_depth_port)
pub_dvl = utils.publisher(zmq_topics.topic_dvl_port)

pub_sonar = utils.publisher(zmq_topics.topic_sonar_port)

dvlSim=DVLSim()
cvshow=0
#cvshow=False

dill.settings['recurse'] = True

#work arrond https://github.com/uqfoundation/dill/pull/406 suggested for loading python3.7 pickled in python 3.8
import platform
if int(platform.python_version_tuple()[1])>=8:
    dill._dill._reverse_typemap['CodeType'] = dill._dill._create_code

lamb=dill.load(open('../sim/lambda.pkl','rb'))
current_command=[0 for _ in range(8)] # 8 thrusters
dt=1/60.0
#pybullet init
render = pb.GUI if len(sys.argv)>1 and sys.argv[1]=='g' else pb.DIRECT # pb.GUI

physicsClient = pb.connect(render)#or p.DIRECT for non-graphical versio
pb.setRealTimeSimulation(False)
pb.setTimeStep(1/10)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pb.setGravity(0,0,-0)
print('start...')
planeId = pb.loadURDF("plane.urdf")
pb.resetBasePositionAndOrientation(planeId,(0,0,-20),pb.getQuaternionFromEuler((0,0,0,)))
import random
robj=[]
#set_random_objects()
keep_running = True

def MatDsym2pybullet(x,y,z,yaw,pitch,roll):
    M=np.zeros((4,4))
    M[:3,:3]=R.from_euler('ZYX',(yaw,pitch,roll),degrees=False).as_matrix()
    M[:,3]=[x,y,z,1]
    MI=np.linalg.inv(M)
    return M,MI


def translateQuatfromM(M):
    quat=R.from_matrix(M[:3,:3]).as_quat()
    t=M[:3,3]
    #print('==',t)
    return t.tolist(),quat.tolist()

def translateM(dx,dy,dz):
    T=np.eye(4)
    T[:,3]=(dx,dy,dz,1)
    return T



def getrov():

    shift = [0, -0.0, 0]
    meshScale = np.array([0.01, 0.01, 0.01])*0.2
    vfo=pb.getQuaternionFromEuler(np.deg2rad([-90, 0, 90]))
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
    currents_vector = [0,0.0,0]
    u_dot_f=lamb(curr_q,curr_u,*forces,*currents_vector).flatten()
    next_q=curr_q+curr_u*dt
    next_u=curr_u+u_dot_f*dt
    return next_q,next_u


def resize(img,factor):
    h,w = img.shape[:2]
    return cv2.resize(img,(int(w*factor),int(h*factor)))

def hsv_range_scale(rgbImg,depthImg):
    hsv = cv2.cvtColor(rgbImg,cv2.COLOR_BGR2HSV)

    #from matplotlib import pyplot as plt
    #plt.imshow(depthImg)
    #plt.show()
    #fff

    #hsv[:,:,0]=(np.clip(10*(1-depthImg),0.0,1)*hsv[:,:,0].astype('float')).astype('uint8')
    hsv[depthImg>0.94,0]=0
    #hsv[depthImg<0.94,0]=255
    #hsv[:,:,0]=255
    rgbImg = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    #rgbImg = cv2.blur(rgbImg,(3,3))
    return rgbImg

def main():
    cnt=0
    frame_cnt=0
    frame_ratio=1 # for 6 sim cycles 1 video frame
    resize_fact=0.5
    mono=False
    imgl = None
    curr_q = np.zeros(6)
    curr_u = np.zeros(6)
    current_command = np.zeros(8)
    fov=42

    if render==pb.GUI:
        boxId = getrov()
    
    scene = getscene()

    last_fps_print=time.time()

    while keep_running:
        tic_cycle = time.time()
        while len(zmq.select([zmq_sub],[],[],0.001)[0])>0:
            data = zmq_sub.recv_multipart()
            topic=data[0]
            if topic==zmq_topics.topic_thrusters_comand:
                _,current_command=pickle.loads(data[1])
                current_command=[i*1.3 for i in current_command]
            if topic==zmq_topics.topic_dvl_cmd:
                dvlSim.reset()

        next_q,next_u=get_next_state(curr_q,curr_u,current_command,dt,lamb)
        next_q,next_u=next_q.flatten(),next_u.flatten()
        curr_q,curr_u=next_q,next_u
        dvlSim.update(curr_q,curr_u)

        ps={}
        Mdsym, MdsymI=MatDsym2pybullet(*curr_q[:6])

        if cnt%frame_ratio==0:
            bl=0.0 if mono else 0.065
            CM = np.array(pb.computeViewMatrix((0,-0,0),(1,-0,0),(0,-0,-1))).reshape((4,4),order='F')
            #VML= CM @ MdsymI
            VML= CM @ translateM(0,bl/2,0) @ MdsymI

            #VM = pb.computeViewMatrixFromYawPitchRoll(pos,0.1,*pb.getEulerFromQuaternion(quat),2)
            #print('===',len(VML))
            PM = pb.computeProjectionMatrixFOV(fov=fov,aspect=1.0,nearVal=0.1,farVal=100)
            w = int(config.cam_res_rgbx*resize_fact)
            h = int(config.cam_res_rgby*resize_fact)
            width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=w, 
                height=h,
                viewMatrix=VML.flatten('F').tolist(),
                projectionMatrix=PM,renderer = pb.ER_BULLET_HARDWARE_OPENGL,
                lightColor=(0,0,1))
            #cv2.circle(rgbImg,(w//2,h//2),8,(225,255,0),2)

            ##experimental
            rgbImg = hsv_range_scale(rgbImg,depthImg)

            imgl=resize(rgbImg,1/resize_fact)#inly interested in rgb
            #imgl=resize(rgbImg[:,:,[2,1,0]],1/resize_fact)#inly interested in rgb
            #print('max...',depthImg.max(),depthImg.min())
            #hsv[:,:,0]=

            #second camera
            if not mono:
                #CM = np.array(pb.computeViewMatrix((0,bl,0),(1,bl,0),(0,0,-1))).reshape((4,4),order='F')
                VMR=CM @ translateM(0,-bl/2,0) @ MdsymI #left camera 0.2 for left 
                PM = pb.computeProjectionMatrixFOV(fov=fov,aspect=1.0,nearVal=0.1,farVal=1000)
                width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                    width=w, 
                    height=h,
                    viewMatrix=VMR.flatten('F').tolist(),
                    projectionMatrix=PM,renderer = pb.ER_BULLET_HARDWARE_OPENGL)
            
                rgbImg = hsv_range_scale(rgbImg,depthImg)
                #imgr=resize(rgbImg[:,:,[2,1,0]],1/resize_fact) #todo...
                imgr=resize(rgbImg,1/resize_fact) #todo...

            if cvshow:
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
            zmq_pub.send_multipart([zmq_topics.topic_stereo_camera_ts,pickle.dumps((frame_cnt,time.time()))]) #for sync
                
            #get depth image
            depthImg=depthImg[::4,::4]
            min_range=depthImg.min() 
            #import pdb;pdb.set_trace()

            img_show=(depthImg/10.0).clip(0,255).astype('uint8')
            depthImg[depthImg>5000]=np.nan
            max_range=np.nanmax(depthImg)
            #print('sonar::',min_range,max_range)
            pub_sonar.send_multipart([zmq_topics.topic_sonar,pickle.dumps([min_range,max_range])])

            if cvshow:
                cv2.imshow('depth',img_show)
                cv2.waitKey(1)
            frame_cnt+=1

            if render==pb.GUI:
                tr,qu = translateQuatfromM(Mdsym) 
                pb.resetBasePositionAndOrientation(boxId,tr,qu)
            ### test
            tic=time.time()
            imu={'ts':tic}
            imu['yaw'],imu['pitch'],imu['roll']=np.rad2deg(curr_q[3:])
            q3,q4,q5=curr_q[3:]
            u3,u4,u5=curr_u[3:]
            imu['rates']=(
                    -u3*sin(q4) + u5,
                    u3*sin(q5)*cos(q4) + u4*cos(q5),
                    u3*cos(q4)*cos(q5) - u4*sin(q5))
            pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])
            pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':curr_q[2]})])

        if cnt%5==0:
            tic=time.time()
            pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps({'ts':tic,'dvl_raw':dvlSim.dvl_pos_msg()})])
            pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps({'ts':tic,'dvl_raw':dvlSim.dvl_vel_msg()})])

        #time.sleep(0.100)
        time.sleep(0.01)

        if cnt%20==0 and imgl is not None:
            print('send...',cnt, imgl.shape, 'fps=%.1f'%(20/(time.time()-last_fps_print)))
            last_fps_print=time.time()
        cnt+=1


       
if __name__ == '__main__':
    main()
