#!/usr/bin/env python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#        1             2
#         X ----a---- X
#         |           |
#         |  \     /  |          4              3
#         |  5     6  |           X------------X |
#    ^x   |           |                          |   z
#    |    |    TOP    b                          |   ^
#y<--+    |           |                SIDE      h   |
#         |  /     \  |                          |
#         |  8     7  |             X--------X   |
#         X-----------X            7          8
#        4             3



import sys,time
import pybullet as pb
import pybullet_data
sys.path.append('..')
sys.path.append('../utils')
sys.path.append('unreal_proxy')
import zmq
import cv2
import numpy as np
import pickle
import zmq_wrapper as utils
import zmq_topics
import config
from dvl_sim import vel_msg
from numpy import sin,cos
from scipy.spatial.transform import Rotation as Rot

from mussels_scene import getscene

zmq_controller=utils.subscribe([zmq_topics.topic_dvl_cmd],zmq_topics.topic_controller_port)
zmq_sub=utils.subscribe([zmq_topics.topic_thrusters_comand],zmq_topics.topic_thrusters_comand_port)
zmq_pub=utils.publisher(zmq_topics.topic_camera_port)
pub_imu = utils.publisher(zmq_topics.topic_imu_port)
pub_depth = utils.publisher(zmq_topics.topic_depth_port)
pub_dvl = utils.publisher(zmq_topics.topic_dvl_port)

pub_sonar = utils.publisher(zmq_topics.topic_sonar_port)

#dvlSim=DVLSim()
#cvshow=False
current_command=[0 for _ in range(8)] # 8 thrusters
#pybullet init
render = pb.GUI if len(sys.argv)>1 and sys.argv[1]=='g' else pb.DIRECT # pb.GUI
#render = pb.GUI
physicsClient = pb.connect(render)#or p.DIRECT for non-graphical versio
#pb.setPhysicsEngineParameter(enableFileCaching=0)
pb.setRealTimeSimulation(False)
sim_step=1/100
pb.setTimeStep(sim_step)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pb.setGravity(0,0,-0.01)
print('start...')
planeId = pb.loadURDF("plane.urdf")
pb.resetBasePositionAndOrientation(planeId,(0,0,-20),pb.getQuaternionFromEuler((0,0,0,)))
import random
robj=[]
#set_random_objects()
keep_running = True


def __getrov():
    shift = [0, -0.0, 0]
    meshScale = np.array([0.01, 0.01, 0.01])*0.2
    vfo=pb.getQuaternionFromEuler(np.deg2rad([-90, 0, 90]))
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                        fileName="br1g.obj",
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

def getrov():
    boxId = pb.loadURDF('./brov.urdf', useMaximalCoordinates=False,flags=0)
    pb.changeDynamics(boxId,-1,linearDamping=5,angularDamping=5)
    return boxId



from scipy.interpolate import interp1d
pts_16v_t200 = [(1100,-4.07) , (1184, -2.94), (1308, -1.35) , (1472,0) , (1528,0), 
                (1624,0.87) , (1728,2.22), (1840,4.25), (1864,4.71), (1900,5.25)]
pwm,thrust = zip(*pts_16v_t200)
thrust=np.array(thrust)/0.101972 #kgf to newton
pwm_to_thrust=interp1d(pwm, thrust)
def scale_thrust(control):
    return np.array([pwm_to_thrust(c*400+1500)/10 for c in control])

def resize(img,factor):
    h,w = img.shape[:2]
    return cv2.resize(img,(int(w*factor),int(h*factor)))

def hsv_range_scale(rgbImg,depthImg):
    hsv = cv2.cvtColor(rgbImg,cv2.COLOR_BGR2HSV)
    hsv[depthImg>0.94,0]=0
    rgbImg = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    return rgbImg

#https://www.3dgep.com/understanding-the-view-matrix/#The_View_Matrix
def getCameraViewMat(bindex,link):
    pos,ori=pb.getLinkState(bindex,link,0,1)[:2]
    print(';;;',link,pos,Rot.from_quat(ori).as_euler('ZYX',degrees=True))
    Rmat = np.diag(np.ones(4))
    #r=Rot.from_rotvec(-np.pi/2 * np.array([0, 0, 1])).as_matrix()
    Rmat[:3,:3] = Rot.from_quat(ori).as_matrix().T# @ r
    Rmat[3,3]=1
    TR = np.diag(np.ones(4))
    TR[3,:3]=-np.array(pos)
    return Rmat@TR

grip=False

def main():
    cnt=0
    frame_cnt=0
    frame_ratio=20 # for 5 sim cycles 1 video frame
    resize_fact=0.5
    mono=False
    cvshow=False

    imgl = None
    current_command = np.zeros(8)
    fov=42

    sim_time=0
    #getscene()
    boxId = getrov()
    start_depth=0.5

    last_fps_print=time.time()


    sim_start=time.time()
    while keep_running:
        tic_cycle = time.time()
        while 1:
            socks=zmq.select([zmq_sub,zmq_controller],[],[],0.001)[0]
            if len(socks)==0:
                break
            for sock in socks:
                data = sock.recv_multipart()
                topic=data[0]
                if topic==zmq_topics.topic_thrusters_comand:
                    _,current_command=pickle.loads(data[1])
                if topic==zmq_topics.topic_dvl_cmd:
                    print('got dvl reset....')

        #pos,orient=pb.getBasePositionAndOrientation(boxId)
        #vpos,vspeed=pb.getBaseVelocity(boxId)
        pos_com,orient_com,_,_,_,_,vel_linear,vel_rot=pb.getLinkState(boxId,0,1,1)
        vel_rot_in_body = (Rot.from_quat(orient_com).as_matrix().T @ np.array(vel_rot).reshape(-1,1)).flatten()
        vel_linear_in_body = (Rot.from_quat(orient_com).as_matrix().T @ np.array(vel_linear).reshape(-1,1)).flatten()
        yaw,pitch,roll = Rot.from_quat(orient_com).as_euler('ZYX')


        #print('==0',pb.getLinkState(boxId,0,1,1)[0])
        #print('==2',pb.getLinkState(boxId,1,0,1)[0])
        #print('==3',pb.getLinkState(boxId,2,0,1)[0])
        if cnt%frame_ratio==0:
            PM = pb.computeProjectionMatrixFOV(fov=fov,aspect=1.0,nearVal=0.01,farVal=100)
            w = int(config.cam_res_rgbx*resize_fact)
            h = int(config.cam_res_rgby*resize_fact)

            VML = getCameraViewMat(boxId,1)
            width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=w, 
                height=h,
                viewMatrix=VML.flatten('C').tolist(),
                projectionMatrix=PM,renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                lightColor=(0,0,1))
            rgbImg = hsv_range_scale(rgbImg,depthImg)
            imgl=resize(rgbImg,1/resize_fact)#inly interested in rgb
            #second camera
            if not mono:
                VML = getCameraViewMat(boxId,2)
                width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                    width=w, 
                    height=h,
                    viewMatrix=VML.flatten('C').tolist(),
                    projectionMatrix=PM,renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                    lightColor=(0,0,1))
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

            if grip:# or render==pb.GUI:
                tr,qu = translateQuatfromM(Mdsym) 
                pb.resetBasePositionAndOrientation(boxId,tr,qu)
            ### test
            tic=time.time()
            imu={'ts':tic}
            #imu rotated by 180 arround x axis
            imu['yaw'],imu['pitch'],imu['roll']=np.deg2rad([-yaw,-pitch,roll])
            wx,wy,wz=vel_rot_in_body
            imu['rates']=[wx,-wy,-wz]            
            pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])

            pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':start_depth-pos_com[2]})])

            if 1:
                pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps(\
                        {'ts':tic,'dvl_raw':vel_msg(*vel_linear_in_body,sim_time)})])

        #time.sleep(0.100)

        if cnt%20==0 and imgl is not None:
            print('send...',cnt, imgl.shape, 'fps=%.1f'%(20/(time.time()-last_fps_print)))
            last_fps_print=time.time()
        
        s2=np.sqrt(2)
        a=0.3
        b=0.4
        h=0.2

#        1             2
#         X ----a---- X
#         |           |
#         |  \     /  |          4              3
#         |  5     6  |           X------------X |
#    ^x   |           |                          |   z
#    |    |    TOP    b                          |   ^
#y<--+    |           |                SIDE      h   |
#         |  /     \  |                          |
#         |  8     7  |             X--------X   |
#         X-----------X            7          8
#        4             3

        for i,(x,y) in enumerate([
                [1,1],
                [1,-1],
                [-1,-1],
                [-1,1]
                ]):
            pb.applyExternalForce(boxId,-1,[0,0,current_command[i]],[x*b/2,y*a/2,h/2],pb.LINK_FRAME)
            c=current_command[i+4]
            pb.applyExternalForce(boxId,-1, [x*s2*c,y*s2*c,0] ,[x,y,-h/2],pb.LINK_FRAME)
        cnt+=1
        pb.stepSimulation()
        sim_time=cnt*sim_step
        real_time = time.time()-sim_start
        rt_delta=sim_time-real_time
        if rt_delta>0:
            print('sleeping',rt_delta)
            time.sleep(rt_delta)


       
if __name__ == '__main__':
    main()
