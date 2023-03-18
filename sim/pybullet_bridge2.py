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
from dvl_sim import vel_msg,pos_msg
from numpy import sin,cos
from scipy.spatial.transform import Rotation as Rot

from mussels_scene import MusseleRopesScene as getscene

zmq_controller=utils.subscribe([zmq_topics.topic_dvl_cmd],zmq_topics.topic_controller_port)
zmq_sub=utils.subscribe([zmq_topics.topic_thrusters_comand,zmq_topics.topic_gripper_cmd],zmq_topics.topic_thrusters_comand_port)
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
pb.configureDebugVisualizer(0,0,[0,0,100],rgbBackground=[43/255+0.26,84/255+0.26,132/255+0.26])
#pb.setPhysicsEngineParameter(enableFileCaching=0)
pb.setRealTimeSimulation(False)
sim_step=1/200
pb.setTimeStep(sim_step)
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pb.setGravity(0,0,0)
print('start...')
#planeId = pb.loadURDF("plane.urdf")
#pb.resetBasePositionAndOrientation(planeId,(0,0,-20),pb.getQuaternionFromEuler((0,0,0,)))
import random
robj=[]
#set_random_objects()
keep_running = True

def getrov():
    rov_base_ori = Rot.from_euler('ZYX',[0,0,0],degrees=True)
    rov_base_pos = [0,0,2]
    boxId = pb.loadURDF('./brov.urdf', 
            basePosition = rov_base_pos,
            baseOrientation = rov_base_ori.as_quat(),useMaximalCoordinates=False,flags=pb.URDF_MAINTAIN_LINK_ORDER | pb.URDF_USE_INERTIA_FROM_FILE)
    pb.changeDynamics(boxId,-1,linearDamping=5,angularDamping=5)
    _link_name_to_index = {pb.getBodyInfo(boxId)[0].decode('UTF-8'):-1,}
    _joint_name_to_index = {}
    for _id in range(pb.getNumJoints(boxId)):
        _name = pb.getJointInfo(boxId, _id)[12].decode('UTF-8')
        _link_name_to_index[_name] = _id
        _name = pb.getJointInfo(boxId, _id)[1].decode('UTF-8')
        _joint_name_to_index[_name] = _id


    #import ipdb;ipdb.set_trace()
    return boxId,_link_name_to_index,_joint_name_to_index


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
    hsv[depthImg>10.1,0]=0
    rgbImg = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
    return rgbImg
#def hsv_range_scale(rgbImg,depthImg):
#    return rgbImg.copy()


#https://www.3dgep.com/understanding-the-view-matrix/#The_View_Matrix
def getCameraViewMat(bindex,link):
    pos,ori=pb.getLinkState(bindex,link,0,1)[0:2]
    #print(';;;',link,pos,Rot.from_quat(ori).as_euler('ZYX',degrees=True))
    R=Rot.from_quat(ori).as_matrix()
    up=(R @ np.array([[0,0,1]]).T).flatten() 
    eye=pos
    target = (R @ np.array([[10,0,0]]).T).flatten()
    Rmat = pb.computeViewMatrix(eye,target,up)
    Rmat=np.array(Rmat).reshape((4,-1))
    return Rmat

grip=False
fps=10
imu_fps=100
dvl_pos_fps=5
dvl_vel_fps=10
print_fps=0.5
def main():
    def ratio(x):
        return cnt%int(1/sim_step/x)==0
    cnt=0
    frame_cnt=0
    

    resize_fact=0.5
    mono=False
    cvshow=False

    imgl = None
    current_command = np.zeros(8)
    fov=42

    sim_time=0
    scene=getscene(1,1)
    boxId,link_name_to_index,joint_name_to_index = getrov()
    start_depth=0.5

    last_fps_print=time.time()
    gripper_cmd=0

    sim_start=time.time()
    while keep_running:
        tic_cycle = time.time()
        if 1:
            socks=zmq.select([zmq_sub,zmq_controller],[],[],0.001)[0]
            #if len(socks)==0:
            #    break
            for sock in socks:
                data = sock.recv_multipart()
                topic=data[0]
                if topic==zmq_topics.topic_thrusters_comand:
                    _,current_command=pickle.loads(data[1])
                if topic==zmq_topics.topic_dvl_cmd:
                    print('got dvl reset....')
                if topic==zmq_topics.topic_gripper_cmd:
                    gripper_cmd=pickle.loads(data[1])
                    print('gripper command',gripper_cmd)
 


        scene.update()
        #pos,orient=pb.getBasePositionAndOrientation(boxId)
        #vpos,vspeed=pb.getBaseVelocity(boxId)
        pos_com,orient_com,_,_,_,_,vel_linear,vel_rot=pb.getLinkState(boxId,0,1,1)
        vel_rot_in_body = (Rot.from_quat(orient_com).as_matrix().T @ np.array(vel_rot).reshape(-1,1)).flatten()
        vel_linear_in_body = (Rot.from_quat(orient_com).as_matrix().T @ np.array(vel_linear).reshape(-1,1)).flatten()
        yaw,pitch,roll = Rot.from_quat(orient_com).as_euler('ZYX')


        #print('==0',pb.getLinkState(boxId,0,1,1)[0])
        #print('==2',pb.getLinkState(boxId,1,0,1)[0])
        #print('==3',pb.getLinkState(boxId,2,0,1)[0])
        if ratio(fps):
            PM = pb.computeProjectionMatrixFOV(fov=fov,aspect=1.0,nearVal=0.01,farVal=100)
            w = int(config.cam_res_rgbx*resize_fact)
            h = int(config.cam_res_rgby*resize_fact)

            VML = getCameraViewMat(boxId,0)
            width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                width=w, 
                height=h,
                viewMatrix=VML.flatten('C').tolist(),
                lightColor=(0,0,1),
                projectionMatrix=PM,renderer=pb.ER_BULLET_HARDWARE_OPENGL)
            rgbImg = hsv_range_scale(rgbImg,depthImg)
            imgl=resize(rgbImg,1/resize_fact)#inly interested in rgb
            #second camera
            if not mono:
                VML = getCameraViewMat(boxId,1)
                width, height, rgbImg, depthImg, segImg = pb.getCameraImage(
                    width=w, 
                    height=h,
                    viewMatrix=VML.flatten('C').tolist(),
                    lightColor=(0,0,1),
                    projectionMatrix=PM,renderer=pb.ER_BULLET_HARDWARE_OPENGL)
                rgbImg = hsv_range_scale(rgbImg,depthImg)
                imgr=resize(rgbImg,1/resize_fact) #todo...

            if cvshow:
                imgls = imgl[::2,::2]
                imgrs = imgr[::2,::2]
                cv2.imshow('l',imgls)
                cv2.imshow('r',imgrs)
                cv2.waitKey(1)
            
            if mono:
                zmq_pub.send_multipart([zmq_topics.topic_stereo_camera,pickle.dumps([frame_cnt,imgl.shape]),imgl.tostring()],copy=False)
            else:
                zmq_pub.send_multipart([zmq_topics.topic_stereo_camera,pickle.dumps([frame_cnt,imgl.shape]),imgl.tostring(),imgr.tostring()],copy=False)
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

        #if grip:# or render==pb.GUI:
        #    tr,qu = translateQuatfromM(Mdsym) 
        #    pb.resetBasePositionAndOrientation(boxId,tr,qu)
        #### test
        tic=time.time()
        imu={'ts':tic}
        #imu rotated by 180 arround x axis
        if ratio(imu_fps):
            imu['yaw'],imu['pitch'],imu['roll']=np.rad2deg([-yaw,-pitch,roll])
            wx,wy,wz=vel_rot_in_body
            imu['rates']=[wx,-wy,-wz]            
            pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])

            pub_depth.send_multipart([zmq_topics.topic_depth,pickle.dumps({'ts':tic,'depth':start_depth-pos_com[2]})])
            #print('send imu',imu)

        if ratio(dvl_vel_fps):
            pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps(
                    {'ts':tic,'dvl_raw':vel_msg(*vel_linear_in_body,sim_time)})])
        if ratio(dvl_pos_fps):
            pub_dvl.send_multipart([zmq_topics.topic_dvl_raw,pickle.dumps(
                    {'ts':tic,'dvl_raw':pos_msg(pos_com[0],-pos_com[1],-pos_com[2],-yaw)})])

        #time.sleep(0.100)

        if ratio(print_fps) and imgl is not None:
            print('send...',cnt, imgl.shape, 'step/sec=%.1f'%(20/(time.time()-last_fps_print)))
            last_fps_print=time.time()
        
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
        s2=np.sin(45)
        a=0.3
        b=0.4
        h=0.2
        a2=a/2
        b2=b/2
        h2=h/2
        cc=current_command
        pb.applyExternalForce(boxId,-1,[0,0,-cc[0]],[b2,a2,h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[0,0,-cc[1]],[b2,-a2,h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[0,0,-cc[2]],[-b2,-a2,h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[0,0,-cc[3]],[-b2,a2,h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[s2*cc[4],-s2*cc[4],0],[b2,a2,-h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[s2*cc[5],s2*cc[5],0],[b2,-a2,-h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[s2*cc[6],-s2*cc[6],0],[-b2,-a2,-h2],pb.LINK_FRAME)
        pb.applyExternalForce(boxId,-1,[s2*cc[7],s2*cc[7],0],[-b2,a2,-h2],pb.LINK_FRAME)

        boyency_force = (Rot.from_quat(orient_com).as_matrix().T @ np.array([0,0,100]).reshape(-1,1)).flatten()
        pb.applyExternalForce(boxId,link_name_to_index['COB'],boyency_force,[0,0,0],pb.LINK_FRAME)
        gravity_force = (Rot.from_quat(orient_com).as_matrix().T @ np.array([0,0,-100]).reshape(-1,1)).flatten()
        pb.applyExternalForce(boxId,link_name_to_index['COM'],gravity_force,[0,0,0],pb.LINK_FRAME)
        #boyency force

        maxForce = 500
        pb.setJointMotorControl2(bodyUniqueId=boxId,
        jointIndex=joint_name_to_index['gripper_right'],
        controlMode=pb.POSITION_CONTROL,
        targetPosition = gripper_cmd,
        force = maxForce)
        pb.setJointMotorControl2(bodyUniqueId=boxId,
        jointIndex=joint_name_to_index['gripper_left'],
        controlMode=pb.POSITION_CONTROL,
        targetPosition = gripper_cmd,
        force = maxForce)
        
       
        pb.stepSimulation()
        sim_time=cnt*sim_step
        real_time = time.time()-sim_start
        rt_delta=sim_time-real_time
        if rt_delta>0:
            #print('sleeping',rt_delta)
            time.sleep(rt_delta)
        else:
            time.sleep(0.001)
        cnt+=1
        cycle_time=time.time()-tic_cycle
        if cycle_time>0.1:
            print(cnt,'very slow cycle_time',cycle_time)

       
if __name__ == '__main__':
    main()
