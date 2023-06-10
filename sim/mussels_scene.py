import pybullet as pb
import pybullet_data
import numpy as np
import cv2
from numpy import random
random.seed(0)

def set_random_objects():
    random.seed(0)
    for row in [0,1]:
        for col in [0,1,2]:
            for _ in range(200):
                urdf = "random_urdfs/{0:03d}/{0:03d}.urdf".format(random.randint(1+row*50*3+col*50,50+row*50*3+col*50)) 
                x = col*1.5 + (random.random()-0.5)*.3
                y = row*1.5 + (random.random()-0.5)*.3
                z = (random.random())*-10
                try:
                    obj = pb.loadURDF(urdf,basePosition=[x,y,z],globalScaling=3,baseOrientation=[random.random() for _ in range(4)])
                    obj.append(obj)
                    print('---loading--',urdf,x,y)
                except Exception as E:
                    print('fail to load object',x,y,E)
                #pb.resetBasePositionAndOrientation(obj,(x,y,z),pb.getQuaternionFromEuler((0,0,0,)))

def rand(x,scl=0.03):
    return random.normal(x,scl)

def gen_rope(x,y):
    ret=[]
    dia=1.75*2
    scl=1/dia*0.3
    meshScale = np.array([scl,0.2,scl]) #30 cm width
    for i in range(20):
        for _ in range(2):
            vfo=pb.getQuaternionFromEuler(np.deg2rad([90+rand(0,2), rand(0,2), i*30]))
            visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                            fileName="pybullet_data/mussle_rope_part.obj",
                                            visualFramePosition=[0,0,0],
                                            visualFrameOrientation=vfo,
                                            meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
            ret.append(pb.createMultiBody(baseMass=0,
                              baseInertialFramePosition=[0, 0, 0],
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=[rand(x),rand(y),i*1.2],
                              useMaximalCoordinates=True))

    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_SPHERE,
                                    visualFramePosition=[0,0,0],
                                    rgbaColor=[0,0,1,1],
                                    radius=0.013)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    collisionShapeId = pb.createCollisionShape(shapeType=pb.GEOM_SPHERE,
                                    radius=0.013)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    if 0:
        ret.append(pb.createMultiBody(baseMass=0.05,
                          baseInertialFramePosition=[0, 0, 0],
                          baseVisualShapeIndex=visualShapeId,
                          baseCollisionShapeIndex=collisionShapeId,
                          basePosition=[1.8,0,0],
                          useMaximalCoordinates=True))

        #pb.createConstraint(ret[-1],-1,-1,-1,pb.JOINT_POINT2POINT,[0,0,1],
        #        [0,0,0],[0,0,0],[0,0,0,1],[0,0,0,1])
        ret.append(pb.createConstraint(ret[-1], -1, -1, -1, pb.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0]))


    return ret

class Mussle(object):
    def __init__(self,mussle_pos,ref_body):
        self.mussle_pos=mussle_pos
        self.ref_body=ref_body
        self.mmm=pb.loadURDF("mussle.urdf",basePosition=self.mussle_pos)
        pb.changeDynamics(self.mmm,-1,linearDamping=1,angularDamping=15)
        #self.ccc=pb.createConstraint(self.mmm, -1, -1, -1, pb.JOINT_FIXED, [0, 0, 0], -self.mussle_pos, [0, 0, 0])
        self.ccc=pb.createConstraint(self.mmm, -1, ref_body, -1, pb.JOINT_FIXED, [0, 0, 0], [0,0,0] , self.mussle_pos)

        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_SPHERE,
                                        rgbaColor=[0,0,1,0.2],
                                        radius=0.1)
        self.mussle_clue=pb.createMultiBody(baseMass=0,
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=self.mussle_pos,
                              useMaximalCoordinates=True)
        self.cnt=0

    def update(self):
        pb.applyExternalForce(self.mmm,-1,[0,0,-0.001],[0,0,0],pb.WORLD_FRAME)
        if self.cnt==100:
            pb.changeConstraint(self.ccc,maxForce=0.3)
        if self.ccc is not None:
            pos,rot=pb.getBasePositionAndOrientation(self.mmm)
            pb.resetBasePositionAndOrientation(self.mussle_clue,pos,rot)
            rpos,rrot=pb.getBasePositionAndOrientation(self.ref_body)
            if self.cnt>100 and np.linalg.norm(pos-self.mussle_pos-rpos)>0.1:
                #pb.removeConstraint(self.ccc) #couse segmentation fault
                pb.changeConstraint(self.ccc,maxForce=0.0)
                self.ccc=None
                pb.removeBody(self.mussle_clue)
        self.cnt+=1



class MusseleRopesScene(object):
    def __init__(self,rows=2,cols=6):
        meshScale = np.ones(3)*1
        self.seabed_pos=[0,0,-2]
        #self.mussle_pos = np.array([1.8,0,0])
        vfo=pb.getQuaternionFromEuler(np.deg2rad([90, 0, 0]))
        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                            fileName="pybullet_data/seabed2.obj",
                                            visualFramePosition=self.seabed_pos,
                                            visualFrameOrientation=vfo,
                                            meshScale=[2,2,2])#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        sb=pb.createMultiBody(baseMass=0,
                              baseInertialFramePosition=[0, 0, 0],
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=[0,0,0],
                              useMaximalCoordinates=True)
        self.sb=sb
     
        vfo=pb.getQuaternionFromEuler(np.deg2rad([0, 0, 180]))
        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                            fileName="pybullet_data/mussels3Dmodel.obj",
                                            visualFramePosition=[1.5,0,0],
                                            visualFrameOrientation=vfo,
                                            meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        self.vlines=[]
        self.vlines_const=[]
        for i in range(cols):
            base_position = [0,1.4*i,0]
            rr=pb.createMultiBody(baseMass=0.1,
                                  #baseInertialFramePosition=base_position,
                                  baseVisualShapeIndex=visualShapeId,
                                  basePosition=base_position,
                                  useMaximalCoordinates=True)
            self.vlines.append(rr)
            print('--1')
            #if i==0:
            #    base_position = [0,1.4*i,2]
            cc=pb.createConstraint(rr, -1, sb, -1, pb.JOINT_FIXED, [0, 0, 0], [0,0,0],base_position)
            self.vlines_const.append(cc)
            print('--2')
            #for j in range(rows):
        #    for i in range(cols):
        #        ret+=gen_rope(2+j*3,i*1)
        self.mmm=[]
        for mussle_pos in [[1.46,0,2],[1.46,0.1,2.1],[1.46,-0.1,1.9],[1.46,-0.07,2.02]]:
            self.mmm.append(Mussle(np.array(mussle_pos),self.vlines[0]))

        self.cnt=0

    def update(self):
        for m in self.mmm:
            m.update()

        if self.cnt>300:
            #fd=1 if self.cnt%1000<500 else -1
            T=2000
            fd=np.sin((self.cnt%T-(T//2))/T*np.pi*2)
            rpos,rrot=pb.getBasePositionAndOrientation(self.vlines[0])
            rpos=np.array(rpos)
            rpos+=[0,0,fd*0.0002]
            pb.removeConstraint(self.vlines_const[0])
            self.vlines_const[0]=pb.createConstraint(self.vlines[0], -1, self.sb, -1, pb.JOINT_FIXED, [0, 0, 0], [0,0,0],rpos)
        self.cnt+=1


