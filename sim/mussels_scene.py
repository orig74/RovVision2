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
                    robj.append(obj)
                    print('---loading--',urdf,x,y)
                except:
                    print('fail to load object',x,y)
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

class MusseleRopesScene(object):
    def __init__(self,rows=2,cols=5):
        ret=[]
        meshScale = np.ones(3)*1
        #self.mussle_pos = np.array([1.8,0,0])
        self.mussle_pos = np.array([1.46,0,2])
        vfo=pb.getQuaternionFromEuler(np.deg2rad([90, 0, 0]))
        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                            fileName="pybullet_data/seabed2.obj",
                                            visualFramePosition=[0,0,-5],
                                            visualFrameOrientation=vfo,
                                            meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        ret.append(pb.createMultiBody(baseMass=0,
                              baseInertialFramePosition=[0, 0, 0],
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=[0,0,0],
                              useMaximalCoordinates=True))
     
        vfo=pb.getQuaternionFromEuler(np.deg2rad([0, 0, 90]))
        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                            fileName="pybullet_data/blueplane.obj",
                                            visualFramePosition=[80,0,0],
                                            visualFrameOrientation=vfo,
                                            meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        ret.append(pb.createMultiBody(baseMass=0,
                              baseInertialFramePosition=[0, 0, 0],
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=[0,0,0],
                              useMaximalCoordinates=True))

        vfo=pb.getQuaternionFromEuler(np.deg2rad([0, 0, 180]))
        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                            fileName="pybullet_data/mussels3Dmodel.obj",
                                            visualFramePosition=[1.5,0,0],
                                            visualFrameOrientation=vfo,
                                            meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        ret.append(pb.createMultiBody(baseMass=0,
                              baseInertialFramePosition=[0, 0, 0],
                              baseVisualShapeIndex=visualShapeId,
                              basePosition=[0,0,0],
                              useMaximalCoordinates=True))
        #for j in range(rows):
        #    for i in range(cols):
        #        ret+=gen_rope(2+j*3,i*1)

        self.mmm=pb.loadURDF("mussle.urdf",basePosition=self.mussle_pos)
        pb.changeDynamics(self.mmm,-1,linearDamping=1,angularDamping=1)
        #self.ccc=pb.createConstraint(self.mmm, -1, -1, -1, pb.JOINT_FIXED, [0, 0, 0], -self.mussle_pos, [0, 0, 0])
        self.ccc=pb.createConstraint(self.mmm, -1, -1, -1, pb.JOINT_FIXED, [0, 0, 0], [0,0,0] , self.mussle_pos)
        pb.changeConstraint(self.ccc,maxForce=0.3)

    def update(self):
        #small force to emulate gravity
        pb.applyExternalForce(self.mmm,-1,[0,0,-0.001],[0,0,0],pb.WORLD_FRAME)
        if self.ccc is not None:
            pos,rot=pb.getBasePositionAndOrientation(self.mmm)
            if np.linalg.norm(pos-self.mussle_pos)>0.1:
                pb.removeConstraint(self.ccc)
                self.ccc=None


