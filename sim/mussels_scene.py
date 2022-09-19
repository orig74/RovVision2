import pybullet as pb
import pybullet_data
import numpy as np
import cv2

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

def gen_rope(x,y):
    ret=[]
    meshScale = np.ones(3)*0.2
    for i in range(20):
        vfo=pb.getQuaternionFromEuler(np.deg2rad([90, 0, i*30]))
        visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                        fileName="pybullet_data/mussle_rope_part.obj",
                                        visualFramePosition=[0,0,0],
                                        visualFrameOrientation=vfo,
                                        meshScale=meshScale)#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
        ret.append(pb.createMultiBody(baseMass=0,
                          baseInertialFramePosition=[0, 0, 0],
                          baseVisualShapeIndex=visualShapeId,
                          basePosition=[x,y,i*1.2],
                          useMaximalCoordinates=True))

    return ret

def getscene():
    ret=[]
    meshScale = np.ones(3)*1
    vfo=pb.getQuaternionFromEuler(np.deg2rad([-90, 0, 0]))
    visualShapeId = pb.createVisualShape(shapeType=pb.GEOM_MESH,
                                        fileName="pybullet_data/seabed2.obj",
                                        visualFramePosition=[0,0,10],
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
    for j in range(2):
        for i in range(5):
            ret+=gen_rope(2+j*6,i*3)

    return ret



