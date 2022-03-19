# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import zmq,pickle,time,os
import struct
import ue4_zmq_topics as config
from Wrappers import phandlers as ph
import numpy as np
import cv2
print('*'*500)

drone_texture_names=['/Game/mycontent/bluerovstereo2','/Game/mycontent/bluerovstereo1' ]
#drone_texture_names=['bluerovstereo1','bluerovstereo2' ]

drone_textures_depth_names=['/Game/mycontent/TextureRenderTarget2D_depth']
#drone_textures_depth_names=['TextureRenderTarget2D_depth']
#drone_textures_depth_names=[]
#needed actors
#drone_actors_names=['BlueRov1']
drone_actors_names=['StaticMeshActor_1']
context = zmq.Context()

show_cv=False
pub_cv=True
fps = 10

drone_subs=[]

if 'CAMERA_RIG_PITCH' in os.environ and os.environ['CAMERA_RIG_PITCH']:
    pitch=float(os.environ['CAMERA_RIG_PITCH'])
else:
    pitch=0

if 'INITIAL_DRONE_POS' in os.environ and os.environ['INITIAL_DRONE_POS']:
    initial_pos = list(map(float,os.environ['INITIAL_DRONE_POS'].split(',')))
else:
    initial_pos = None

############### need to be updated for mu;tiple drones
socket_sub = context.socket(zmq.SUB)
drone_ip,port=config.zmq_pub_drone_fdm
#drone_ip='172.17.0.%d'%(ind+2) #172.17.0.1 for the docker host and 172.17.0.2 for first drone etc...
addr='tcp://%s:%d'%(drone_ip,port)
print("connecting to",addr)
socket_sub.connect(addr)
socket_sub.setsockopt(zmq.SUBSCRIBE,config.topic_sitl_position_report)
drone_subs.append(socket_sub)

socket_pub = context.socket(zmq.PUB)
socket_pub.bind("tcp://%s:%d" % config.zmq_pub_unreal_proxy )


start=time.time()


def main_loop(gworld):
    frame_cnt = 0
    print('-- actors list --',gworld)
    i=0
    for p in ph.GetActorsNames(gworld,1024*1000):
        p_wc = p[1::2]
        a = ph.FindActorByName(gworld,p_wc)
        print(i,p,ph.GetActorLocation(a) if a else a)#,'pos',ph.GetActorLocation(a))
        if 0:
            import pdb;pdb.set_trace()
        i+=1
        #print(p,'pos',ph.GetActorLocation(a))
    print('-- textures --')
    print('-- starting rovvision simulation --')
    drone_textures=[]
    for tn in drone_texture_names:
        drone_textures.append(ph.GetTextureByName(tn))
    drone_textures_depth=[]
    for tn in drone_textures_depth_names:
        drone_textures_depth.append(ph.GetTextureByName(tn))

    if not all(drone_textures):
        print("Error, Could not find all textures", drone_textures)
        while 1:
            yield
    drone_actors=[]
    print('---looking for actors---')
    for drn in drone_actors_names:
        drone_actors.append(ph.FindActorByName(gworld,drn))
    print('---looking for actors 2---',drone_actors)

    if not all(drone_actors):
        print("Error, Could not find all drone actors")
        while 1:
            yield
    print('---debug 1---')
    
    #change cameras angle
    #for cam_name in ['SceneCaptureBROV1left','SceneCaptureBROV1right']: 
    for cam_name in ['SceneCapture2D_2','SceneCapture2D_3','SceneCapture2D_4']: 
        #ca=ph.FindActorByName(gworld,'SceneCaptureBROV1left')
        print('---debug 1.1---')
        ca=ph.FindActorByName(gworld,cam_name)
        print('---debug 1.2---',ca)
        ph.SetActorRotation(ca,(1,-pitch,89))
        #ph.SetActorRotation(ca,(0,-pitch,0))
        print('---debug 1.3---')
        yield
        #ph.SetActorRotation(caml,(1,1,89)) #facing down
        print('camera ' +cam_name + ' rotation ---',ph.GetActorRotation(ca))
        print('---debug 1.4---')

    #print('--- ',caml)
    print('---debug 2---')



    for _ in range(10): #need to send it a few time don't know why.
        print('sending state main loop')
        socket_pub.send_multipart([config.topic_unreal_state,b'main_loop'])
        yield
    print('initial_pos is ',initial_pos)
    drone_start_positions=[np.array(ph.GetActorLocation(drone_actor) if initial_pos is None else initial_pos) for drone_actor in drone_actors]
    positions=[None for _ in range(config.n_drones)]
    tic = time.time()
    while 1:
        while time.time()-tic < 1.0/fps:
            time.sleep(0.001)
            yield
        tic=time.time()

        for drone_index in range(config.n_drones):
            socket_sub=drone_subs[drone_index]
            drone_actor=drone_actors[drone_index]
            while len(zmq.select([socket_sub],[],[],0)[0])>0:
                topic, msg = socket_sub.recv_multipart()
                positions[drone_index]=pickle.loads(msg)
                #print('-----',positions[drone_index])

            position=positions[drone_index]
            if position is not None:
                new_pos=drone_start_positions[drone_index]+np.array([position['posy'],position['posx'],-position['posz']])*100 #turn to cm
                #print('-----',drone_index,position)
                ph.SetActorLocation(drone_actor,new_pos)
                ph.SetActorRotation(drone_actor,(position['roll']+90,position['pitch'],position['yaw']))
                positions[drone_index]=None
        #yield
        topics=[]
        imgs=[]

        if 1:
        #img=ph.GetTextureData(drone_textures[0])
            imgr=ph.GetTextureData(drone_textures[0])
            imgl=ph.GetTextureData(drone_textures[1])
        #topics.append(config.topic_unreal_drone_rgb_camera%0)

            img_depth=ph.GetTextureData32f(drone_textures_depth[0],channels=[3],verbose=1) #depth data will be in red componnent
            #img_depth=ph.GetTextureData(drone_textures_depth[0],channels=[2]) #depth data will be in red componnent
            socket_pub.send_multipart([config.topic_unreal_stereo_camera%0,pickle.dumps((frame_cnt,imgl.shape)),imgl.tostring(),imgr.tostring()])
            socket_pub.send_multipart([config.topic_unreal_depth%0,pickle.dumps((frame_cnt,img_depth.shape)),img_depth.tostring()])

            if show_cv:
                cv2.imshow('drone camera %d'%drone_index,imgl)
                cv2.waitKey(1)

            frame_cnt += 1

def kill():
    print('done!')
    socket_pub.send_multipart([config.topic_unreal_state,b'kill'])
    if show_cv:
        cv2.destroyAllWindows()
        for _ in range(10):
            cv2.waitKey(10)



if __name__=="__main__":
    while 1:
        for drone_index in range(config.n_drones):
            socket_sub=drone_subs[drone_index]
            while len(zmq.select([socket_sub],[],[],0)[0])>0:
                topic, msg = socket_sub.recv_multipart()
                print("got ",topic)
