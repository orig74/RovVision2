import pygame,time,zmq,pickle,sys
#pygame.init() ### =100%cpu
sys.path.append('../')
sys.path.append('../utils')
import zmq_topics
import zmq_wrapper as utils

pygame.display.init()
pygame.joystick.init()
#pygame.init()
try:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    name = joystick.get_name()
    #isxbox = 'Microsoft X-Box One pad' in name
    isxbox = 'X-Box' in name
    print("Joystick name: {}".format(name))
    axes = joystick.get_numaxes()
    print( "Number of axes: {}".format(axes))
    n_buttons = joystick.get_numbuttons()
except pygame.error:
    print("failed to detect joystick\nlisten only to buttons")
    screen = pygame.display.set_mode((300, 300))
    joystick=None
    axes=None

clock = pygame.time.Clock()

pub_sock=utils.publisher(zmq_topics.topic_joy_port)
done = False
cnt=0
axes_vals=None

start_time=time.time()
joy_log=open('joy.log','wb')
hat=[0,0]

def pub(topic,data):
    pub_sock.send_multipart([topic,data])
    pickle.dump([time.time()-start_time,topic,data],joy_log,-1)

while not done:
    # EVENT PROCESSING STEP
    cnt+=1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
 
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
            buttons = [joystick.get_button(i) for i in range(n_buttons)]
            print('pub buttons=',buttons)
            pub(zmq_topics.topic_button,pickle.dumps(buttons))
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
            buttons = [joystick.get_button(i) for i in range(n_buttons)]
            pub(zmq_topics.topic_button,pickle.dumps(buttons))

        if event.type == pygame.KEYDOWN:
            axes_vals = [0 for i in range(10)]
            buttons = [0 for i in range(10)]
            #https://www.pygame.org/docs/ref/key.html
            if event.mod == pygame.KMOD_NONE:
                lshift=False 
            else:
                if event.mod & pygame.KMOD_LSHIFT:
                    lshift=True
                    print('Left shift was in a pressed state when this event ')
            keys=pygame.key.get_pressed()
            mag=0.3
            if keys[pygame.K_LEFT]:
                print('left')
                axes_vals[3]=-mag
            if keys[pygame.K_RIGHT]:
                print('right')
                axes_vals[3]=mag
            if keys[pygame.K_UP]:
                print('fw')
                axes_vals[4]=mag
            if keys[pygame.K_DOWN]:
                print('bk')
                axes_vals[4]=-mag
            if keys[pygame.K_0]:
                print('arm/disarm')
            if keys[pygame.K_1]:
                print('depth hold')
            if keys[pygame.K_2]:
                print('att hold')
            if keys[pygame.K_a]:
                print('float')
            if keys[pygame.K_z]:
                print('dive')
            if keys[pygame.K_x]:
                print('yaw left')
            if keys[pygame.K_c]:
                print('yaw right')


        if joystick and joystick.get_numhats()>0:
            hat = joystick.get_hat(0)
            if abs(hat[0])>0 or abs(hat[1])>0:
                print('hat',hat)
                pub(zmq_topics.topic_hat,pickle.dumps(hat))
        

        if joystick:
            axes_vals = []
            for i in range(axes):
                axis = joystick.get_axis(i)
                dead_band = 0.02
                expo  = 0.6
                if isxbox:
                    if abs(axis)<dead_band:
                        axis=0.0
                    elif axis > 0:
                        axis -= dead_band
                    elif axis < 0:
                        axis += dead_band
                    #Calc expo
                    axis *= abs(axis) ** expo 
                axes_vals.append(axis)
            if axes==6: #add hat to axes to maintain compatibility
                axes_vals+=[float(hat[0]),float(hat[1])]

        if cnt%10==0 and axes_vals:
            print('axes_vals=',','.join(['{:4.3f}'.format(i) for i in axes_vals]))
        #mixng axes
        
        pub(zmq_topics.topic_axes,pickle.dumps(axes_vals,-1))
        #print('{:> 5} P {:> 5.3f} S {:> 5.3f} V {:> 5.3f}'.format(cnt,port,starboard,vertical),end='\r')

    #pygame.time.wait(0)
    clock.tick(30)
pygame.quit()
