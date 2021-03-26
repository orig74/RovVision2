import pygame,time,zmq,pickle,sys
#pygame.init() ### =100%cpu
sys.path.append('../')
sys.path.append('../utils')
import zmq_topics
import zmq_wrapper as utils

pygame.display.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
name = joystick.get_name()
#isxbox = 'Microsoft X-Box One pad' in name
isxbox = 'X-Box' in name
print("Joystick name: {}".format(name))
axes = joystick.get_numaxes()
print( "Number of axes: {}".format(axes))
n_buttons = joystick.get_numbuttons()

clock = pygame.time.Clock()

pub_sock=utils.publisher(zmq_topics.topic_joy_port)
done = False
cnt=0

start_time=time.time()
joy_log=open('joy.log','wb')
hat=[0,0]

def pub(topic,data):
    pub_sock.send_multipart([topic,data])
    pickle.dump([time.time()-start_time,topic,data],joy_log,-1)

while not done:
    # EVENT PROCESSING STEP
    cnt+=1
    if pygame.event.peek():
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


            if joystick.get_numhats()>0:
                hat = joystick.get_hat(0)
                if abs(hat[0])>0 or abs(hat[1])>0:
                    print('hat',hat)
                    pub(zmq_topics.topic_hat,pickle.dumps(hat))


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

    if cnt%10==0:
        print(cnt,'axes_vals=',','.join(['{:4.3f}'.format(i) for i in axes_vals]))
    #mixng axes
    
    pub(zmq_topics.topic_axes,pickle.dumps(axes_vals,-1))
        #print('{:> 5} P {:> 5.3f} S {:> 5.3f} V {:> 5.3f}'.format(cnt,port,starboard,vertical),end='\r')

    #pygame.time.wait(0)
    clock.tick(15)
pygame.quit()
