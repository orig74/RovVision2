import numpy as np
class DVLSim(object):
    def __init__(self):
        self.dvl_angle_offsets=np.zeros(3)
        self.dvl_offset=np.zeros(3)

    def update(self,curr_q,curr_u):
        self.curr_q,self.curr_u=curr_q,curr_u

    def dvl_vel_msg(self):
        curr_u=self.curr_u
        vx,vy,vz = curr_u[:3]
        #simulate dvl messgaes
        yaw_off=-self.dvl_angle_offsets[0]#-np.pi/2
        c,s = np.cos(yaw_off),np.sin(yaw_off)
        vx,vy = vx*c-vy*s,vx*s+vy*c
        vel_msg='wrz,{},{},{},y,1.99,0.006,3.65e-05;3.39e-06;7.22e-06;3.39e-06;2.46e-06;-8.5608e-07;7.223e-06;-8.560e-07;3.2363e-06,1550139816188624,1550139816447957,188.80,0*XX\r\n'.format(vx,vy,vz).encode()
        return vel_msg

    def dvl_pos_msg(self):
        curr_q=self.curr_q
        x,y,z=curr_q[:3]-self.dvl_offset
        yaw_off=-self.dvl_angle_offsets[0]#-np.pi/2
        c,s = np.cos(yaw_off),np.sin(yaw_off)
        x,y = x*c-y*s,x*s+y*c

        pos_msg='wrp,1550139816.178,{},{},{},{},2.5,-3.7,-62.5,0*XX\r\n'.\
                format(x,y,z,100).encode()
        return pos_msg

    def reset(self): #command b'wcr\n'
        print('got dvl reset')
        self.dvl_offset=self.curr_q[:3]
        self.dvl_angle_offsets=self.curr_q[3:]





