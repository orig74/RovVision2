import numpy as np
from scipy.spatial.transform import Rotation as Rot

def vel_msg(vx,vy,vz,ts_sec):
    return \
    'wrz,{},{},{},y,1.99,0.006,3.65e-05;7.22e-06;7.22e-06;3.39e-06;2.46e-06;-8.5608e-07;7.223e-06;-8.560e-07;3.2363e-06,{},1550139816447957,188.80,0*XX\r\n'.format(vx,vy,vz,int(ts_sec*1e6)).encode()

def pos_msg(x,y,z,yaw_deg):
    return  'wrp,1550139816.178,{},{},{},{},2.5,-3.7,{},0*XX\r\n'.\
        format(x,y,z,100,yaw_deg%360).encode()


class DVLSim(object):
    def __init__(self):
        self.dvl_angle_offsets=np.zeros(3)
        self.dvl_offset=np.zeros(3)
        self.dvl_yaw_drift=0
        self.dvl_xy_drift=np.array([0.00001,0.00001])
        self.cnt=0

    def update(self,curr_q,curr_u,t):
        self.curr_q,self.curr_u=curr_q,curr_u
        self.cnt+=1
        self.t=t

    def dvl_vel_msg(self):
        curr_u=self.curr_u
        vx,vy,vz = curr_u[:3]
        _y,_p,_r=self.curr_q[3:]
        R=Rot.from_euler('xyz',[_r,_p,_y]).as_matrix().T
        #simulate dvl messgaes
        yaw_off=-_y#-np.pi/2#-self.dvl_angle_offsets[0]#-np.pi/2
        c,s = np.cos(yaw_off),np.sin(yaw_off)
        vx,vy = vx*c-vy*s,vx*s+vy*c
        #print(f'1- vx={vx*c-vy*s},vy={vx*s+vy*c}')
        #vx,vy,vz = (R.T @ np.array([[vx,vy,vz]]).T).flatten() #velocity vector in body frame as defined 
        print(f'vx={vx},vy={vy},vz={vz}')
        vel_msg='wrz,{},{},{},y,1.99,0.006,3.65e-05;7.22e-06;7.22e-06;3.39e-06;2.46e-06;-8.5608e-07;7.223e-06;-8.560e-07;3.2363e-06,{},1550139816447957,188.80,0*XX\r\n'.format(vx,vy,vz,int(self.t*1e6)).encode()
        return vel_msg

    def dvl_pos_msg(self):
        curr_q=self.curr_q
        x,y,z=curr_q[:3]-self.dvl_offset
        _y,_p,_r=self.curr_q[3:]
        yaw_off=-self.dvl_angle_offsets[0]#-np.pi/2
        c,s = np.cos(yaw_off),np.sin(yaw_off)
        x,y = x*c-y*s,x*s+y*c
        x_,y_=self.dvl_xy_drift*self.cnt
        x+=x_
        y+=y_

        pos_msg='wrp,1550139816.178,{},{},{},{},2.5,-3.7,{},0*XX\r\n'.\
                format(x,y,z,100,np.rad2deg(yaw_off+_y)%360).encode()
        return pos_msg

    def reset(self): #command b'wcr\n'
        print('got dvl reset')
        self.dvl_offset=self.curr_q[:3]
        self.dvl_angle_offsets=self.curr_q[3:]
        self.dvl_yaw_drift=0
        self.cnt=0





