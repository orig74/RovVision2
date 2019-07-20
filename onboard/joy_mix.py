# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
from config import Joy_map as jm


#return inertial_joy,ud_joy,lr_joy,fb_joy,yaw_joy,pitch_joy,roll_joy
def joy_mix(axis,joy_buttons):
    inertial = joy_buttons[jm.shift2_bt]==1
    
    fb,lr = (0,0) if joy_buttons[jm.shift1_bt] else (-axis[jm.fb],axis[jm.lr])
    pitch,roll = (axis[jm.fb],axis[jm.lr]) if joy_buttons[jm.shift1_bt] else (0,0)
    ret = {'inertial':inertial, 
            'ud':axis[jm.ud],
            'lr':lr,
            'fb':fb,
            'yaw':axis[jm.yaw],
            'pitch':pitch,
            'roll':roll}
    return ret

