# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#Joystick configuration
#mode 2
#jtype = 'sony'
import time
import sys
sys.path.append('..')
import config

jtype='xbox'
class Joy_map:
    if jtype=='sony':
        _left_stick_fwd_bak=1
        _left_stick_right_left=0
        _right_stick_fwd_bak=4
        _right_stick_left_right=3
        _left_shift=4 #left shift
        _right_shift=5 #right shift
        _home=10
        _start=9
        _red=1 #circle / rkeys right
        _yelow=2 #triangle / rkeys up
        _blue=3 #square /rkeys left
        _green = 0 #X /rkeys down

    if jtype=='xbox':
        _left_stick_click=9
        _left_stick_fwd_bak=1
        _left_stick_right_left=0
        _right_stick_click=10
        _right_stick_fwd_bak=4
        _right_stick_left_right=3
        _left_shift=4 #left shift
        _right_shift=5 #right shift
        _home=8
        _start=7
        _red=1 #circle / rkeys right
        _yelow=3 #triangle / rkeys up
        _blue=2 #square /rkeys left
        _green = 0 #X /rkeys down
        

    def __init__(self):
        self.buttons=[0]*16
        self.prev_buttons=[0]*16
        self.axis=[0]*8
        self.last_hat=time.time()

    def update_buttons(self,buttons):
        self.prev_buttons=self.buttons
        self.buttons=buttons

    def update_axis(self,axis):
        if config.joy_deadband>0:
            naxis = [x  if abs(x)>config.joy_deadband else 0 for x in axis]
            self.axis=naxis
        else:
            self.axis=axis


    def __test_togle(self,b):
        try:
            return self.buttons[b]==1 and self.prev_buttons[b]==0
        except Exception as E:
            print('Error in __test_togle',E)
       
    def __left_shift(self):
        return self.buttons[self._left_shift]
    
    def __right_shift(self):
        return self.buttons[self._right_shift]
    
    def __no_shift(self):
        return not self.buttons[self._right_shift] and not self.buttons[self._left_shift]

    def arm_event(self):
        return self.__test_togle(self._start)
    
    def att_hold_event(self):
        return self.__test_togle(self._yelow) and self.__no_shift()

    def sonar_hold_event(self):
        return self.__test_togle(self._blue) and self.__no_shift()

    def depth_hold_event(self):
        return self.__test_togle(self._red) and self.__no_shift()

    def record_event(self):
        return self.__test_togle(self._home)

    def Rx_hold_event(self):
        return self.__test_togle(self._yelow) and self.__left_shift()
    
    def Ry_hold_event(self):
        return self.__test_togle(self._red) and self.__left_shift()
    
    def Rz_hold_event(self):
        return self.__test_togle(self._blue) and self.__left_shift()

    def auto_nav_event(self):
        return self.__right_shift() and self.__left_shift()

    def cam_calib_event(self):
        return self.__test_togle(self._right_stick_click) and self.__right_shift()

    def image_rect_event(self):
        return self.__test_togle(self._right_stick_click) and not self.__right_shift()

    def main_camera_up_event(self):
        return self.__test_togle(self._yelow) and self.__right_shift()

    def main_camera_down_event(self):
        return self.__test_togle(self._green) and self.__right_shift()

    def dvl_reset_event(self):
        return self.__test_togle(self._blue) and self.__right_shift()



    def track_lock_event(self):
        return self.__test_togle(self._green) and self.__left_shift()

    def inc_lights_event(self):
        if jtype=='xbox':
            axis=self.axis
            if axis[7]<-0.9 and time.time()-self.last_hat>0.3:
                self.last_hat=time.time()
                return True
        return False

    def dec_lights_event(self):
        if jtype=='xbox':
            axis=self.axis
            if axis[7]>0.9 and time.time()-self.last_hat>0.3:
                self.last_hat=time.time()
                return True
        return False

    def inc_freqs_track_rope(self):
        if jtype=='xbox':
            axis=self.axis
            #print('axis[6]',axis,time.time()-self.last_hat)
            if axis[6]<-0.9 and time.time()-self.last_hat>0.3:
                self.last_hat=time.time()
                return True
        return False
    
    def dec_freqs_track_rope(self):
        if jtype=='xbox':
            axis=self.axis
            if axis[6]>0.9 and time.time()-self.last_hat>0.3:
                self.last_hat=time.time()
                return True
        return False



    def joy_mix(self):
        joy_buttons=self.buttons
        axis=self.axis
        jm=Joy_map
        inertial = joy_buttons[jm._right_shift]==1
        
        fb,lr = (0,0) if joy_buttons[jm._left_shift] else (-axis[jm._right_stick_fwd_bak],axis[jm._right_stick_left_right])
        pitch,roll = (axis[jm._right_stick_fwd_bak],axis[jm._right_stick_left_right]) if joy_buttons[jm._left_shift] else (0,0)
        ret = {'inertial':inertial, 
                'ud':axis[jm._left_stick_fwd_bak],
                'lr':lr,
                'fb':fb,
                'yaw':axis[jm._left_stick_right_left],
                'pitch':pitch,
                'roll':roll}
        return ret

