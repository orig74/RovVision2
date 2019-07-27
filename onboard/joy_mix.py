# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
#Joystick configuration
#mode 2
class Joy_map:
    _ud=1
    _yaw=0
    _fb=4
    _lr=3
    _shift1_bt=4
    _shift2_bt=5
    _record_bt=10
    _arm_disarm=9
    _depth_hold_bt=1
    _att_hold_bt=2

    def __init__(self):
        self.buttons=[0]*16
        self.prev_buttons=[0]*16
        self.axis=[0]*6

    def update_buttons(self,buttons):
        self.prev_buttons=self.buttons
        self.buttons=buttons

    def update_axis(self,axis):
        self.axis=axis

    def __test_togle(self,b):
        return self.buttons[b]==1 and self.prev_buttons[b]==0

    def arm_event(self):
        return self.__test_togle(self._arm_disarm)
    
    def att_hold_event(self):
        return self.__test_togle(self._att_hold_bt)

    def depth_hold_event(self):
        return self.__test_togle(self._depth_hold_bt)

    def record_event(self):
        return self.__test_togle(self._record_bt)

    def joy_mix(self):
        joy_buttons=self.buttons
        axis=self.axis
        jm=Joy_map
        inertial = joy_buttons[jm._shift2_bt]==1
        
        fb,lr = (0,0) if joy_buttons[jm._shift1_bt] else (-axis[jm._fb],axis[jm._lr])
        pitch,roll = (axis[jm._fb],axis[jm._lr]) if joy_buttons[jm._shift1_bt] else (0,0)
        ret = {'inertial':inertial, 
                'ud':axis[jm._ud],
                'lr':lr,
                'fb':fb,
                'yaw':axis[jm._yaw],
                'pitch':pitch,
                'roll':roll}
        return ret

