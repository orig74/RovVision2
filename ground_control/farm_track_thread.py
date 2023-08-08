import time
import json
import numpy as np
states = ['wait_start','stabilize','go_down','stabilize','slide','stabilize','go_up','stabilize','slide','stabilize']
#states = ['stabilize','go_down','stabilize','slide','stabilize','go_up','stabilize','slide','stabilize']

mission_vars_default=[
    ('horizontal_slide',0.8),
    ('back_slide',-0.2),
    ('target_depth_up',0.5),
    ('target_depth_down',1.0),
    ('minimal_step_time',15),
    ('vertical_step',0.1),
    ('slide_step',0.1),
    ('max_iters',3),
    ('max_alt',3),
    ]

tool_tips={
        'horizontal_slide':'horizontal slide between ropes right is positive',
        'max_alt':'maximum dvl alt from seabed'
    }

class FarmTrack(object):
    def __init__(self,iterations=1,rov_comander=None,rov_data_handler=None,printer=None):
        self.auto_next=True
        self.set_params(mission_vars_default)

        self.rov_comander=rov_comander
        self.rov_data_handler=rov_data_handler
        self.printer=printer
        #self.override_do_next=False
        self.reset()

    def set_params(self,mission_vars):
        for k,v in mission_vars:
            self.__dict__[k]=v
        print('=======',mission_vars)

    def reset(self):
        self.state_ind=0
        self.last_run=time.time()
        self.start_step_time=time.time()
        self.tmp_target_depth=None
        self.tmp_target_slide=None
        self.final_target_slide=None
        self.final_target_depth=None
        self.states=states+states[2:]*int((self.max_iters-1))+['done']

    def get_state(self):
        return self.states[self.state_ind]

    def start(self):
        self.printer('start mission ----')
        self.reset()
        self.__inc_step()

    def step_time(self):
        return time.time()-self.start_step_time 

    def __inc_step(self):
        if self.get_state()=='done':
            return 
        self.state_ind+=1
        self.printer(f'{self.state_ind}/{len(self.states)} state is: {self.states[self.state_ind]}')
        self.start_step_time=time.time()

    def __target_depth_achived(self):
        dh=self.rov_data_handler
        if self.final_target_depth is None:
            return False
        #return abs(dh.get_depth()-dh.get_target_depth())<0.2
        return abs(dh.get_depth()-self.final_target_depth)<0.2

    def __target_xy_achived(self,tresh=0.3):
        #if self.override_do_next:
        #    return True
        dh=self.rov_data_handler
        xy=dh.get_pos_xy2()
        if self.final_target_slide is None:
            return False
        #txy=dh.get_target_xy()
        txy=self.final_target_slide
        dx = abs(xy[0]-txy[0])
        dy = abs(xy[1]-txy[1])
        #self.printer(f'dslide {dx:.2f} {dy:.2f}')
        return dx<tresh and dy<tresh

    def __is_stable_xy(self,tresh=0.2):
        dh=self.rov_data_handler
        xy=dh.get_pos_xy2()
        txy=dh.get_target_xy()
        dx = abs(xy[0]-txy[0])
        dy = abs(xy[1]-txy[1])
        #self.printer(f'dxdy {dx:.2f} {dy:.2f} {dx<tresh and dy<tresh}')
        return dx<tresh and dy<tresh

    def run(self,range_to_target,max_alt,Pxy):
        dh=self.rov_data_handler
        tic=time.time()
        if tic-self.last_run<0.5:
            return
        self.last_run=tic

        if self.states[self.state_ind] in ['wait_start','done']:
            return

        do_next = self.__is_stable_xy()
        do_next = do_next and self.step_time()>self.minimal_step_time
        #do_next = do_next or self.override_do_next

        if self.states[self.state_ind]=='stabilize':
            #self.printer(f'M: xy_ach: {self.__target_xy_achived()} done_st: {self.done_step}')
            if do_next:
                self.__inc_step()
                if self.states[self.state_ind]=='slide':
                    self.printer('going slide')
                    self.last_rope_xy=dh.get_pos_xy2()
                    self.rov_comander.vertical_object_unlock()
                    self.final_target_slide=np.array([self.last_rope_xy[0]+self.back_slide,self.last_rope_xy[1]+self.horizontal_slide])
                    self.tmp_target_slide=np.array(self.last_rope_xy)
                    #self.rov_comander.go(self.tmp_target_slide,relative=False)

                if self.states[self.state_ind]=='go_down':
                    self.printer('going down')
                    #self.rov_comander.lock_max()
                    self.rov_comander.vertical_object_lock(rng=range_to_target,Pxy=Pxy)
                    self.final_target_depth=self.target_depth_down
                    self.tmp_target_depth=dh.get_depth()
                    #self.rov_comander.depth_command(self.target_depth_down,relative=False)

                if self.states[self.state_ind]=='go_up':
                    self.printer('going up')
                    #self.rov_comander.lock_max()
                    self.rov_comander.vertical_object_lock(rng=range_to_target,Pxy=Pxy)
                    self.final_target_depth=self.target_depth_up
                    self.tmp_target_depth=dh.get_depth()
                    #self.rov_comander.depth_command(self.target_depth_up,relative=False)

        elif self.states[self.state_ind].startswith('go'):
            #self.printer(f'M: d_ach: {self.__target_depth_achived()} done_st: {self.done_step}')
            too_close_to_seabed = False
            is_go_down= self.states[self.state_ind]=='go_down'
            end_rope_detected = dh.get_rope_down_end_detected() and is_go_down
            if dh.get_alt() is not None and dh.get_alt()<max_alt and is_go_down:
                too_close_to_seabed=True
                self.rov_comander.depth_command(dh.get_depth(),relative=False)
                self.printer(f'too close to seabed {dh.get_alt()}')

            if end_rope_detected:
                self.printer(f'end rope detected {dh.get_alt()}')
                self.rov_comander.depth_command(dh.get_depth(),relative=False)

            if (self.__target_depth_achived() and do_next) or too_close_to_seabed or end_rope_detected:
                self.printer(f'done up down stage {self.__target_depth_achived()}')
                self.final_target_depth=None
                self.__inc_step()
            else:
                v=self.final_target_depth-self.tmp_target_depth
                if abs(v)<=2*self.vertical_step:
                    self.rov_comander.depth_command(self.final_target_depth,relative=False)
                else:
                    self.tmp_target_depth+=self.vertical_step*np.sign(v)
                    self.rov_comander.depth_command(self.tmp_target_depth,relative=False)


        elif self.states[self.state_ind]=='slide':
            if self.__target_xy_achived() and do_next:
                self.final_target_slide=None
                self.__inc_step()
                self.rov_comander.lock_max()
                self.rov_comander.vertical_object_lock(rng=range_to_target,Pxy=Pxy)
            else:
                v=self.final_target_slide-self.tmp_target_slide
                v_norm=np.linalg.norm(v)
                if v_norm<=self.slide_step:
                    self.rov_comander.go(self.final_target_slide,relative=False)
                else:
                    self.tmp_target_slide+=self.slide_step*v/v_norm
                    self.rov_comander.go(self.tmp_target_slide,relative=False)
