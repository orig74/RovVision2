import time
states = ['wait_start','stabilize','go_down','stabilize','slide','stabilize','go_up','stabilize','slide','stabilize']

class FarmTrack(object):
    def __init__(self,iterations=1,rov_comander=None,rov_data_handler=None,printer=None):
        self.printer=printer
        self.auto_next=False
        self.horizontal_slide=0.8
        self.back_slide=-0.2
        self.target_depth_up=0.5
        self.target_depth_down=1.0
        self.rov_comander=rov_comander
        self.rov_data_handler=rov_data_handler
        self.reset()

    def reset(self):
        self.state_ind=0
        self.done_step=self.auto_next
        self.iter=0
        self.current_y_command=0
        self.current_x_command=0
        self.current_depth_command=self.target_depth_up
        self.last_run=time.time()

    def start(self):
        self.state_ind=0
        self.__inc_step()

    def do_next(self):
        self.done_step=True

    def __inc_step(self):
        self.state_ind+=1
        self.state_ind=self.state_ind%len(states)
        self.printer('state is: '+states[self.state_ind])
        self.done_step=self.auto_next

    def __target_depth_achived(self):
        dh=self.rov_data_handler
        return abs(dh.get_depth()-dh.get_target_depth())<0.1

    def __target_xy_achived(self,tresh=0.1):
        dh=self.rov_data_handler
        xy=dh.get_pos_xy()
        txy=dh.get_target_xy()
        return abs(xy[0]-txy[0])<tresh and abs(xy[1]-txy[1])<tresh

    def run(self,range_to_target,Pxy):
        dh=self.rov_data_handler
        tic=time.time()
        if tic-self.last_run<0.5:
            return
        self.last_run=tic

        #self.printer(f'>>>> depth {dh.get_depth():.2f}')
        #self.printer(f'>>>> target depth {dh.get_target_depth():.2f}')
        #self.printer(f'>>>> xy {dh.get_pos_xy()}')
        #self.printer(f'>>>> target_xy {dh.get_target_xy()}')

        if states[self.state_ind]=='wait_start':
            return

        if states[self.state_ind]=='stabilize':
            self.printer(f'M: xy_ach: {self.__target_xy_achived()} done_st: {self.done_step}')
            if self.__target_xy_achived() and self.done_step:
                self.__inc_step()
                if states[self.state_ind]=='slide':
                    self.rov_comander.vertical_object_unlock()
                if states[self.state_ind]=='go_down':
                    self.rov_comander.depth_command(self.target_depth_down,relative=False)
                if states[self.state_ind]=='go_up':
                    self.rov_comander.depth_command(self.target_depth_up,relative=False)

        if states[self.state_ind].startswith('go'):
            self.printer(f'M: d_ach: {self.__target_depth_achived()} done_st: {self.done_step}')
            if self.__target_depth_achived() and self.done_step:
                self.__inc_step()
                self.rov_comander.vertical_object_lock(rng=range_to_target,Pxy=Pxy)

        if states[self.state_ind]=='slide':
            self.rov_comander.go((
                self.last_rope_xy[0]+self.back_slide,self.last_rope_xy[1]+self.horizontal_slide),relative=False)
            if self.__target_xy_achived() and self.done_step:
                self.rov_comander.lock_max()
                self.__inc_step()
        else:
            self.rov_comander.vertical_object_lock(rng=range_to_target,Pxy=Pxy)
            self.last_rope_xy=dh.get_pos_xy()
            






            
        
        


