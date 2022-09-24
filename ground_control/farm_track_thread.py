import time
states = ['wait_start','stabilize_up1','go_down','stabilize_down1','slide_down','stabilize_down2','go_up','stabilize_up2','slide_up']

class FarmTrack(object):
    def __init__(self,iterations=1,rov_comander=None,rov_data_handler=None,printer=None):
        self.printer=printer
        self.auto_next=False
        self.horizontal_range=-3.0
        self.verical_range=10.0
        self.range_to_target=0.29
        self.target_depth_up=3
        self.target_depth_down=10
        self.Py=0.1
        self.Px=0.1
        self.pause=False
        
        self.rov_comander=rov_comander
        self.rov_data_handler=rov_data_handler

        self.reset()


    def reset(self):
        self.state_ind=0
        self.done_step=self.auto_next
        self.iter=0

    def start(self):
        self.state_ind=1
        self.done_step=self.auto_next

    def update_telem(self,telem):
        self.telem=telem

    def do_next(self):
        self.done_step=True

    def __get_horizontal_rope_delta(self):
        return 0

    def __get_range_delta(self):
        return 0

    def __get_xy_pos(self):
        pass

    def __arrived_target_depth_up(self):
        return True

    def __arrived_target_depth_down(self):
        return True

    def __arrive_target_horiz(self);
        pass

    def __is_rope_center(self):
        return True

    def __is_range_ok(self):
        return True

    def pause(self,x):
        self.pause=x

    def __inc_step(self):
        self.state_ind+=1
        self.state_ind=state_ind*len(states)
        self.printer('state is: '+stats[self.state_ind])
        self.done_step=self.auto_next

    def run(self):
        horiz_ok = self.__is_rope_center() and self.__is_range_ok()

        if states[self.state_ind].startswith('stabilize'):
            if not self.pause:
                pass
            if  horiz_ok and self.done_step:
                self.__inc_step()
        if states[self.state_ind]=='go_down':
            if  self.__arrived_target_depth_down() and self.done_step:
                self.__inc_step()
        if states[self.state_ind]=='go_up':
            if  self.__arrived_target_depth_up() and self.done_step:
                self.__inc_step()






            
        
        


