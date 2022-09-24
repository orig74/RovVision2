import time
states = ['wait_start','stabilize','go_down','stabilize','slide','stabilize','go_up','stabilize','slide','stabilize']

class FarmTrack(object):
    def __init__(self,iterations=1,rov_comander=None,rov_data_handler=None,printer=None):
        self.printer=printer
        self.auto_next=False
        self.horizontal_slide=-3.0
        self.range_to_target=0.29
        self.target_depth_up=3
        self.target_depth_down=10
        self.pause=False
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

    def start(self):
        self.state_ind=1
        self.done_step=self.auto_next

    def do_next(self):
        self.done_step=True

    def pause(self,x):
        self.pause=x

    def __inc_step(self):
        self.state_ind+=1
        self.state_ind=self.state_ind%len(states)
        self.printer('state is: '+states[self.state_ind])
        self.done_step=self.auto_next

    def run(self):
        if states[self.state_ind]=='stabilize':
            if not self.pause:
                pass
            if self.rov_data_handler.vertical_lock_state()=='locked' and self.done_step:
                self.__inc_step()
                if states[self.state_ind]=='slide':
                    self.rov_comander.vertical_object_unlock()
                    self.rov_comander.go((0,self.horizontal_slide),relative=True)
                if states[self.state_ind]=='go_down':
                    self.rov_comander.depth_command(self.target_depth_down,relative=False)
                if states[self.state_ind]=='go_up':
                    self.rov_comander.depth_command(self.target_depth_up,relative=False)

        if states[self.state_ind].startswith('go'):
            if self.rov_data_handler.arrived_target_depth() and self.done_step:
                self.__inc_step()
                self.rov_comander.vertical_object_lock()






            
        
        


