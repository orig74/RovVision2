from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
import numpy as np
import config

class CycArr():
    def __init__(self,size=20000):
        self.buf=[]
        self.size=size

    def add(self,arr):
        self.buf.append(arr)
        if len(self.buf)>self.size:
            self.buf.pop(0)

    def __call__(self):
        return np.array(self.buf)

    def __len__(self):
        return len(self.buf)

    def clear(self):
        self.buf=[]


class Data:
    def reset(self):
        self.curr_pos=None
        self.pos_hist = CycArr()
        self.trace_hist = CycArr(500)
        self.map_center = (0,0)
        self.range_arr = CycArr(500)
        self.prev_pts = None
        self.last_ref = -1

    def __init__(self):
        self.reset()

gdata=Data()

def roty(a):
    R_y = \
        np.array([\
            [np.cos(a),    0,      np.sin(a)  ],
            [0,                     1,      0         ],
            [-np.sin(a),   0,      np.cos(a)  ]
                    ])
    return R_y
def rotx(a):
        ca = np.cos(a)
        sa = np.sin(a)

        R_x = \
            np.array([  [   1,  0,  0   ],
                        [   0,  ca, -sa ],
                        [   0,  sa, ca  ],
                        ])
        return R_x

def rotz(a):
    ch = np.cos(a)
    sh = np.sin(a)
    Rz = np.array([
        [   ch,     -sh,    0],
        [   sh,     ch,     0],
        [   0,      0,      1]])
    return Rz


def get_rot(yaw,pitch,roll):
    #return rotz(np.radians(yaw)) @ roty(np.radians(pitch)+np.radians(args.camera_pitch)) @ rotx(np.radians(roll))
    #return roty(np.radians(yaw)) @ rotx(np.radians(pitch)+np.radians(args.camera_pitch)) @ rotz(np.radians(roll))
    #R=rotz(np.radians(roll)) @ rotx(np.radians(pitch)+np.radians(args.camera_pitch)) @ roty(np.radians(yaw))
    MR=rotx(np.radians(roll))
    MP=roty(np.radians(pitch))
    MCP=roty(np.radians(args.camera_pitch))
    MY=rotz(np.radians(yaw))
    R=MCP @ MR @ MP @ MY
    return R

class Plotter(object):
    def __init__(self,canvas):
        self.figure1 = Figure(figsize=(4,4), dpi=100)
        self.ax1 = self.figure1.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure1, master=canvas)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=1)
        self.initPlots()
        self.runPlotsFlag = True
        self.rad=4.0

    def initPlots(self):
        self.ax1.clear()
        self.hdl_pos = self.ax1.plot([1,2],[1,2],'-')
        self.hdl_arrow = self.ax1.arrow(1,1,0.5,0.5,width=0.1)
        self.hdl_target_pos = self.ax1.plot([1],[1],'+')
        self.ax1.grid('on')
        self.canvas.draw()

    def center(self):
        gdata.map_center = gdata.curr_pos[:]
        
    def clear_trace(self):
        gdata.pos_hist.clear() 

    def update_dvl_data(self,ldata,target_pos=None,yaw_deg=None):
        if ldata is not None and ldata['type']=='deadreacon':
            #new_data=True
            ret=(ldata['y'],ldata['x'])
            gdata.pos_hist.add(ret)
            #print('===',ret)
            gdata.trace_hist.add(ret)
            gdata.curr_pos=ret
            #self.yaw_rad = np.radians(ldata['yaw'] if yaw_deg is None else yaw_deg)
            self.yaw_rad = np.radians(ldata['yaw'])
            self.target_pos=target_pos

    def redraw(self):

            yaw_rad=self.yaw_rad
            ch=np.cos(yaw_rad-np.pi/2)
            sh=np.sin(yaw_rad-np.pi/2)

            pos_arr = gdata.pos_hist()
            #pos_arr=pos_arr-pos_arr[0,:]
            self.hdl_pos[0].set_ydata(pos_arr[:,1])
            self.hdl_pos[0].set_xdata(pos_arr[:,0])
            #hdl_last_pos
            self.hdl_arrow.remove()
            self.hdl_arrow = self.ax1.arrow(gdata.curr_pos[0],gdata.curr_pos[1],ch*0.005,-sh*0.005,width=0.1,alpha=0.4)

            if self.target_pos is None:
                self.target_pos=(0,0)

            x,y = self.target_pos[:2]
            x,y = x*ch-y*sh,x*sh+y*ch
            
            self.hdl_target_pos[0].set_ydata(-x)
            self.hdl_target_pos[0].set_xdata(y)

            cx,cy = gdata.map_center[:2]
            rad=self.rad
            self.ax1.set_xlim(-rad+cx,rad+cx)
            self.ax1.set_ylim(-rad+cy,rad+cy)
            self.canvas.draw()




