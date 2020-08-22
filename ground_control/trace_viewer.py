# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import matplotlib
import argparse
matplotlib.use('TKAgg')
import matplotlib.pyplot as plt
import sys,os,time
from datetime import datetime
sys.path.append('../')
sys.path.append('../utils')
sys.path.append('../onboard')
import zmq
import pickle
import select
import struct
import cv2,os
import sys,os,time
import zmq_topics
import zmq_wrapper as utils
parser = argparse.ArgumentParser()
parser.add_argument("--ip",help="main ground control ip addr", default='127.0.0.1')
parser.add_argument("--scale",help="map size deafult 20", default=20.0, type=float)
args = parser.parse_args()

subs_socks=[]
subs_socks.append(utils.subscribe([zmq_topics.topic_tracker], zmq_topics.topic_tracker_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_imu], zmq_topics.topic_imu_port))
subs_socks.append(utils.subscribe([zmq_topics.topic_sonar], zmq_topics.topic_sonar_port))

##### map radious im meters
rad=float(args.scale)

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


class Data:
    def reset(self):
        self.curr_pos=None
        self.pos_hist = CycArr()
        self.trace_hist = CycArr(500)
        self.heading_rot = None
        self.map_center = (0,0)
        self.range_arr = CycArr(500)
        self.prev_pts = None
        self.last_ref = -1

    def __init__(self):
        self.reset()

gdata=Data()

#from utils import ab_filt
#xf,yf,zf=ab_filt(),ab_filt(),ab_filt()

ch,sh=0,0
fd = open(r'trace_data.pkl','wb')
def update_graph(axes):
    global hdl_pos,hdl_arrow,ch,sh
    tic=time.time()
    new_data=False
    tin_data={}
    while 1:
        socks=zmq.select(subs_socks,[],[],0.001)[0]
        if time.time()-tic>=0.09:
            print('too much time break',time.time()-tic())
            break
        if len(socks)==0:
            break
        else:
            for sock in socks:
                ret = sock.recv_multipart()
                topic , data = ret
                #print('--- topic ---',topic)
                data=pickle.loads(ret[1])
                if topic==zmq_topics.topic_tracker:
                    #new_data=True
                    keys=['valid','pt_l','pt_r','range','ref_cnt'] 
                    #print([(i,data[i]) for i in keys])
                    for k in keys:
                        tin_data[k]=data[k]
                    pickle.dump(tin_data,fd)
                    print(tin_data) 
                if topic==zmq_topics.topic_imu:
                    for k in ['yaw','pitch','roll']:
                        tin_data[k]=data[k]
                if topic==zmq_topics.topic_sonar:
                    tin_data['sonar']=data
                    #new_data=True
                    #toprint=['valid','pt_l','pt_r','range']
                    #print('--imu--',data)

    if not pause_satus and new_data:
        xs = np.arange(len(gdata.trace_hist))
        hdl_pos[0].set_ydata(pos_arr[:,1])
        hdl_pos[0].set_xdata(pos_arr[:,0])
        #hdl_last_pos
        for i in [0,1,2]:
            hdl_trace[i][0].set_xdata(xs)
            hdl_trace[i][0].set_ydata(gdata.trace_hist()[:,i])
        ax2.set_xlim(len(gdata.trace_hist)-100,len(gdata.trace_hist))
        ax2.set_ylim(-0.2*4,0.2*4)
        hdl_arrow.remove()
        hdl_arrow = ax1.arrow(gdata.curr_pos[0],gdata.curr_pos[1],-ch*0.1,-sh*0.1,width=0.3)

        cx,cy = gdata.map_center[:2]
        ax1.set_xlim(-rad+cx,rad+cx)
        ax1.set_ylim(-rad+cy,rad+cy)

        xs = np.arange(len(gdata.range_arr))
        hdl_range[0][0].set_xdata(xs)
        #print(pos_arr[:,2][-3:])
        hdl_range[0][0].set_ydata(gdata.range_arr.buf)
        ax3.set_xlim(len(xs)-100,len(xs))
        ax3.set_ylim(0,2)

        axes.figure.canvas.draw()

def clear(evt):
    gdata.reset()
    print('reset data')

pause_satus=False
def pause(evt):
    global pause_satus
    pause_satus=not pause_satus
    print('pause=',pause_satus)

def center(evt):
    gdata.map_center = gdata.curr_pos.copy()

from matplotlib.widgets import Button

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
axcenter = plt.axes([0.59, 0.05, 0.1, 0.075])
axpause = plt.axes([0.7, 0.05, 0.1, 0.075])
axclear = plt.axes([0.81, 0.05, 0.1, 0.075])




ax1=plt.subplot2grid((3,2), (0,1),rowspan=3)
hdl_pos = ax1.plot([1,2],[1,2],'-')
hdl_arrow = ax1.arrow(1,1,0.5,0.5,width=0.1)
plt.xlabel('[m]')
plt.ylabel('[m]')

ax2=plt.subplot2grid((3,2), (0,0))
plt.title('trace not oriented')
plt.legend(list('xyz'))
plt.xlabel('[frame]')
plt.ylabel('[m/frame]')
hdl_trace = [ax2.plot([1],'-r'),ax2.plot([1],'-g'),ax2.plot([1],'-b')]

ax3=plt.subplot2grid((3,2), (2,0))
plt.title('ground range')
hdl_range  = [ax3.plot([1],'-')]
plt.xlabel('[frame]')
plt.ylabel('[m]')
plt.grid('on')



timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_graph, ax)
timer.start()


bnpause = Button(axpause, 'Pause')
bnpause.on_clicked(pause)
bnclear = Button(axclear, 'Clear')
bnclear.on_clicked(clear)
bncenter = Button(axcenter, 'Center')
bncenter.on_clicked(center)

plt.show()
