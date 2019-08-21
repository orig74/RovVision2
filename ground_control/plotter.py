# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import matplotlib
matplotlib.use('TKAgg')

import matplotlib.pyplot as plt
import sys,os,time
sys.path.append('../')
import zmq
import pickle
import argparse
import numpy as np
import sys
sys.path.append('../')
sys.path.append('../utils')
import zmq_wrapper 

parser = argparse.ArgumentParser()
parser.add_argument("--topic",help="which topic")
parser.add_argument("--port",help="which port",type=int)
parser.add_argument("--type",help="plot type: pid or vector",default='pid')
args = parser.parse_args()

subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([args.topic.encode() ], args.port) )

class CycArr():
    def __init__(self,size=20000):
        self.buf=[]
        self.size=size

    def add(self,arr):
        self.buf.append(arr)
        if len(self.buf)>self.size:
            self.buf.pop(0)

    def get_data(self,labels):
        data = np.zeros((len(self.buf),len(labels)))
        for i,d in enumerate(self.buf):
            for j,l in enumerate(labels):
                data[i][j]=d[l]
        return data

    def get_vec(self):
        return np.array([d for _,d in self.buf])

    def __len__(self):
        return len(self.buf)

    def reset(self):
        self.buf=[]


msgs = {}

def update_graph(axes):
    tic=time.time()
    new_data=False
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
                data=pickle.loads(ret[1])
                if topic not in msgs:
                    msgs[topic] = CycArr(500)
                msgs[topic].add(data)
                new_data=True

    if not pause_satus and new_data:
        if args.type=='pid':
            update_pid(axis_hdls,args.topic.encode())
            axes.figure.canvas.draw()
        if args.type=='vector':
            update_vector(axis_hdls,args.topic.encode())
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

def plot_vec(pid_label):
    ax=plt.subplot(1,1,1)
    plt.title(pid_label)
    hdls=[ax.plot([1]) for _ in range(8)]
    plt.legend(list('12345678'),loc='upper left')
    plt.grid('on')
    return (ax,*hdls)

def update_vector(ax_hdls,topic):
    if topic not in msgs:
        return
    data = msgs[topic].get_vec()
    xs = np.arange(data.shape[0])
    ax,hdls = ax_hdls
    hdls[0].set_ydata(data) #skip timestemp
    hdls[0].set_xdata(xs)
    ax.set_xlim(data.shape[0]-400,data.shape[0])
    ax.set_ylim(-1,1)

def plot_pid(pid_label):
    ax=plt.subplot(2,1,1)
    plt.title(pid_label)
    hdls=[ax.plot([1],'-b'),ax.plot([1],'-g'),ax.plot([1],'-r'), ax.plot([1],'-k')]
    plt.legend(list('pidc'),loc='upper left')
    plt.grid('on')

    ax2=plt.subplot(2,1,2,sharex=ax)
    plt.title(pid_label + ' target')
    hdls2=[ax2.plot([1],'-b'),ax2.plot([1],'-g'), ax2.plot([1],'-r')]
    plt.legend(list('TNR'),loc='upper left')
    plt.grid('on')
    return ((ax,*hdls),(ax2,*hdls2))

def update_pid(ax_hdls,topic):
    if topic not in msgs:
        return
    data = msgs[topic].get_data(['TS','P','I','D','C'])
    xs = np.arange(data.shape[0])
    ax,hdls = ax_hdls[0][0],ax_hdls[0][1:]
    for i in [0,1,2,3]:
        hdls[i][0].set_ydata(data[:,i+1]) #skip timestemp
        hdls[i][0].set_xdata(xs)
    ax.set_xlim(data.shape[0]-400,data.shape[0])
    ax.set_ylim(-1,1)

    ax2,hdls2 = ax_hdls[1][0],ax_hdls[1][1:]
    data = msgs[topic].get_data(['T','N','R'])
    xs = np.arange(data.shape[0])
    #cmd_data=gdata.md_hist.get_data(label+'_cmd')
    for i in [0,1,2]:
        hdls2[i][0].set_ydata(data[:,i])
        hdls2[i][0].set_xdata(xs)
    ax2.set_xlim(data.shape[0]-400,data.shape[0])
    min_y = data.min()
    max_y = data.max()
    ax2.set_ylim(min_y,max_y)

from matplotlib.widgets import Button

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
axcenter = plt.axes([0.59, 0.05, 0.1, 0.075])
axpause = plt.axes([0.7, 0.05, 0.1, 0.075])
axclear = plt.axes([0.81, 0.05, 0.1, 0.075])

timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_graph, ax)
timer.start()

bnpause = Button(axpause, 'Pause')
bnpause.on_clicked(pause)
bnclear = Button(axclear, 'Clear')
bnclear.on_clicked(clear)
bncenter = Button(axcenter, 'Center')
bncenter.on_clicked(center)

if args.type=='pid':
    axis_hdls=plot_pid(args.topic)
if args.type=='vector':
    axis_hdls=plot_vec(args.topic)

plt.show()
