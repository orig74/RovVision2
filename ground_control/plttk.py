from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib import pyplot as plt
import numpy as np

class Plotter(object):
    def __init__(self,canvas):
        self.figure1 = Figure(figsize=(6,6), dpi=100)
        self.ax1 = self.figure1.add_subplot(211)
        self.ax2 = self.figure1.add_subplot(212)
        self.canvas = FigureCanvasTkAgg(self.figure1, master=canvas)
        self.canvas.get_tk_widget().pack(side="top", fill="both", expand=1)
        self.initPlots()
        self.runPlotsFlag = True

    def initPlots(self):
        self.ax1.clear()
        self.ax2.clear()
        al=0.6
        self.hdls=[self.ax1.plot([1],'-b', alpha=al), self.ax1.plot([1],'-g', alpha=al), self.ax1.plot([1],'-r', alpha=al), self.ax1.plot([1],'-k', alpha=al)]
        self.ax1.legend(list('PIDC'))
        self.ax1.grid('on')
        self.hdls2=[self.ax2.plot([1],'-b', alpha=al), self.ax2.plot([1],'-g', alpha=al), self.ax2.plot([1],'-r', alpha=al), self.ax2.plot([1],'-k', alpha=al)]
        self.ax2.legend(list('TNR'))
        self.ax2.grid('on')
        self.canvas.draw()

    def update_pid(self,cyc_buffer,ylim=None):
        if not cyc_buffer.changed:
            return 
        data = cyc_buffer.get_data(['TS','P','I','D','C'])
        xs = np.arange(data.shape[0])
        ax,hdls = self.ax1, self.hdls#ax_hdls[0][0],ax_hdls[0][1:]
        for i in [0,1,2,3]:
            hdls[i][0].set_ydata(data[:,i+1]) #skip timestemp
            hdls[i][0].set_xdata(xs)
        ax.set_xlim(data.shape[0]-400,data.shape[0])
        if ylim is not None:
            ax.set_ylim(-ylim,ylim)
        else:
            #ignore timetag thats why 1:
            min_y = data[:,1:].min()-0.01
            max_y = data[:,1:].max()+0.01
            ax.set_ylim(min_y,max_y)


        ax2,hdls2 = self.ax2,self.hdls2#ax_hdls[1][0],ax_hdls[1][1:]
        data = cyc_buffer.get_data(['T','N','R'])
        xs = np.arange(data.shape[0])
        #cmd_data=gdata.md_hist.get_data(label+'_cmd')
        for i in [0,1,2]:
            hdls2[i][0].set_ydata(data[:,i])
            hdls2[i][0].set_xdata(xs)
        ax2.set_xlim(data.shape[0]-400,data.shape[0])
        min_y = data.min()-0.1
        max_y = data.max()+0.1
        ax2.set_ylim(min_y,max_y)
        cyc_buffer.changed = False 
        self.canvas.draw()


