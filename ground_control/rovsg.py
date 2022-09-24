# image_viewer.py
import io
import os
import PySimpleGUI as sg
from PIL import Image,ImageTk
import time

import os
import sys
import socket
import pickle
import json

sys.path.append('../onboard')
sys.path.append('../hw')
sys.path.append('../utils')
sys.path.append('..')

import config
from annotations import draw_mono
import numpy as np
import cv2
from select import select
import zmq
import image_enc_dec

import argparse
import matplotlib.pyplot as plt

from rov_data_handler import rovDataHandler,rovCommandHandler
from plttk import Plotter
from plttk_tracer import Plotter as TracePlotter

import zmq_topics


def img_to_tk(img,shrink=1,h_hsv=False):
    if h_hsv:
        img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:,:,0]
    if shrink==1:
        img = Image.fromarray(img)
    else:
        img = Image.fromarray(img[::shrink,::shrink])
    img = ImageTk.PhotoImage(img)
    return img

def draw_image(self, image):
    id = self._TKCanvas2.create_image((image.width()//2,image.height()//2), image=image)
    self.Images[id] = image
    return id

#unicode simbols from (https://unicode-table.com/en/sets/arrow-symbols/#up-down-arrows)
sym_up='\u2191'
sym_down='\u2193'
sym_left='\u21e6'
sym_right='\u21e8'
sym_fwd='\u21e7'
sym_back='\u21e9'
sym_yaw_left='\u21b6'
sym_yaw_right='\u21b7'

def printer(text,color=None):
    print('printer:',text)
    sg.cprint(text,c='black on white')

def main():
    rovHandler = rovDataHandler(None,printer=printer)
    rovCommander = rovCommandHandler()
    im_size = (960,600) 
    row1_layout = [[
        sg.Graph(im_size, graph_bottom_left=(0, im_size[1]), graph_top_right=(im_size[0],0) ,key="-MAIN-IMAGE-",
            change_submits=True,
            enable_events=True,
            ),
        sg.Image(key="-IMAGE-1-"),#,sg.Image(key="-IMAGE-2-")],
        ]]

    cmd_column = [
            [sg.Button('Arm-Disarm')],
            [sg.Button('Depth-Hold'),
                sg.Button(sym_up),sg.Button(sym_down),sg.Input(key='Target-Depth',default_text='0.1',size=(4,1))],
            [sg.Button('Att-hold'),sg.Text('Pitch:'),sg.Input(key='Target-Pitch',default_text='0.0',size=(4,1))],
            [sg.Button('X-hold'),sg.Button(sym_fwd),sg.Button(sym_back),sg.Input(key='Target-X',default_text='0.1',size=(4,1))],
            [sg.Button('Y-hold'),sg.Button(sym_left),sg.Button(sym_right),sg.Input(key='Target-Y',default_text='0.1',size=(4,1))],
            [sg.Button(sym_yaw_left),sg.Button(sym_yaw_right),sg.Input(key='DeltaYawD',default_text='1.0',size=(4,1))],
            [sg.Button('C',key='CENTER_MAX_CORR',tooltip='center on max corr')]

            ]

    yaw_source_options=['VNAV','DVL']
    config_column = [
            [sg.Text('Yaw Source:'),sg.Combo(yaw_source_options,key='YAW_SOURCE',default_value=yaw_source_options[0])],
            [sg.Button('Reset-DVL'),sg.Button('Calib-DVL')],
            [sg.Button('CF+'),sg.Button('CF-')],
            [sg.Button('Lights+'),sg.Button('Lights-')],
            [sg.Multiline(key='MESSEGES',s=(23,5), autoscroll=True, reroute_stdout=False, write_only=True)],
            [sg.Checkbox('H',key='HSV_H',tooltip='h from hsv on cam 1')],
            ]

    plot_options=['DEPTH','X_HOLD','Y_HOLD','YAW','PITCH','ROLL']
    matplot_column = [
        [sg.Text('Plot Type:'),sg.Combo(plot_options,key='-PLOT-TYPE-',default_value=plot_options[0]),
            sg.Button('P+'),sg.Button('P-'),
            sg.Button('I+'),sg.Button('I-'),
            sg.Button('D+'),sg.Button('D-'),
            sg.Text('Prc:'),sg.Input(key='PID_Mul',default_text='3.0',size=(4,1)),
            sg.Button('S',key='SAVE_PID'),
            ],
        [ sg.Canvas(key="-CANVAS-", size=(300,200)), sg.Canvas(key="-TRACE-CANVAS-", size=(300,300))]]

    row2_layout = [[
            #sg.Canvas(key="-CANVAS-", size=(500,500)),
            sg.Column(matplot_column),
            sg.VSeperator(),
            sg.Column(cmd_column,vertical_alignment='top'),
            sg.VSeperator(),
            sg.Column(config_column,vertical_alignment='top'),
    ]]

    layout = [
        [sg.Frame('',row1_layout)],
        [sg.Frame('',row2_layout)],
    ]

    

    window = sg.Window("ROV Viewer", 
            layout, finalize=True, 
            element_justification='left', 
            font='Helvetica 12',
            size=(1920,1080))
    #window['-MAIN-IMAGE-'].bind('<Button-1>','')
    plotter = Plotter(window["-CANVAS-"].TKCanvas)
    trace_plotter = TracePlotter(window["-TRACE-CANVAS-"].TKCanvas)
    
    last_im=None
    image_shape=None 
    cnt=0

    current_yaw_deg=0
    target_xy=[0,0]
    sg.cprint_set_output_destination(window, 'MESSEGES')
    while True:
        event, values = window.read(timeout=20) #10 mili timeout
        if event == "Exit" or event == sg.WIN_CLOSED:
            break

        if image_shape is not None and event.startswith('-MAIN'):
            x,y=values['-MAIN-IMAGE-']
            x=x/image_shape[1]
            y=y/image_shape[0]
            print('---click--',x,y)
            rovCommander.lock(x,y)

        frameId, rawImgs = rovHandler.getNewImages()

        if rawImgs is not None:
            image_shape=rawImgs[0].shape
            #print('===',time.time(),rawImgs[0].shape)
            #print(rawImgs[0].shape)
            if last_im is not None:
                #window["-MAIN-IMAGE-"].delete_figure(last_im)
                window["-MAIN-IMAGE-"].erase()
                #window["-MAIN-IMAGE-"].Images[last_im]=img_to_tk(rawImgs[0])
            
            #last_im=window["-MAIN-IMAGE-"].draw_image(data=img_to_tk3(rawImgs[0]),location=(0,0))#im_size[1]))
            last_im=draw_image(window["-MAIN-IMAGE-"],img_to_tk(rawImgs[0],h_hsv=values['HSV_H']))#im_size[1]))
            window["-IMAGE-1-"].update(data=img_to_tk(rawImgs[1],1))
 
        if event == "Arm-Disarm":
            rovCommander.armdisarm()
        if event == "Depth-Hold":
            rovCommander.depth_hold()
        if event == sym_down:
            rovCommander.depth_command(float(values['Target-Depth']))
        if event == sym_up:
            rovCommander.depth_command(-float(values['Target-Depth']))
        if event == 'Att-hold':
            rovCommander.att_hold()
        if event == 'CF+':
            rovCommander.clear_freqs(1)
        if event == 'CF-':
            rovCommander.clear_freqs(-1)
        if event == 'X-hold':
            rovCommander.x_hold()
        if event == 'Y-hold':
            rovCommander.y_hold()
        if event == 'Z-hold':
            rovCommander.z_hold()
        if event == sym_fwd:
            rovCommander.go((float(values['Target-X']),0,0))
        if event == sym_back:
            rovCommander.go((-float(values['Target-X']),0,0))
        if event == sym_right:
            rovCommander.go((0,float(values['Target-Y']),0))
        if event == sym_left:
            rovCommander.go((0,-float(values['Target-Y']),0))
        if event == sym_yaw_left:
            rovCommander.att_cmd((-float(values['DeltaYawD']),0,0))
        if event == sym_yaw_right:
            rovCommander.att_cmd((float(values['DeltaYawD']),0,0))
        if event in ['P+','P-','I+','I-','D-','D+']:
            plot_type=values['-PLOT-TYPE-']
            mul=float(values['PID_Mul'].strip())/100+1.0
            if event[1]=='-':
                mul=1/mul
            sg.cprint('sending pid',c='black on white')
            rovCommander.update_pid(plot_type,event[0],mul)
        if event=='SAVE_PID':
            plot_type=values['-PLOT-TYPE-']
            sg.cprint('sending saving pid',c='black on white')
            rovCommander.save_pid(plot_type)

        if (cnt%(1000//20))==0:
            rovCommander.heartbit()
        
        cnt+=1

        if values['-PLOT-TYPE-']=='DEPTH':
            plotter.update_pid(rovHandler.plot_buffers[zmq_topics.topic_depth_hold_pid])
        
        for i,p_type in [(0,'X_HOLD'),(1,'Y_HOLD')]:
            pb=rovHandler.plot_buffers[zmq_topics.topic_pos_hold_pid_fmt%i]
            if values['-PLOT-TYPE-']==p_type:
                plotter.update_pid(pb)
            target_xy[i]=pb.get_last('T')
            if target_xy[i] is None:
                target_xy[i]=0
        for p_type,pb in [\
                ('YAW',zmq_topics.topic_att_hold_yaw_pid),
                ('PITCH',zmq_topics.topic_att_hold_pitch_pid),
                ('ROLL',zmq_topics.topic_att_hold_roll_pid)]:
            if values['-PLOT-TYPE-']==p_type:
                plotter.update_pid(rovHandler.plot_buffers[pb])

        rovHandler.next()
        rov_telem=rovHandler.getTelemtry()
        if 'dvl_deadrecon' in rov_telem:
            trace_plotter.update_dvl_data(rov_telem['dvl_deadrecon'],target_pos=target_xy[::-1],yaw_deg=current_yaw_deg)
        if zmq_topics.topic_imu in rov_telem:
            current_yaw_deg = rov_telem[zmq_topics.topic_imu]['yaw']

    window.close()
if __name__ == "__main__":
    main()
