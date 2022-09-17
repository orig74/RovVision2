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
import zmq_topics


def img_to_tk(img,shrink=1):
    if shrink==1:
        img = Image.fromarray(img)
    else:
        img = Image.fromarray(img[::shrink,::shrink])
    img = ImageTk.PhotoImage(img)
    return img
#def img_to_tk2(img,shrink=1):
#    if shrink==1:
#        img = Image.fromarray(img)
#    else:
#        img = Image.fromarray(img[::shrink,::shrink])
#    bio = io.BytesIO()
#    img.save(bio,format='PNG')
#    return bio.getvalue()
#def img_to_tk3(img,shrink=1):
#    if shrink>1:
#        img = img[::shrink,::shrink]
#    return cv2.imencode('.png',img)[1].tobytes()
#def img_to_tk4(img,shrink=1):
#    if shrink==1:
#        img = Image.fromarray(img)
#    else:
#        img = Image.fromarray(img[::shrink,::shrink])
#    return img

def draw_image(self, image):
    id = self._TKCanvas2.create_image((image.width()//2,image.height()//2), image=image)
    self.Images[id] = image
    return id

def main():
    rovHandler = rovDataHandler(None)
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
            [sg.Button('Depth-Hold'),sg.Button('D+'),sg.Button('D-'),sg.Input(key='Target-Depth',size=(4,1))],
            [sg.Button('Att-hold')],
            [sg.Button('X-hold'),sg.Button('X+'),sg.Button('X-'),sg.Input(key='Target-X',size=(4,1))],
            [sg.Button('Y-hold'),sg.Button('Y+'),sg.Button('Y-'),sg.Input(key='Target-Y',size=(4,1))],
            [sg.Button('Yaw+'),sg.Button('Yaw-')],
            ]

    yaw_source_options=['VNAV','DVL']
    config_column = [
            [sg.Text('Yaw Source:'),sg.Combo(yaw_source_options,key='YAW_SOURCE',default_value=yaw_source_options[0])]
            ]

    plot_options=['DEPTH','YAW','PITCH','ROLL','RANGE']
    matplot_column = [
        [sg.Text('Plot Type:'),sg.Combo(plot_options,key='-PLOT-TYPE-',default_value=plot_options[0])],
        [ sg.Canvas(key="-CANVAS-", size=(500,500))]]
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
            font='Helvetica 16',
            size=(1920,1280))
    #window['-MAIN-IMAGE-'].bind('<Button-1>','')
    plotter = Plotter(window["-CANVAS-"].TKCanvas)
    
    last_im=None
    image_shape=None 
    while True:
        event, values = window.read(timeout=20) #10 mili timeout
        #print('----------',event,values)
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
            last_im=draw_image(window["-MAIN-IMAGE-"],img_to_tk(rawImgs[0]))#im_size[1]))
            window["-IMAGE-1-"].update(data=img_to_tk(rawImgs[1],1))
 
        if event == "Arm-Disarm":
            #filename = values["-FILE-"]
            rovCommander.armdisarm()

        plotter.update_pid(rovHandler.plot_buffers[zmq_topics.topic_depth_hold_pid])

        rovHandler.next()


    window.close()
if __name__ == "__main__":
    main()
