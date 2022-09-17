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


def main():
    rovHandler = rovDataHandler(None)
    rovCommander = rovCommandHandler()
    layout = [
        [sg.Image(key="-MAIN-IMAGE-"),
            sg.Image(key="-IMAGE-1-")],#,sg.Image(key="-IMAGE-2-")],
        [sg.Canvas(key="-CANVAS-")],

        [
            sg.Text("Image File"),
            sg.Input(size=(25, 1), key="-FILE-"),
            sg.Button("Arm-Disarm"),
        ],
    ]
    window = sg.Window("ROV Viewer", 
            layout, finalize=True, 
            element_justification='left', 
            font='Helvetica 16',
            size=(1920,1280))
    plotter = Plotter(window["-CANVAS-"].TKCanvas)
    
    while True:
        event, values = window.read(timeout=20) #10 mili timeout
        if event == "Exit" or event == sg.WIN_CLOSED:
            break

        frameId, rawImgs = rovHandler.getNewImages()

        if rawImgs is not None:
            #print('===',time.time(),rawImgs[0].shape)
            window["-MAIN-IMAGE-"].update(data=img_to_tk(rawImgs[0]))
            #window["-IMAGE-1-"].update(data=img_to_tk(rawImgs[0]))
            window["-IMAGE-1-"].update(data=img_to_tk(rawImgs[1],1))
 
        if event == "Arm-Disarm":
            #filename = values["-FILE-"]
            rovCommander.armdisarm()

        plotter.update_pid(rovHandler.plot_buffers[zmq_topics.topic_depth_hold_pid])

        rovHandler.next()


    window.close()
if __name__ == "__main__":
    main()
