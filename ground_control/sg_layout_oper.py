import io
import os
import PySimpleGUI as sg
from screeninfo import get_monitors
scale_screen=get_monitors()[0].width==1600
from PIL import Image,ImageTk
import config

import farm_track_sg as TrackThreadSG
import sg_symbols as syms
sg.theme('DarkGrey1')

def get_layout(track_thread=None):
    im_size2 = (config.cam_main_sx*2,config.cam_main_sy*2)
    right_column = [
            [sg.Button('Arm-Disarm',size=(16,2),border_width=3)],
            [sg.Button('Hold')],
            [sg.Checkbox('Mission Start',key='AUTO_NEXT',enable_events=True,tooltip='start stop mission',default=False)],
            [sg.Text('MState:'),sg.Text('WAIT',key='MSTATE')],
        ]

    row1_layout = [[
        sg.Graph(im_size2, graph_bottom_left=(0, im_size2[1]), graph_top_right=(im_size2[0],0) ,key="-IMAGE-2-",
            change_submits=True,
            enable_events=True,
            ),
        sg.Column(right_column,vertical_alignment='top')
        ],
        ]

    cmd_column = []
    cmd_column+=[
            [
            sg.Input(key='D405EXP',default_text='0',size=(4,1),enable_events=True,
                tooltip='d405 exposure milis')
            ] 
            ]
    row2_layout = [
            [sg.Button('Gc',tooltip="gripper close"),sg.Button('Go',tooltip="gripper open"),sg.Button('Tx',tooltip='stop main tracker'),
            sg.Button('REC'),sg.Button('Reset-DVL'),sg.Button('Calib-DVL'),sg.Checkbox('L2',key='LAYOUT2',tooltip='layout2'),
            #[sg.Button('CF+'),sg.Button('CF-'),sg.Button('Lights+'),sg.Button('Lights-')],
            sg.Multiline(key='MESSEGES',s=(23,2) , autoscroll=True, reroute_stdout=False, write_only=True),
            ]]

    layout = [
        [sg.Frame('',row1_layout)],
        [sg.Frame('',row2_layout)],
    ]

    window = sg.Window("ROV Viewer", 
            layout, finalize=True, 
            no_titlebar=False,#scale_screen,
            element_justification='left', 
            font='Helvetica 9' if scale_screen else 'Helvetica 10',
            size=(1600,900) if scale_screen else (1920,1080))
    if scale_screen:
        window.Maximize()

    return window
