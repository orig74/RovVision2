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
    im_size = (616,514)
    im_size2 = (config.cam_main_sx,config.cam_main_sy)
    row1_layout = [[
        sg.Graph(im_size2, graph_bottom_left=(0, im_size2[1]), graph_top_right=(im_size2[0],0) ,key="-IMAGE-2-",
            change_submits=True,
            enable_events=True,
            ),
        ]]

    cmd_column = [
            [sg.Button('Arm-Disarm')],
            [sg.Button('Hold')],

            [ 
                sg.Checkbox('Mission Start',key='AUTO_NEXT',enable_events=True,tooltip='start stop mission',default=False),
                ],
            [sg.Text('MState:'),sg.Text('WAIT',key='MSTATE')],
            ]
    cmd_column+=TrackThreadSG.get_layout(track_thread)
    cmd_column+=[[sg.Button('Save',key='MISSION_SAVE',tooltip='save mission params')]]
    cmd_column+=[
            [
            sg.Input(key='D405EXP',default_text='0',size=(4,1),enable_events=True,
                tooltip='d405 exposure milis')
            ] 
            ]
    config_column = [
            [sg.Image(key="-IMAGE-2D-")],
            [sg.Button('Gc',tooltip="gripper close"),sg.Button('Go',tooltip="gripper open"),sg.Button('Tx',tooltip='stop main tracker')],
            [sg.Button('REC'),sg.Button('Reset-DVL'),sg.Button('Calib-DVL'),sg.Checkbox('L2',key='LAYOUT2',tooltip='layout2')],
            #[sg.Button('CF+'),sg.Button('CF-'),sg.Button('Lights+'),sg.Button('Lights-')],
            [sg.Multiline(key='MESSEGES',s=(23,5) if scale_screen else (55,8), autoscroll=True, reroute_stdout=False, write_only=True)],
           ]
    plot_options=['DEPTH','X_HOLD','Y_HOLD','YAW','PITCH','ROLL']
    matplot_column1 = [
        [sg.Text('PType:'),sg.Combo(plot_options,key='-PLOT-TYPE-',default_value=plot_options[0]),
            sg.Button('P+'),sg.Button('P-'),
            sg.Button('I+'),sg.Button('I-'),
            sg.Button('D+'),sg.Button('D-'),
            sg.Text('Prc:'),sg.Input(key='PID_Mul',default_text='3.0',size=(4,1)),
            sg.Button('S',key='SAVE_PID'),
            sg.Checkbox('A',key='AUTOSCALEY',tooltip='auto scale y plot'),
            ],
        [sg.Canvas(key="-CANVAS-", size=(300,300))]
        ]

    matplot_column2 = [
        [sg.Button('cnt',key='CENTER_TRACE',tooltip='center trace'),sg.Button('clr',key='CLEAR_TRACE',tooltip='clear trace'),sg.Combo([f'{i}' for i in range(1,10)],key='PLOTER_RAD',enable_events=True,tooltip='map radius in meters')],
        [sg.Canvas(key="-TRACE-CANVAS-", size=(300,300))]
        ]

    row2_layout = [[
            #sg.Canvas(key="-CANVAS-", size=(500,500)),
            sg.Column(matplot_column1),
            sg.VSeperator(),
            sg.Column(matplot_column2),
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
            no_titlebar=False,#scale_screen,
            element_justification='left', 
            font='Helvetica 9' if scale_screen else 'Helvetica 10',
            size=(1600,900) if scale_screen else (1920,1080))
    if scale_screen:
        window.Maximize()

    return window
