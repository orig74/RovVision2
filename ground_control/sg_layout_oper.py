import io
import os
import PySimpleGUI as sg

from screeninfo import get_monitors
#scale_screen=get_monitors()[0].width==1600
from PIL import Image,ImageTk
import config

import farm_track_sg as TrackThreadSG
import sg_symbols as syms
sg.theme('DarkGrey1')
#sg.set_options(button_color=('white', 'red'))
def get_main_image_sz(values):
    return (config.cam_main_sx*2,config.cam_main_sy*2) 

def get_main_annotation_image_key():
    return "-IMAGE-2-"

def _default_button_r(txt,cls=sg.Button,**kargs):
    xxx=sg.theme_button_color()
    return cls(txt,size=(12,2),border_width=3,disabled_button_color=None, highlight_colors=None, mouseover_colors=xxx,**kargs)
def _default_button_rh(txt,cls=sg.Button,**kargs):
    xxx=sg.theme_button_color()
    return cls(txt,size=(4,2),border_width=3,disabled_button_color=None, highlight_colors=None, mouseover_colors=xxx,**kargs)


def _default_button_b(txt,cls=sg.Button,**kargs):
    return cls(txt,size=(3,2),border_width=3,**kargs)


class ButtonCont(sg.Button):
    def __init__(_,t,**kargs):
        super(ButtonCont,_).__init__(t,**kargs)
        _.pressed = False

    def finalize(_):
        _.bind('<ButtonPress>', " Press", propagate=False)
        _.bind('<ButtonRelease>', " Release", propagate=False)

    def is_pressed(_,event):
        if _.key+" Press"==event:
            _.pressed=True
        if _.key+" Release"==event:
            _.pressed=False
        return _.pressed

    def is_released(_,event):
        return _.key+" Release"==event

class SGjoy(sg.Graph):
    def __init__(self,sx=155,sy=155,key='SGJOY'):
        super(SGjoy,self).__init__(canvas_size=(sx,sy), graph_bottom_left=(0, 0), graph_top_right=(sx,sy),
 background_color=sg.theme_button_color_background(), drag_submits=True,motion_events=False, enable_events=True,change_submits=True, key='SGJOY')
        self.circle=None
        self.sx,self.sy=sx,sy
        self._key=key
        self.x1=self.sx/2
        self.y1=self.sy/2
        self.start_pt=None

    def update_event(_,event,values):
        ret=None
        if _._key in event:
            x2,y2= values[_._key]
            if _.start_pt is None:
                _.start_pt=(x2,y2)
            if 'UP' in event:
                x2,y2=_.sx/2,_.sy/2
                _.start_pt=None
                ret=(0,0)
            _.MoveFigure(_.circle, x2-_.x1, y2-_.y1)
        else:
            x2,y2=_.x1,_.y1    

        _.x1,_.y1=x2,y2
        if _.start_pt is not None:
            ret=(x2-_.start_pt[0])/(_.sx/2),(y2-_.start_pt[1])/(_.sy/2)
        #if ret:
        #    print('joy ret is',ret)
        return ret

    def finalize(self):
        if self.circle is None:
            self.circle = self.draw_circle((self.sx/2,self.sy/2), 10, fill_color='black', line_color='white')


def get_layout(track_thread=None):
    im_size2 = (config.cam_main_sx*2,config.cam_main_sy*2)
    sgjoy=SGjoy();
    _UP=_default_button_r('UP',key='UP_CONT',cls=ButtonCont)
    _DOWN=_default_button_r('DOWN',key='DOWN_CONT',cls=ButtonCont)
    Gvl=_default_button_b(syms.sym_yaw_left,key='Gvl',cls=ButtonCont)
    Gvr=_default_button_b(syms.sym_yaw_right,key='Gvr',cls=ButtonCont)
    to_fin=[sgjoy,_UP,_DOWN,Gvr,Gvl]
    right_column = [
            [_default_button_r('Arm-Disarm')],
            [_default_button_r('Hold')],
            [_default_button_r('REC')],
            [sg.Radio('Scan Left', "SCAN_DIR", default=True)], [sg.Radio('Scan Right', "SCAN_DIR")],
            [sg.Checkbox('Mission\n Start',key='AUTO_NEXT',enable_events=True,tooltip='start stop mission',default=False)],
            [sg.Text('Mission State:')],[sg.Text('WAIT',key='MSTATE')],
            [sg.Text('',size=(3,10))], #place holder
            [_default_button_rh(syms.sym_yaw_left),_default_button_rh(syms.sym_yaw_right)],
            [sgjoy],
            [_UP],
            [_DOWN],
            ]

    row1_layout = [[
        sg.Graph(im_size2, graph_bottom_left=(0, im_size2[1]), graph_top_right=(im_size2[0],0) ,key="-IMAGE-2-",
            change_submits=True,
            enable_events=True,
            ),
        sg.Column(right_column,vertical_alignment='top')
        ],
        ]

    row2_layout = [
            [_default_button_b('Gc'),_default_button_b('Go'),
            Gvl,Gvr,
            sg.Multiline(key='MESSEGES',s=(23,2) , no_scrollbar=True, reroute_stdout=False, write_only=True),
            sg.Combo([f'{i/10}' for i in range(0,10)],key='D405EXP',s=(3,6),enable_events=True)
            ]]

    layout = [
        [sg.Frame('',row1_layout)],
        [sg.Frame('',row2_layout)],
    ]

    window = sg.Window("ROV Viewer", 
            layout, finalize=True, 
            no_titlebar=False,#scale_screen,
            element_justification='left', 
            font='Helvetica 15',
            size=(1920,1080))
    for fn in to_fin:
        fn.finalize()
    #if scale_screen:
    #    window.Maximize()

    return window
