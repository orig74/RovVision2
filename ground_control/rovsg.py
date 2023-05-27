# image_viewer.py
import io
import os
import PySimpleGUI as sg
from PIL import Image,ImageTk
import time

import sys
import socket
import pickle
import json
import traceback

from screeninfo import get_monitors

sys.path.append('../onboard')
sys.path.append('../hw')
sys.path.append('../utils')
sys.path.append('..')

import config
from annotations import draw_mono,draw_main
import numpy as np
import cv2
from select import select
import zmq
import image_enc_dec

import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--data_path", help="path for data", default='../../data')
parser.add_argument("--pub_data", help="publish data", action='store_true')
parser.add_argument("--depth", help="Display Depth", action='store_true')
args = parser.parse_args()


from rov_data_handler import rovDataHandler,rovCommandHandler
from plttk import Plotter
from plttk_tracer import Plotter as TracePlotter

import zmq_topics
from farm_track_thread import FarmTrack as TrackThread
import farm_track_sg as TrackThreadSG
track_thread_file='farm_track_params.json'
scale_screen=get_monitors()[0].width==1600
#scale_screen=None


def img_to_tk(img,shrink=1,h_hsv=False):
    if h_hsv:
        img=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:,:,0]
    if shrink==1:
        img = Image.fromarray(img)
    else:
        #img = Image.fromarray(img[::shrink,::shrink])
        img=cv2.resize(img,(int(img.shape[1]/shrink),int(img.shape[0]/shrink)),cv2.INTER_NEAREST) 
        img = Image.fromarray(img)
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
    rovHandler = rovDataHandler(None,printer=printer,args=args)
    rovCommander = rovCommandHandler()
    track_thread = TrackThread(rov_comander=rovCommander,rov_data_handler=rovHandler,printer=printer)

    if os.path.isfile(track_thread_file):
        track_thread.load_params(track_thread_file)


    last_heartbit=time.time()
    #im_size = (960,600) 
    im_size = (616,514)
    im_size2 = (config.cam_main_sx,config.cam_main_sy)
    row1_layout = [[
        sg.Graph(im_size, graph_bottom_left=(0, im_size[1]), graph_top_right=(im_size[0],0) ,key="-IMAGE-0-",
            change_submits=True,
            enable_events=True,
            ),
        sg.Image(key="-IMAGE-1-"),#,sg.Image(key="-IMAGE-2-")],
        sg.Graph(im_size2, graph_bottom_left=(0, im_size2[1]), graph_top_right=(im_size2[0],0) ,key="-IMAGE-2-",
            change_submits=True,
            enable_events=True,
            ),
        ]]

    cmd_column = [
            [sg.Button('Arm-Disarm'),
                sg.Combo([f'{i/10}' for i in range(1,10)],key='MANUAL_LIMIT',
                default_value=str(config.manual_control_limit),enable_events=True,
                    tooltip='set manual control limit'),
                sg.Combo([f'{i/10}' for i in range(1,10)],key='THRUSTER_LIMIT',
                default_value=str(config.thruster_limit_controler),enable_events=True,
                    tooltip='set thruster limit for all thrusters')],
             [sg.Button('Depth-Hold'),
                sg.Button(sym_up),sg.Button(sym_down),sg.Input(key='Target-Depth',default_text='0.1',size=(4,1)),
                sg.Checkbox('L',key='D_LOCK',enable_events=True,tooltip='depth main tracker lock')],

            [sg.Button('Att-hold'),sg.Text('Pitch:'),sg.Input(key='Target-Pitch',default_text='0.0',size=(4,1))],

            [sg.Button('X-hold'),sg.Button(sym_fwd),sg.Button(sym_back),sg.Input(key='Target-X',default_text='0.1',size=(4,1)),
                sg.Checkbox('L',key='X_LOCK',enable_events=True,tooltip='x (range) main tracker lock')],

            [sg.Button('Y-hold'),sg.Button(sym_left),sg.Button(sym_right),sg.Input(key='Target-Y',default_text='0.1',size=(4,1)),
                sg.Checkbox('L',key='Y_LOCK',enable_events=True,tooltip='y (sideways main tracker lock')],

            [sg.Button(sym_yaw_left),sg.Button(sym_yaw_right),sg.Input(key='DeltaYawD',default_text='1.0',size=(4,1))],
            [
                sg.Checkbox('V',key='V_LOCK',enable_events=True,tooltip='vertical object lock'),
                sg.Text('Range:'),
                sg.Input(key='Lrange',default_text='0.35',size=(4,1)),
                sg.Text('Pxy:'),
                sg.Input(key='Px',default_text='0.01',enable_events=True,size=(5,1)),
                sg.Input(key='Py',default_text='0.01',enable_events=True,size=(5,1))
                ],
            [ 
                sg.Button('Ml',tooltip='tracker max lock'),
                sg.Button('Ms',tooltip='mission start'),
                sg.Checkbox('Ma',key='AUTO_NEXT',enable_events=True,tooltip='auto next',default=False),
                sg.Button('Mn',tooltip='mission next'),
                sg.Checkbox('Mp',key='MISSION_PAUSE',enable_events=False,tooltip='pause mission',default=True)
                ],
            [sg.Text('MState:'),sg.Text('WAIT',key='MSTATE')],
            ]
    cmd_column+=TrackThreadSG.get_layout(track_thread)
    cmd_column+=[[sg.Button('Save',key='MISSION_SAVE',tooltip='save mission params')]]
    cmd_column+=[
            #[sg.Checkbox('H',key='HSV_H',tooltip='h from hsv on cam 1')],
            [
            #sg.Combo(list('RGBrgb'),key='CHANNEL',enable_events=True ,default_value='B',
            #        tooltip='detect rope from grey image channel'),
            #sg.Combo(list('01'),key='ROPE_TO_HSV',default_value='0',enable_events=True,
            #        tooltip='detect rope from hsv image channel (h) or in h'),
            #sg.Combo(list('123'),key='KEEP_STROBE_MODE',enable_events=True ,default_value='N',
            #        tooltip='select 1-keep strobe 2-keep dark 3-keep all'),
            sg.Input(key='D405EXP',default_text='0',size=(4,1),enable_events=True,
                tooltip='d405 exposure milis')
            ] 
            ]
    #yaw_source_options=['VNAV','DVL']
    config_column = [
            [sg.Image(key="-IMAGE-2D-")],
            [sg.Button('Gc',tooltip="gripper close"),sg.Button('Go',tooltip="gripper open"),sg.Button('Tx',tooltip='stop main tracker')],
            [sg.Button('REC'),sg.Button('Reset-DVL'),sg.Button('Calib-DVL'),sg.Checkbox('L2',key='LAYOUT2',tooltip='layout2')],
            #[sg.Button('CF+'),sg.Button('CF-'),sg.Button('Lights+'),sg.Button('Lights-')],
            [sg.Multiline(key='MESSEGES',s=(23,5) if scale_screen else (55,8), autoscroll=True, reroute_stdout=False, write_only=True)],
           ]
            #sg.Button('RTHSV',key='ROPE_TO_HSV',tooltip='detect rope from hsv image channel (h)')],

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
        #[ sg.Canvas(key="-CANVAS-", size=(300,200)), sg.Canvas(key="-TRACE-CANVAS-", size=(300,300))]]
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
            #size=(1600,900))
    #window['-IMAGE-0-'].bind('<Button-1>','')
    plotter = Plotter(window["-CANVAS-"].TKCanvas)
    trace_plotter = TracePlotter(window["-TRACE-CANVAS-"].TKCanvas)
    
    last_im=None
    image_shape=None 
    cnt=0

    current_yaw_deg=0
    target_xy=[0,0]
    sg.cprint_set_output_destination(window, 'MESSEGES')

    last_plot_pids=time.time()
    last_plot_dvl =time.time()
    while True:
        try:
            cycle_tic=time.time()
            event, values = window.read(timeout=2) #10 mili timeout
            main_image_size=(config.cam_main_sx,config.cam_main_sy) if values['LAYOUT2'] else (config.cam_main_gui_sx,config.cam_main_gui_sy)
            if event == "Exit" or event == sg.WIN_CLOSED:
                break
            
            rovHandler.dump_gui_event([event,values])

            scaley=None if values['AUTOSCALEY'] else 1.0
            if image_shape is not None and event.startswith('-IMAGE-0'):
                x,y=values['-IMAGE-0-']
                x=x/image_shape[1]
                y=y/image_shape[0]
                print('---click--',x,y)
                rovCommander.lock(x,y)
            
            if event.startswith('-IMAGE-2'):
                x,y=values['-IMAGE-2-']
                x=x/main_image_size[0]
                y=y/main_image_size[1]
                printer(f'click2,{x},{y}')
                rovCommander.main_track((x,y))
            
            _, rawImg = rovHandler.getSincedImages(0)
            if rawImg is not None:
                image_shape=rawImg.shape
                if last_im is not None:
                    window["-IMAGE-0-"].erase()
                last_im=draw_image(window["-IMAGE-0-"],img_to_tk(rawImg))#,h_hsv=values['HSV_H']))#im_size[1]))
            frameId, rawImg = rovHandler.getSincedImages(1)
            if rawImg is not None:
                window["-IMAGE-1-"].update(data=img_to_tk(rawImg,1.65 if values['LAYOUT2'] else 1))
            main_image = rovHandler.getMainImage()
            if main_image is not None:
                #if config.cam_main_gui_sx!=config.cam_main_sx:
                if not values['LAYOUT2']:
                    main_image=cv2.resize(main_image,(config.cam_main_gui_sx,config.cam_main_gui_sy),cv2.INTER_NEAREST) 
                window["-IMAGE-2-"].erase()
                tr_main=rovHandler.get_main_track_pt()
                #print('tr_main is',tr_main)
                grng,gleft,gup=config.grip_pos_rel_mm
                uv=np.array(config.cam_main_int) @ np.array([[gleft,gup,grng]]).T
                uv=(uv/uv[2,0]).flatten()[:2]*((main_image_size[0]/config.cam_main_sx),(main_image_size[1]/config.cam_main_sy))
                cv2.circle(main_image,(int(uv[0]),int(uv[1])),4,(0,255,0),1)
                draw_main(main_image,rovHandler)

                if tr_main and tr_main['xy'] is not None:
                    x,y=tr_main['xy']
                    x*=main_image_size[0]
                    y*=main_image_size[1]
                    cv2.circle(main_image,(int(x),int(y)),4,(255,0,0),1)
                    if tr_main['range'] is not None:
                        rng=tr_main['range']
                        left=tr_main['left']
                        up=tr_main['up']
                        cv2.putText(main_image, f'rng{rng-grng:04.1f} up{up-gup:04.1f} left{left-gleft:04.1f} mm', (50,23), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
                draw_image(window["-IMAGE-2-"],img_to_tk(main_image,1))#im_size[1]))
            main_image_depth = rovHandler.getMainImageDepth()
            if main_image_depth is not None:
                window["-IMAGE-2D-"].update(data=img_to_tk(main_image_depth,1))
                #window["-IMAGE-2-"].update(data=img_to_tk(main_image,1))
            
                #print(f'=== tk images took {(time.time()-tic1)*1000:.1f} msec, grab  {(time.time()-tic2)*1000:.1f} msec')
     
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
            #if event == 'CF+':
            #    rovCommander.clear_freqs(1)
            #if event == 'CF-':
            #    rovCommander.clear_freqs(-1)
            if event == 'X-hold':
                rovCommander.x_hold()
            if event == 'Y-hold':
                rovCommander.y_hold()
            if event == 'Z-hold':
                rovCommander.z_hold()
            if event == 'X_LOCK':
                rovCommander.x_lock(values['X_LOCK'])
            if event == 'Y_LOCK':
                rovCommander.y_lock(values['Y_LOCK'])
            if event == 'D_LOCK':
                rovCommander.d_lock(values['D_LOCK'])
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
            if event == 'MANUAL_LIMIT':
                rovCommander.set_manual_control_limit(float(values['MANUAL_LIMIT']))
            if event == 'THRUSTER_LIMIT':
                rovCommander.set_thruster_limit(float(values['THRUSTER_LIMIT']))

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

            if event=='V_LOCK':
                printer(f"got v_lock {values['V_LOCK']}")
                if values['V_LOCK']:
                    rovCommander.vertical_object_lock(rng=float(values['Lrange']),
                            Pxy=(float(values['Px']),float(values['Py']))
                            )
                else:
                    rovCommander.vertical_object_unlock()

            if event in ['Px','Py']:
                rovCommander.update_pxy((float(values['Px']),float(values['Py'])))

            window['V_LOCK'](rovCommander.vertical_object_lock_state)

            if event=='Ml':
                rovCommander.lock_max()

            #if (cnt%(1000//20))==0:
            if time.time()-last_heartbit>2.0:
                last_heartbit=time.time()
                rovCommander.heartbit()

            if event=='Ms':
                track_thread.set_params(TrackThreadSG.get_layout_values(values))
                track_thread.start()

            if event=='MISSION_SAVE':
                track_thread.set_params(TrackThreadSG.get_layout_values(values))
                track_thread.save_params(track_thread_file)

            if event=='AUTO_NEXT':
                track_thread.auto_next=values['AUTO_NEXT']
                printer(f'set auto next to {track_thread.auto_next}')

            if event=='Mn':
                track_thread.do_next()

            if not values['MISSION_PAUSE']:
                track_thread.run(float(values['Lrange']),float(values['Pxy']))
                window['MSTATE'](track_thread.get_state(),text_color='white',background_color='black')

            if event=='CENTER_TRACE':
                trace_plotter.center()

            if event=='CLEAR_TRACE':
                trace_plotter.clear_trace()

            if event=='PLOTER_RAD':
                trace_plotter.update_rad(float(values['PLOTER_RAD']))

            if event=='Reset-DVL':
                rovCommander.reset_dvl()

            if event=='Calib-DVL':
                rovCommander.calib_dvl()

            if event=='Lights+':
                rovCommander.lights_inc()

            if event=='Lights-':
                rovCommander.lights_dec()

            if event=='REC':
                rovHandler.toggle_recording()

            if event=='Go':
                rovCommander.set_gripper(0.0)

            if event=='Gc':
                rovCommander.set_gripper(1.0)

            if event=='Tx':
                rovCommander.main_track(None)

            if values['-PLOT-TYPE-']=='DEPTH':
                plotter.update_pid(rovHandler.plot_buffers[zmq_topics.topic_depth_hold_pid],ylim=scaley)

            if event=='CHANNEL':
                rovCommander.set_rope_tracker_to_grey(chan='RGBrgb'.index(values['CHANNEL']))
            if event=='ROPE_TO_HSV':
                rovCommander.set_rope_tracker_to_hsv(int(values['ROPE_TO_HSV']))
            if event=='KEEP_STROBE_MODE':
                rovCommander.set_strob_mode(int(values['KEEP_STROBE_MODE']))
            if event=='D405EXP':
                rovCommander.set_exposure_d405(int(values['D405EXP']))
            
            for i,p_type in [(0,'X_HOLD'),(1,'Y_HOLD')]:
                pb=rovHandler.plot_buffers[zmq_topics.topic_pos_hold_pid_fmt%i]
                if values['-PLOT-TYPE-']==p_type:
                    plotter.update_pid(pb,ylim=scaley)
                target_xy[i]=pb.get_last('T')
                if target_xy[i] is None:
                    target_xy[i]=0
            tic=time.time()
            if time.time()-last_plot_pids>0.3:
                last_plot_pids=time.time()
                for p_type,pb in [\
                        ('YAW',zmq_topics.topic_att_hold_yaw_pid),
                        ('PITCH',zmq_topics.topic_att_hold_pitch_pid),
                        ('ROLL',zmq_topics.topic_att_hold_roll_pid)]:
                    if values['-PLOT-TYPE-']==p_type:
                        plotter.update_pid(rovHandler.plot_buffers[pb],ylim=scaley)

            rovHandler.next()

            rov_telem=rovHandler.getTelemtry()
            #if 'dvl_deadrecon' in rov_telem :
            #    trace_plotter.update_dvl_data(rov_telem['dvl_deadrecon'],target_pos=target_xy[::-1],yaw_deg=current_yaw_deg)
            trace_plotter.update_pos_data(rovHandler.get_pos_xy2(),rovHandler.get_target_xy(),current_yaw_deg)
            if time.time()-last_plot_dvl>0.3:
                last_plot_dvl=time.time()
                trace_plotter.redraw()

            if zmq_topics.topic_imu in rov_telem:
                current_yaw_deg = rov_telem[zmq_topics.topic_imu]['yaw']
            if 0:
                print(f'=== took {(time.time()-tic)*1000:.1f} msec')

            cnt+=1
            if 0:
                print(f'=== tk images took {(time.time()-cycle_tic)*1000:.1f} msec')
            #time.sleep(0.001)
        except Exception as E:
            print('*'*100)
            traceback.print_exc(file=sys.stdout)
            time.sleep(0.1)

    window.close()
if __name__ == "__main__":
    main()
