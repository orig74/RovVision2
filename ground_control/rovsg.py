# image_viewer.py
import os
import PySimpleGUI as sg
from PIL import Image,ImageTk
import time

import sys
import traceback
import pickle

sys.path.append('../onboard')
sys.path.append('../hw')
sys.path.append('../utils')
sys.path.append('..')

import config
from annotations import draw_main
import numpy as np
import cv2
import argparse

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
param_file='param_file.pkl'
#scale_screen=None

from sg_layout_eng import get_layout
import sg_symbols as syms

def save_sg_state(window):
    d=window.AllKeysDict
    to_save={}
    for k in d:
        if type(window[k]) in [sg.Input,sg.Checkbox]:
            to_save[k]=window[k].get()
    with open(param_file,'wb') as fd:
        pickle.dump(to_save,fd,protocol=0)

params_file_data=None
def load_sg_state(window):
    global params_file_data
    if os.path.isfile(param_file):
        di=pickle.load(open(param_file,'rb'))
        d=window.AllKeysDict
        for k in d:
            if k in di and type(window[k]) in [sg.Input,sg.Checkbox]:
                window[k](di[k])
        params_file_data=di

def update_default_sg_values(vals):
    if params_file_data is not None:
        for k in params_file_data:
            if k not in vals:
                vals[k]=params_file_data[k]


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

def printer(text,color=None):
    print('printer:',text)
    sg.cprint(text,c='black on white')

def update_values_from_file(values):
    if os.path.isfile(track_thread_file):
        #track_thread.load_params(track_thread_file)
        import json
        js=json.loads(open(track_thread_file,'rb').read().strip())
        for key in js:
            values[key]=js[key]

def main():
    rovHandler = rovDataHandler(None,printer=printer,args=args)
    rovCommander = rovCommandHandler()
    track_thread = TrackThread(rov_comander=rovCommander,rov_data_handler=rovHandler,printer=printer)

    last_heartbit=time.time()
    last_im=None
    image_shape=None 
    cnt=0

    current_yaw_deg=0
    target_xy=[0,0]

    last_plot_pids=time.time()
    last_plot_dvl =time.time()
    window = get_layout(track_thread)
    load_sg_state(window)
    sg.cprint_set_output_destination(window, 'MESSEGES')
    plotter = Plotter(window["-CANVAS-"].TKCanvas)
    trace_plotter = TracePlotter(window["-TRACE-CANVAS-"].TKCanvas)
    #import ipdb;ipdb.set_trace()

    while True:
        try:
            cycle_tic=time.time()
            event, values = window.read(timeout=2) #10 mili timeout

            update_default_sg_values(values) #diffrent layouts might not have the defaults values as inputs

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
                last_im=draw_image(window["-IMAGE-0-"],img_to_tk(rawImg))
            frameId, rawImg = rovHandler.getSincedImages(1)
            if rawImg is not None:
                window["-IMAGE-1-"].update(data=img_to_tk(rawImg,1.65 if values['LAYOUT2'] else 1))
            main_image = rovHandler.getMainImage()
            if main_image is not None:
                if not values['LAYOUT2']:
                    main_image=cv2.resize(main_image,(config.cam_main_gui_sx,config.cam_main_gui_sy),cv2.INTER_NEAREST) 
                window["-IMAGE-2-"].erase()
                tr_main=rovHandler.get_main_track_pt()
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
            if event == "Arm-Disarm":
                rovCommander.armdisarm()
            if event == "Depth-Hold":
                rovCommander.depth_hold()
            if event == syms.sym_down:
                rovCommander.depth_command(float(values['Target-Depth']))
            if event == syms.sym_up:
                rovCommander.depth_command(-float(values['Target-Depth']))
            if event == 'Att-hold':
                rovCommander.att_hold()
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
            if event == syms.sym_fwd:
                rovCommander.go((float(values['Target-X']),0,0))
            if event == syms.sym_back:
                rovCommander.go((-float(values['Target-X']),0,0))
            if event == syms.sym_right:
                rovCommander.go((0,float(values['Target-Y']),0))
            if event == syms.sym_left:
                rovCommander.go((0,-float(values['Target-Y']),0))
            if event == syms.sym_yaw_left:
                rovCommander.att_cmd((-float(values['DeltaYawD']),0,0))
            if event == syms.sym_yaw_right:
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

            if time.time()-last_heartbit>2.0:
                last_heartbit=time.time()
                rovCommander.heartbit()

            if event=='MISSION_SAVE':
                #track_thread.set_params(TrackThreadSG.get_layout_values(values))
                #track_thread.save_params(track_thread_file)
                #import json
                #js=json.dumps({k:values[k] for k in values.keys()},indent=4)
                #open(track_thread_file,'wb').write(js.encode())
                save_sg_state(window)

            if event=='AUTO_NEXT':
                track_thread.auto_next=values['AUTO_NEXT']
                track_thread.set_params(TrackThreadSG.get_layout_values(values))
                track_thread.start()
                printer(f'set auto next to {track_thread.auto_next}')

            if values['AUTO_NEXT']:
                track_thread.run(float(values['Lrange']),float(values['k_max_alt']),
                        Pxy=(float(values['Px']),float(values['Py'])))

            if 1:
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
                window['X_LOCK'](False)
                window['Y_LOCK'](False)
                window['D_LOCK'](False)
                rovCommander.x_lock(False)
                rovCommander.y_lock(False)
                rovCommander.d_lock(False)
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
        except Exception as E:
            print('*'*100)
            traceback.print_exc(file=sys.stdout)
            time.sleep(0.1)

    window.close()
if __name__ == "__main__":
    main()
