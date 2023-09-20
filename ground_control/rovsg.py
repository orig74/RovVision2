# image_viewer.py
import os
import PySimpleGUI as sg
print('pysimplegui version',sg.__version__)
import time

import sys
import traceback

sys.path.append('../onboard')
sys.path.append('../hw')
sys.path.append('../utils')
sys.path.append('..')

import config
from annotations import draw_main
import numpy as np
import cv2
import argparse
import sg_utils

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
#scale_screen=None

from sg_dive_form import DiveForm
if os.environ.get('SG_LAYOUT','ENG')=='ENG':
    import sg_layout_eng as sg_layout
else:
    import sg_layout_oper as sg_layout


import sg_symbols as syms

def printer(text,color=None):
    print('printer:',text)
    sg.cprint(text,c='black on white')

def main():
    rovHandler = rovDataHandler(None,printer=printer,args=args)
    rovCommander = rovCommandHandler()
    track_thread = TrackThread(rov_comander=rovCommander,rov_data_handler=rovHandler,printer=rovHandler.printer)
    sg_utils.set_rovvision_config_path()
    dive_form=DiveForm(sg_utils.dive_form_path)
    dive_form.open()
    rovHandler.update_gui_data('dive_form',dive_form.get())

    last_heartbit=time.time()
    last_im=None
    image_shape=None 
    cnt=0

    initialize_data_sent=False

    current_yaw_deg=0
    target_xy=[0,0]

    last_plot_pids=time.time()
    last_plot_dvl =time.time()
    window = sg_layout.get_layout(track_thread)
    sg_utils.load_sg_state(window)
    sg.cprint_set_output_destination(window, 'MESSEGES')
    plotter=None
    if '-CANVAS-' in window.AllKeysDict:
        plotter = Plotter(window["-CANVAS-"].TKCanvas)
    trace_plotter=None
    if '-TRACE-CANVAS-' in window.AllKeysDict:
        trace_plotter = TracePlotter(window["-TRACE-CANVAS-"].TKCanvas)

    #import ipdb;ipdb.set_trace()

    while True:
        try:
            cycle_tic=time.time()
            event, values = window.read(timeout=2) # mili timeout
            if event != '__TIMEOUT__':
                print('event is: ',event)
                rovHandler.dump_gui_event([event,values])

            sg_utils.update_default_sg_values(values) #diffrent layouts might not have the defaults values as inputs

            main_image_size=sg_layout.get_main_image_sz(values)
            if event == "Exit" or event == sg.WIN_CLOSED:
                break

            scaley=None if values['AUTOSCALEY'] else 1.0
            if image_shape is not None and event.startswith('-IMAGE-0'):
                x,y=values['-IMAGE-0-']
                x=x/image_shape[1]
                y=y/image_shape[0]
                print('---click--',x,y)
                rovCommander.lock(x,y)

            if 'SGJOY' in window.AllKeysDict:
                ret=window['SGJOY'].update_event(event,values)
                if ret is not None:
                    dx,dy=ret
                    if dx==0 and dy==0: #reset movment
                        rovCommander.reset_dvl()
                    else:
                        rovCommander.go((dy/50*float(values['Target-Y']),dx/50*float(values['Target-X']),0))

            if event.startswith('-IMAGE-2'):
                x,y=values['-IMAGE-2-']
                x=x/main_image_size[0]
                y=y/main_image_size[1]
                printer(f'click2,{x},{y}')
                rovCommander.main_track((x,y))

            _, rawImg = rovHandler.getSincedImages(0)
            if rawImg is not None and '-IMAGE-0-' in window.AllKeysDict:
                itic=time.time()
                image_shape=rawImg.shape
                if last_im is not None:
                    window["-IMAGE-0-"].erase()
                if sg_layout.get_main_annotation_image_key()=="-IMAGE-0-":
                    rovHandler.draw_main_annotations(rawImg)
                last_im=sg_utils.draw_image(window["-IMAGE-0-"],sg_utils.img_to_tk(rawImg))
                #print('image 2 took',time.time()-itic)
            frameId, rawImg = rovHandler.getSincedImages(1)

            if rawImg is not None and '-IMAGE-1-' in window.AllKeysDict:
                itic=time.time()
                window["-IMAGE-1-"].update(data=sg_utils.img_to_tk(rawImg,sg_layout.get_image_shrink("-IMAGE-1-",values)))
                #print('image 1 took',time.time()-itic)

            main_image = rovHandler.getMainImage()
            if main_image is not None and '-IMAGE-2-' in window.AllKeysDict:
                new_size = sg_layout.get_main_image_sz(values)
                if sg_layout.get_main_annotation_image_key()=="-IMAGE-2-":
                    rovHandler.draw_main_annotations(main_image)
                if new_size !=main_image.size:
                    main_image=cv2.resize(main_image,new_size,cv2.INTER_NEAREST) 
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
                        cv2.putText(main_image, f'rng{rng-grng:04.1f} up{up-gup:04.1f} left{left-gleft:04.1f} mm', (50,23), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3, cv2.LINE_AA)
                        cv2.putText(main_image, f'rng{rng-grng:04.1f} up{up-gup:04.1f} left{left-gleft:04.1f} mm', (50,23), cv2.FONT_HERSHEY_SIMPLEX, 1, (5,5,5), 1, cv2.LINE_AA)
                sg_utils.draw_image(window["-IMAGE-2-"],sg_utils.img_to_tk(main_image,1))#im_size[1]))
            main_image_depth = rovHandler.getMainImageDepth()
            if main_image_depth is not None and '-IMAGE-2D-' in window.AllKeysDict:
                window["-IMAGE-2D-"].update(data=sg_utils.img_to_tk(main_image_depth,1))
            if event == "Arm-Disarm":
                rovCommander.armdisarm()
            if event == "Depth-Hold":
                rovCommander.depth_hold()
            if event == syms.sym_down:
                rovCommander.depth_command(float(values['Target-Depth']))
            if event == syms.sym_up:
                rovCommander.depth_command(-float(values['Target-Depth']))

            if event == 'LOG':
                printer(f'log({time.time()}): {values["LOGtext"]}') 
                if 'LOGtext' in window.AllKeysDict:
                    window['LOGtext'].update('')


            if 'UP_CONT' in window.AllKeysDict and window['UP_CONT'].is_pressed(event):
                rovCommander.depth_command(-float(values['Target-Depth'])*0.01)
            if 'DOWN_CONT' in window.AllKeysDict and window['DOWN_CONT'].is_pressed(event):
                rovCommander.depth_command(float(values['Target-Depth'])*0.01)


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

            if event == 'Hold':
                rovCommander.depth_hold(True)
                rovCommander.att_hold(True)
                rovCommander.x_hold(True)
                rovCommander.y_hold(True)
                rovCommander.main_track(None)
                rovCommander.x_lock(values['X_LOCK'])
                rovCommander.y_lock(values['Y_LOCK'])
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
                if plot_type!='NONE':
                    mul=float(values['PID_Mul'].strip())/100+1.0
                    if event[1]=='-':
                        mul=1/mul
                    sg.cprint('sending pid',c='black on white')
                    rovCommander.update_pid(plot_type,event[0],mul)
            if event=='SAVE_PID':
                plot_type=values['-PLOT-TYPE-']
                if plot_type!='NONE':
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

            if event in ['Px','Py','Hold']:
                rovCommander.update_pxy((float(values['Px']),float(values['Py'])))

            if 'V_LOCK' in window.AllKeysDict:
                window['V_LOCK'](rovCommander.vertical_object_lock_state)

            if time.time()-last_heartbit>2.0:
                last_heartbit=time.time()
                rovCommander.heartbit()
                #send periodic events

            #send initialization data if not initialized yet and got first image from the ROV
            if not initialize_data_sent and last_im is not None:
                rovCommander.x_lock(values['X_LOCK'])
                rovCommander.y_lock(values['Y_LOCK'])
                rovCommander.d_lock(values['D_LOCK'])
                rovCommander.set_exposure_d405(int(values['D405EXP']))
                initialize_data_sent=True


            if event=='MISSION_SAVE':
                #track_thread.set_params(TrackThreadSG.get_layout_values(values))
                #track_thread.save_params(track_thread_file)
                #import json
                #js=json.dumps({k:values[k] for k in values.keys()},indent=4)
                #open(track_thread_file,'wb').write(js.encode())
                sg_utils.save_sg_state(window)

            if event=='AUTO_NEXT' and values['AUTO_NEXT']:
                #track_thread.auto_next=values['AUTO_NEXT']
                dir_scan=1 if values['SCAN_DIR_R'] else -1
                printer(f'dir_scan is {dir_scan}')
                track_thread.set_params(TrackThreadSG.get_layout_values(values,dir_scan)) #1.0 mean scan to the right -1 to the left
                track_thread.start()
                
                rovCommander.depth_hold(True)
                rovCommander.att_hold(True)
                rovCommander.x_hold(True)
                rovCommander.y_hold(True)

                if not rovHandler.is_recording():
                    rovHandler.toggle_recording()

                printer(f'set auto next to {track_thread.auto_next}')

            if values['AUTO_NEXT']:
                track_thread.run(float(values['Lrange']),float(values['k_max_alt']),
                        Pxy=(float(values['Px']),float(values['Py'])))

            if 1:
                window['MSTATE'](track_thread.get_state(),text_color='white',background_color='black')

            if trace_plotter is not None:
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

            for kkk in ['Gvr','Gvl']:
                if kkk in window.AllKeysDict:
                    if window[kkk].is_pressed(event):
                        print('llll')
                        rovCommander.set_gripper({'rot_vel':-1 if kkk=='Gvl' else 1})
                        printer(f'gripper rot')
                    if window[kkk].is_released(event):
                        rovCommander.set_gripper({'rot_vel':0.0})
                        printer(f'gripper stop rot')

            if event=='Go':
                rovCommander.set_gripper({'openning':0.0})

            if event=='Gc':
                if 'X_LOCK' in window.AllKeysDict:
                    window['X_LOCK'](False)
                    window['Y_LOCK'](False)
                    window['D_LOCK'](False)
                rovCommander.x_lock(False)
                rovCommander.y_lock(False)
                rovCommander.d_lock(False)
                rovCommander.set_gripper({'openning':1.0})

            if event=='Tx':
                rovCommander.main_track(None)

            if plotter is not None and values['-PLOT-TYPE-']=='DEPTH':
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
                if plotter is not None and values['-PLOT-TYPE-']==p_type:
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
                    if plotter is not None and values['-PLOT-TYPE-']==p_type:
                        plotter.update_pid(rovHandler.plot_buffers[pb],ylim=scaley)

            rovHandler.next()

            rov_telem=rovHandler.getTelemtry()
            if trace_plotter is not None:
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
