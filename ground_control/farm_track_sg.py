import farm_track_thread as ft
import PySimpleGUI as sg
def get_layout(cols=2):
    ret=[[]]
    for k,v in ft.mission_vars_default:
        if len(ret[-1])==(2*cols):
            ret.append([])
        if type(v) in [float,int]:
            ret[-1].append(sg.Text(k+':'))
            ret[-1].append(sg.Input(v,size=(4,1),key='k_'+k))
    return ret
#def set_layout():
#    mv=ft.mission_vars_default
#    return \
#        [[sg.Text('Slide:'),sg.Input(mv['horizontal_slide'],
def get_layout_values(values):
    ret={}
    for k,v in ft.mission_vars_default:
        ret[k]=float(values['k_'+k])
    return ret.items()

