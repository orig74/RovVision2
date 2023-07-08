import farm_track_thread as ft
import PySimpleGUI as sg
def get_layout(farm_obj,cols=2):
    ret=[[]]
    for k,_ in ft.mission_vars_default:
        v=farm_obj.__dict__[k]
        if len(ret[-1])==(2*cols):
            ret.append([])
        if type(v) in [float,int]:
            ret[-1].append(sg.Text(k+':',tooltip=ft.tool_tips.get(k,None)))
            ret[-1].append(sg.Input(v,size=(4,1),key='k_'+k,tooltip=ft.tool_tips.get(k,None)))
    return ret
#def set_layout():
#    mv=ft.mission_vars_default
#    return \
#        [[sg.Text('Slide:'),sg.Input(mv['horizontal_slide'],
def get_layout_values(values,dir_scan=1.0): #1.0 mean scan to the right -1 to the left
    ret={}
    for k,v in ft.mission_vars_default:
        ret[k]=float(values['k_'+k])
    ret['horizontal_slide']=abs(ret['horizontal_slide'])*dir_scan
    return ret.items()

