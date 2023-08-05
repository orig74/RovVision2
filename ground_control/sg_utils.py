import pickle,os
from PIL import Image,ImageTk
import PySimpleGUI as sg
import cv2

param_file='param_file.pkl'

ignore_save_load_list=['AUTO_NEXT']

def save_sg_state(window):
    d=window.AllKeysDict
    to_save={}
    for k in d:
        if type(window[k]) in [sg.Input,sg.Checkbox,sg.Combo] and k not in ignore_save_load_list:
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
            if k in di and k not in ignore_save_load_list \
                    and type(window[k]) in [sg.Input,sg.Checkbox]:
                window[k](di[k])
        params_file_data=di

def update_default_sg_values(vals):
    if params_file_data is not None:
        for k in params_file_data:
            if k not in vals:
                vals[k]=params_file_data[k]

def update_image(window,key,raw_image,shrink):
    if raw_image is not None and key in window.AllKeysDict:
        window[key].update(data=img_to_tk(raw_image,shrink)) 

#def update_values_from_file(values):
#    if os.path.isfile(track_thread_file):
        #track_thread.load_params(track_thread_file)
#        import json
#        js=json.loads(open(track_thread_file,'rb').read().strip())
#        for key in js:
#            values[key]=js[key]



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


