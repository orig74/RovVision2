import sys,os,time,traceback
sys.path.append('../')
sys.path.append('../utils')
import pickle
import numpy as np
import pandas as pd
import argparse
import pprint
parser = argparse.ArgumentParser()
parser.add_argument("-p","--date_pattern",default="*",help="date patttern to look default '*'")
parser.add_argument("path",help="dir path")
args = parser.parse_args()
dirs=os.popen(f'cd {args.path} && find . -type d -name "{args.date_pattern}"').readlines()
dirs=[d[2:].strip() for d in dirs if len(d)>2]
dirs.sort()
for ddd in dirs:
    last_frame=None
    pkl_file=args.path+'/'+ddd+'/viewer_data.pkl'
    if os.path.isfile(pkl_file):
        print(f'===== {ddd} =====')
        data=[]
        with open(pkl_file,'rb') as fd:
            while 1:
                try:
                    d=pickle.load(fd)
                    if d[0]==b'gui_data':
                        pprint.pprint(d[2])
                    elif d[0]==b'gui_event':
                        if d[2][0]=='LOG':
                            print(f'{last_frame}:{d[2][0]} {d[2][1]["LOGtext"]}')
                    elif d[0]==b'topic_main_cam_ts':
                        last_frame=d[2][0]

                    data.append(d)
                except EOFError:
                    break
        print(f'recording time {int(data[-1][1]-data[0][1])//60} min')
        #import ipdb;ipdb.set_trace()
        #ggg
        
    


