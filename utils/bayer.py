#gcc -o bayer_conv.so -shared -O3 bayer_conv.c
import ctypes,os
from ctypes import *
import numpy as np

libpath=os.path.join(os.path.dirname(__file__), '../bin')
if not os.path.isdir(libpath):
    os.mkdir(libpath)

libfile=libpath+'/bayer_conv.so'

if not os.path.isfile(libfile):
    os.system('gcc -o {0}/../bin/bayer_conv.so -shared -O3 {0}/bayer_conv.c'.format(os.path.dirname(__file__)))
libc=ctypes.CDLL(libfile,mode=ctypes.RTLD_GLOBAL)

libc.convertRGBtoBayer.argtypes=[c_int,c_int,c_void_p,c_void_p]
libc.convertRGBtoBayer.restype=c_int
def convert_to_bayer(im):
    h,w,_=im.shape
    out=np.zeros((h,w),dtype='uint8')
    inptr=im.ctypes.data_as(c_void_p)
    outptr=out.ctypes.data_as(c_void_p)
    libc.convertRGBtoBayer(w,h,inptr,outptr)
    return out

def shrink_bayer_to_rgb(im):
    #todo
    pass
