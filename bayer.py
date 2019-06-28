#gcc -o bayer_conv.so -shared -O3 bayer_conv.c
import ctypes
from ctypes import *
import numpy as np
libfile='./bayer_conv.so'
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
