from scipy.signal import argrelextrema
import numpy as np

def rope_detect(prev_col,extrema,start_row,nrows,im,clear_freqs=5,max_diff_cols=100):
    imt=im[start_row:start_row+nrows,:].sum(axis=0).flatten().astype(float)
    #plt.plot(np.convolve(im1,np.concatenate((np.ones(8),-np.ones(8)))))
    #conv_val=10
    #im2=np.convolve(im1,np.ones(conv_val),'valid')
    yy=np.fft.fft(imt)
    #yy=np.fft.fftshift(yy) no need for shift high freqs in the middle
    yy[clear_freqs:-clear_freqs]=0
    #yy=np.fft.fftshift(yy)
    im2r=np.fft.ifft(yy).real
    im2r-=im2r.min()
    im2r/=im2r.max()
    #import pdb;pdb.set_trace()

    maxima=argrelextrema(im2r, np.greater)[0]
    minima=argrelextrema(im2r, np.less)[0]
    
    if extrema is None:
        totest=np.concatenate((maxima,minima))
        #new_col = np.argmax(np.abs(im2r))
        #import pdb;pdb.set_trace()
    elif extrema == 'max':
        totest=maxima
    else:
        totest=minima
    if len(totest)==0:
        return None
    new_col=totest[np.argmin(np.abs(totest-prev_col))]
        #print('cossen extrema', extrema,new_col, maxima,minima)
    #print('extrema is',extrema)
    if extrema is not None:
        if new_col-prev_col>max_diff_cols:
            #print('fix drift.1..',new_col,prev_col,prev_col+max_diff_cols, extrema, totest, minima,maxima)
            new_col=prev_col+max_diff_cols
        if prev_col-new_col>max_diff_cols:
            #print('fix drift.2..',new_col,prev_col,prev_col-max_diff_cols, extrema, totest, minima,maxima)
            new_col=prev_col-max_diff_cols
    if extrema is None:
        extrema='min'
        if new_col in maxima:
            extrema='max'
    debug={}
    debug['ifft']=im2r
    return extrema,new_col,debug

def rope_global_extrema(extrema,start_row,nrows,im,clear_freqs=5):
    imt=im[start_row:start_row+nrows,:].sum(axis=0).flatten().astype(float)
    #plt.plot(np.convolve(im1,np.concatenate((np.ones(8),-np.ones(8)))))
    #conv_val=10
    #im2=np.convolve(im1,np.ones(conv_val),'valid')
    yy=np.fft.fft(imt)
    #yy=np.fft.fftshift(yy) no need for shift high freqs in the middle
    yy[clear_freqs:-clear_freqs]=0
    #yy=np.fft.fftshift(yy)
    im2r=np.fft.ifft(yy).real
    im2r-=im2r.min()
    im2r/=im2r.max()
    #import pdb;pdb.set_trace()

    
    new_col=None
    if extrema is 'max':
        new_col = np.argmax(np.abs(im2r))
        #import pdb;pdb.set_trace()
    elif extrema == 'min':
        new_col = np.argmin(np.abs(im2r))
        #print('cossen extrema', extrema,new_col, maxima,minima)
    debug={}
    debug['ifft']=im2r
    return extrema,new_col,debug
