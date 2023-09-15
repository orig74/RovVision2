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

def rope_detect_depth(depth_img,scale_to_mm,water_scale,start_row=150,nrows=100,validation_rows=50):
    #np.save('/tmp/imgd.npy',depth_img)
    marg=100
    sliceimg=depth_img[start_row-validation_rows:start_row+nrows+validation_rows,:]
    scaled_d = sliceimg*scale_to_mm*water_scale
    scaled_d[scaled_d<1]=10000 #10 meters
    imt=scaled_d[validation_rows:-validation_rows].sum(axis=0).flatten()/nrows
    #imt=np.median(scaled_d[validation_rows:-validation_rows],axis=0)#.flatten()/nrows
    #prioritizing center
    r=600
    imtp=imt+np.abs(np.linspace(-r,r,len(imt)))
    #blur line
    flt=50
    imtp=np.convolve(imtp,np.ones(flt)/flt,mode='same')
    col=np.argmin(imtp[marg:-marg])+marg

    #up_validation=scaled_d[:validation_rows,col].sum()/validation_rows
    #down_validation=scaled_d[-validation_rows:,col].sum()/validation_rows
    
    #width_check
    wc=50
    #up_validation=scaled_d[:validation_rows,col-wc:col+wc].mean(axis=0)
    #down_validation=scaled_d[-validation_rows:,col-wc:col+wc].mean(axis=0)
    up_validation=np.median(scaled_d[:validation_rows,col-wc:col+wc],axis=0)
    down_validation=np.median(scaled_d[-validation_rows:,col-wc:col+wc],axis=0)


    if 0:
        print('---',imt[col],col,up_validation.min(),down_validation.min())
        from matplotlib import pyplot as plt
        import sys
        plt.figure('up')
        plt.imshow(scaled_d[:validation_rows,col-wc:col+wc])
        plt.figure('up mean')
        plt.plot(scaled_d[:validation_rows,col-wc:col+wc].mean(axis=0))
        plt.figure('down')
        plt.imshow(scaled_d[-validation_rows:,col-wc:col+wc])
        plt.figure('down mean')
        plt.plot(scaled_d[-validation_rows:,col-wc:col+wc].mean(axis=0))
        plt.figure('med range')
        plt.plot(imtp)
        plt.plot(imt)
        plt.legend(['filtered','raw'])
        plt.show()
        sys.exit(0)

    return imt[col],col,up_validation.min(),down_validation.min(),imtp



