import cv2
def im16to8_22(im):
    #return cv2.applyColorMap(cv2.convertScaleAbs(im[::2,::2].copy(), alpha=0.03), cv2.COLORMAP_JET)
    imret = cv2.applyColorMap(cv2.convertScaleAbs(im[::2,::2].copy(), alpha=0.46), cv2.COLORMAP_JET)[:,:,::-1]
    #imret[im==0]=(0,0,0)
    return imret

