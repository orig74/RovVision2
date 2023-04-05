import numpy as np
import argparse,cv2

def fast_bayer_shrink(dest,img):
    dest[:,:,2]=np.squeeze(img[::2,::2])
    dest[:,:,1]=np.squeeze(img[1::2,0::2])
    dest[:,:,0]=np.squeeze(img[1::2,1::2])


if __name__=='__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("pgmfile",help="pgm file")
    args = parser.parse_args()
    im=cv2.imread(args.pgmfile,cv2.IMREAD_GRAYSCALE)
    cached_image = np.zeros((im.shape[0]//2,im.shape[1]//2,3),dtype='uint8')
    fast_bayer_shrink(cached_image,im)
    jpg_name=args.pgmfile[:-3]+'jpg'
    print('writing',jpg_name)
    cv2.imwrite(jpg_name,cached_image,[int(cv2.IMWRITE_JPEG_QUALITY), 70])
