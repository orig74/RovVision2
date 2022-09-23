# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import sys
sys.path.append("tracker")
import polyfit
import cv2
import scipy
import scipy.signal

sys.path.append("../../utils")

import config_tracker as config
from filters import ab_filt

track_params = config.track_params
stereo_corr_params = config.stereo_corr_params
from camera_tools import get_stereo_cameras,triangulate
from rope_detect import rope_detect 


show_hsv=False
hsv_wait=False

if show_hsv:
    cv2.namedWindow('hsv',cv2.WINDOW_NORMAL)
def togrey(im):
    if 0:
        return im[:,:,2]
    ret= cv2.cvtColor(im,cv2.COLOR_BGR2HSV_FULL)[:,:,0].copy()
    _,ret= cv2.threshold(ret,50,255,cv2.THRESH_TOZERO_INV)
    return ret*3


def generate_stereo_cameras():
    return get_stereo_cameras(config.focal_length,(config.pixelwidthx,config.pixelwidthy),config.baseline,config.camera_pitch)

class StereoTrack():
    def __init__(self):
        self.disparity_offset = config.track_offx
        self.stereo_wx,self.stereo_wy = stereo_corr_params['ws']
        self.stereo_sxl = stereo_corr_params['sxl']
        self.stereo_sxr = stereo_corr_params['sxr']
        self.stereo_sxl2 = stereo_corr_params['sxl2']
        self.stereo_sxr2 = stereo_corr_params['sxr2']
        self.debug=False
        self.proj_cams=generate_stereo_cameras()
        self.dx_filt=None
        self.rope_debug=None
        self.clear_freqs=config.clear_freqs
        self.reset()

    def reset(self,pt=None):
        if pt is None:
            self.ofx=self.disparity_offset
        else:
            self.ofx=int((pt[0]-0.5)*self.shape[1])
        #self.rope_track_state = None #'max','min',None
        self.rope_track_state = 'max' #'max','min',None
        self.last_res=None
        self.ref_point=None #first valid pt after reset

    def __track_left_im(self,imgl):
        shape=imgl.shape
        self.shape=shape
        cx  = shape[1]//2+self.ofx
        cy  = shape[0]//2
        
        #if config.ignore_extrema_type:
        #    self.rope_track_state = None

        ret=rope_detect(cx,self.rope_track_state, cy-100,200, 
                imgl,clear_freqs=self.clear_freqs, max_diff_cols=config.max_diff_cols)
        if ret is not None:
           self.rope_track_state, col,self.rope_debug=ret
           self.ofx = col-shape[1]//2
           return True
        
        return False

    def set_clear_freqs(self,val,relative):
        v = self.clear_freqs+val if relative else val
        self.clear_freqs=min(max(v,2),20)

    def inc_clear_freqs(self): # dec mins incressing the zerofreqs
        self.clear_freqs = max(self.clear_freqs-1,2)

    def dec_clear_freqs(self):
        self.clear_freqs = min(self.clear_freqs+1,20)

    def __track_stereo(self,imgl,imgr):
        cx,cy = imgl.shape[1]//2,imgl.shape[0]//2
        wx,wy = self.stereo_wx,self.stereo_wy
        sxl,sxr = self.stereo_sxl, self.stereo_sxr
        cx_off_l = cx+self.ofx 
        if self.last_res is not None and self.last_res['valid']:
            cx_off_r=int(self.last_res['pt_r'][0])
            sxl=self.stereo_sxl2
            sxr=self.stereo_sxr2
        else:
            cx_off_r=cx_off_l
        l2,r2=cx_off_r-wx//2-sxl,cx_off_r+wx//2+sxr
        #l2=np.clip(l2,0,imgl.shape[1]-1)
        #r2=np.clip(r2,0,imgl.shape[1]-1)
        l2=np.clip(l2,40,imgl.shape[1]-40)
        r2=np.clip(r2,40,imgl.shape[1]-40)
        #else:
        cy_off=cy
        l1,r1=cx_off_l-wx//2,cx_off_l+wx//2
        u1,d1=cy_off-wy//2,cy_off+wy//2



        #search up and down incase of inacurate camera model
        sy=6
        u2,d2=cy_off-wy//2-sy,cy_off+wy//2+sy

        corrs=[]
        try:
            corr_search=togrey(imgr[u2:d2,l2:r2,:].copy())
            corr_pat=togrey(imgl[u1:d1,l1:r1,:]).copy()
            corr = cv2.matchTemplate(corr_search,corr_pat,cv2.TM_CCOEFF_NORMED)
        except Exception as E:
            print('Error corr exception',E)
            return None
        corr-=corr.min()
        corr=corr/corr.max()
        snr=corr.max()/np.median(corr)

        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(corr)
        x,y = max_loc

        if 1<=x<corr.shape[1]-1 and 1<=y<corr.shape[0]-1:
            dx,dy=polyfit.fit(corr[y-1:y+2,x-1:x+2])
            x+=dx
            y+=dy

        #nx=x+(l2-l1)
        ny=y-sy


        #rx,ry =  nx+cx_off_r,ny+cy_off
        rx,ry =  x+l2+wx/2,ny+cy_off
        if self.debug:
            import matplotlib
            matplotlib.use('TKAgg')
            import matplotlib.pyplot as plt
            plt.figure('search {}'.format(self.debug))
            #plt.subplot(2,2,1)
            #plt.title('correlation template')
            #plt.imshow(self.corr_ref_pat,cmap='gray')
            plt.subplot(3,2,3)
            plt.title('stereo template')
            plt.imshow(patern,cmap='gray',vmin=0, vmax=255, interpolation='nearest')
            plt.plot(patern.shape[1]/2,patern.shape[0]/2,'+r')
            plt.subplot(3,2,4)
            plt.title('stereo search area')
            plt.imshow(search,cmap='gray',vmin=0, vmax=255, interpolation='nearest')
            plt.plot(max_loc[0]+patern.shape[0]/2,max_loc[1]+patern.shape[1]/2,'+r')
            plt.subplot(3,1,3)
            plt.title('correlation values')
            plt.xlabel('corr')
            plt.xlabel('x position')
            l=len(corr[0,:])
            plt.plot(np.linspace(0+patern.shape[0]/2,l+patern.shape[1]/2,l),corr.max(axis=0))
            #c2=scipy.signal.resample(corr[0,:],l*4)
            #plt.plot(np.linspace(0,l,len(c2)),c2)
            #plt.figure('img1/2')
            ax1=plt.subplot(3,2,1)
            plt.title('left image')
            plt.imshow(imgl,cmap='gray')
            plt.plot([l1,r1,r1,l1,l1],[u1,u1,d1,d1,u1],'r')
            plt.subplot(3,2,2,sharex=ax1,sharey=ax1)
            plt.title('right image')
            plt.imshow(imgr,cmap='gray')
            plt.plot([l2,r2,r2,l2,l2],[u2,u2,d2,d2,u2],'b')
            xx=[rx-wx/2,rx+wx/2,rx+wx/2,rx-wx/2,rx-wx/2]
            yy=[ry-wy/2,ry-wy/2,ry+wy/2,ry+wy/2,ry-wy/2]
            plt.plot(xx,yy,'r')
            plt.show()
            #import ipdb;ipdb.set_trace()
            self.debug=False
        #print('--->',x,nx,zx)
        if self.rope_debug is not None:
            self.rope_debug['scorr']=corr.max(axis=0)*100
            #self.rope_debug['scorr_xoff']=cx_off-sxl+self.ofx#-wx//2
            self.rope_debug['scorr_xoff']=l2+wx//2#self.ofx#+wx//2
        return rx,ry,snr


    def __track_and_validate(self, imgl, imgr):
        #imgl1r=imgl[:,:,0].copy()
        #imgr1r=imgr[:,:,0].copy()
        #imgl1b=imgl[:,:,2].copy()
        #imgr1b=imgr[:,:,2].copy()

        #im_grey_to_track = imgl[:,:,2].copy()
        im_grey_to_track = togrey(imgl)


        if show_hsv:
            cv2.imshow('hsv',im_grey_to_track)
            if hsv_wait:
                cv2.waitKey(1)
        cx,cy = im_grey_to_track.shape[1]//2,im_grey_to_track.shape[0]//2
        cx_off=cx+self.ofx
 
        valid = self.__track_left_im(im_grey_to_track) #tracked point on left image
        #pt_r_x,pt_r_y=self.__track_stereo(imgl1r,imgr1r) #tracked point on right image
        ret=self.__track_stereo(imgl,imgr) #tracked point on right image
        if ret is None:
            return False,0,0,0,0
        pt_r_x,pt_r_y,s_snr=ret
        #print('snr',s_snr)
        if s_snr<3:
            print('droped by snr',s_snr)
            valid=False

        return valid,cx_off,cy,pt_r_x,pt_r_y

    def __call__(self,imgl,imgr):
        
        valid,pt_l_x,pt_l_y,pt_r_x,pt_r_y = self.__track_and_validate(imgl, imgr)
        res={}

        res['pt_l']=(pt_l_x,pt_l_y)
        res['pt_r']=(pt_r_x,pt_r_y)
        res['range']=-1
        res['range_f']=-1

        t_pt = triangulate(self.proj_cams[0],self.proj_cams[1],pt_l_x,pt_l_y,pt_r_x,pt_r_y)
        #import ipdb;ipdb.set_trace()

        res['range']=t_pt[0] # range in the x direction
        #res['range_z']=t_pt[2] # range in the z direction


    #if valid:#abs(range_f-res['range']) < 0.20:  #range jumps less then 2m per sec (0.2/0.1)
        #if self.new_ref:
        #if self.new_ref or self.t_pt is None:
        dx = t_pt[0]
        dy = t_pt[1]
        dz = t_pt[2]
        valid = valid and dx>0.1 and dx<1.5*3

        if self.dx_filt is not None:
            valid = valid and abs(self.dx_filt.x-dx)<config.diff_range_valid
        if valid and self.ref_point is None:
            self.ref_point=(dx,dy,dz)
        
        if self.ref_point is not None:
            #dy-=self.ref_point[1]
            dz-=self.ref_point[2]

        res['valid']=valid
        if self.dx_filt is None or not valid:
            self.dx_filt = ab_filt((dx,0))
            self.dy_filt = ab_filt((dy,0))
            self.dz_filt = ab_filt((dz,0))


        res['dx']=dx
        res['dy']=dy
        res['dz']=dz
        res['dx_f']=self.dx_filt(dx)
        res['dy_f']=self.dy_filt(dy)
        res['dz_f']=self.dz_filt(dz)
        res['range_f']=res['dx_f'][0]
#else:
    #    print('new range filt')
    #    self.range_filt = ab_filt((res['range'],0))
        res['new_ref']=False
        res['ref_cnt']=0
        res['clr_frq']=self.clear_freqs
        if self.rope_debug is not None:
            res['rope_debug']=self.rope_debug
        self.last_res=res
        return res


def draw_track_rects(ret,imgl,imgr):
    wx,wy=config.stereo_corr_params['ws']

    wx_t,wy_t = config.track_params[:2]
    wx_s,wy_s = config.stereo_corr_params['ws']
    if 'rope_debug' in ret:
        arr=(ret['rope_debug']['ifft']-0.5)*100
        #arr=arr.real**2+arr.imag**2
        #arr=20*np.log(arr)
        #arr-=arr.min()
        #arr=np.clip(arr,-250,250).astype(int)
        arr=arr.astype(int)
        for i,a in enumerate(arr):
            try:
                imgl[a+260,i,:]=0
            except:
                if i==0:
                    print('error in ploting')
        if 'scorr' in ret['rope_debug']:
            arr=-ret['rope_debug']['scorr']
            #arr=20*np.log(arr)
            arr-=arr.min()
            arr=np.clip(arr,-250,250).astype(int)
            for i,a in enumerate(arr):
                try:
                    imgr[a+260,i+ret['rope_debug']['scorr_xoff'],:]=0
                except:
                    print('Err plot',a+260,i+ret['rope_debug']['scorr_xoff'])

    if 'pt_r' in ret:
        xl,yl=map(int,ret['pt_l'])
        xr,yr=map(int,ret['pt_r'])
        valid = ret.get('valid',False)
        if valid:
            col = (0,255,0)
        else:
            col = (0,0,255)
        cv2.rectangle(imgl,(xl-wx_t//2,yl-wy_t//2),(xl+wx_t//2,yl+wy_t//2),col)
        cv2.rectangle(imgl,(xl-wx_s//2,yl-wy_s//2),(xl+wx_s//2,yl+wy_s//2),col)
        cv2.rectangle(imgr,(xr-wx_s//2,yr-wy_s//2),(xr+wx_s//2,yr+wy_s//2),col)
        #print('++++',xr-xl)

    if 'clr_frq' in ret:
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(imgr,'cf %d'%ret['clr_frq'],(10,50), font, 0.5,(0,0,255),1,cv2.LINE_AA)

    shape=imgl.shape
    cx  = shape[1]//2+config.track_offx
    cy  = shape[0]//2
    cv2.circle(imgl, (cx,cy), 5, (0,0,255), 1)


if __name__=="__main__":
    sys.path.append('../../')
    import gst
    dd=StereoTrack()
    show_hsv=True
    #fr=gst.gst_file_reader('../../../data/190726-063112/',False)
    #fr=gst.gst_file_reader('../../../data/190803-141658/',False)
    #fr=gst.gst_file_reader('../../../data/190822-140723/',False)
    fr=gst.gst_file_reader(sys.argv[1],True)
    if len(sys.argv)>2:
        skip=int(sys.argv[2])
    else:
        skip=0
    print('skiping ',skip,'fnums')
    keep_run=True
    for i,data in enumerate(fr):
        #print(i)
        if not keep_run:
            break
        images,cnt=data 
        if cnt>0:
            ret=dd(*images)
            #print(ret)
            iml,imr=images
            imls=iml.copy()
            imrs=imr.copy()
            print('====',cnt,ret['dx'],ret['dy'],ret['dz']) 
            draw_track_rects(ret,imls,imrs)
            if 1:
                cv2.imshow('left',imls)
                cv2.imshow('rigth',imrs)
            else:
                im_st=imls.copy()
                par=150
                im_st[:,par:,2]=imrs[:,:-par,0]
                cv2.imshow('left',im_st)
                
            
            while 1:
                if cnt<skip:
                    k=cv2.waitKey(1)
                    break

                k=cv2.waitKey(0)
                if k%256==ord('q'):
                    keep_run=False
                    break
                if k%256==ord(' '):
                    break
                if k%256==ord('s'):
                    cv2.imwrite('iml.png',iml)

                if k%256==ord('='):
                    dd.dec_clear_freqs()
                    break
                if k%256==ord('-'):
                    dd.inc_clear_freqs()
                    break

                if k%256==ord('d'):
                    dd.debug=True
                if k%256==ord('r'):
                    print('reset')
                    dd.reset()
                    break
