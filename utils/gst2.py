from subprocess import Popen,PIPE
import sys,time,select,os
import numpy as np
import cv2
#import image_enc_dec
if 0:
    import config
    gst_speed_preset=config.gst_speed_preset
    gst_bitrate=config.gst_bitrate
else:
    gst_speed_preset=1
    gst_bitrate=1024*3

#
class Writer(object):
    def __init__(self,port,sx,sy,pad_lines=0):
        cmd="gst-launch-1.0 {{}}! x264enc threads=1 speed-preset={} tune=zerolatency  bitrate={} key-int-max=50 ! tcpserversink port={{}}".format(gst_speed_preset, gst_bitrate)
        #gstsrc = 'fdsrc ! videoparse width={} height={} format=15 ! videoconvert ! video/x-raw, format=I420'.format(sx,sy+pad_lines) #! autovideosink'
        gstsrc = 'fdsrc ! videoparse width={} height={} format=16 ! videoconvert ! video/x-raw, format=I420'.format(sx,sy+pad_lines) #! autovideosink'

        gcmd = cmd.format(gstsrc,port)
        self.p = Popen(gcmd, shell=True, bufsize=0,stdin=PIPE, stdout=sys.stdout, close_fds=False)
        print('gcmd = ',gcmd)
        self.cnt=0
        #self.pad_lines=pad_lines
        #self.sx=sx
        self.lines_to_add=b'\x00'*(sx*3*pad_lines) if pad_lines>0 else None

    def write(self,im):
        if len(select.select([],[self.p.stdin],[],0.005)[1])>0:
            tosend=im.tostring()
            self.p.stdin.write(tosend)
            if self.lines_to_add is not None:
                self.p.stdin.write(self.lines_to_add)

            self.cnt+=1

class Reader(object):
    def __init__(self,name,port,sx,sy,pad_lines=0):
        if 1: #h264
            cmd='gst-launch-1.0 tcpclientsrc port={} ! identity sync=true  ! tee name=t ! queue ! filesink location=fifo_264_{}  sync=false  t. ! queue !'+\
            ' h264parse ! decodebin ! videoconvert ! video/x-raw,height={},width={},format=RGB ! filesink location=fifo_raw_{}  sync=false'
        if 0:
            cmd='gst-launch-1.0 -q udpsrc port={} ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,height={},width={},format=RGB ! fdsink'

        fname_264='fifo_264_'+name
        os.system('rm '+fname_264)
        os.mkfifo(fname_264)
        r = os.open(fname_264,os.O_RDONLY | os.O_NONBLOCK)
        fname_raw='fifo_raw_'+name
        os.system('rm '+fname_raw)
        os.mkfifo(fname_raw)
        r1 = os.open(fname_raw,os.O_RDONLY | os.O_NONBLOCK)
        #gcmd = cmd.format(port,name,sy,sx,name)
        gcmd = cmd.format(port,name,sy+pad_lines,sx,name)
        print('gst line is:')
        print(gcmd)
        self.pipe_264=r
        self.pipe=r1
        Popen(gcmd, shell=True, bufsize=0)
        self.sxsy=(sx,sy)
        self.save_fd=None
        self.name=name
        self.pad_lines=pad_lines

    def set_save_fd(self,fd):
        self.save_fd=fd

    def get_save_fd(self):
        return self.save_fd

    def get_img(self):
        sx,sy=self.sxsy
        image=None
        #print('gst2...name',self.name)
        while len(select.select([self.pipe],[],[],0.001)[0])>0:
            data=b''
            toread=sx*(sy+self.pad_lines)*3
            while len(data)<toread:
                try:
                    data+=os.read(self.pipe,toread-len(data))
                except BlockingIOError:
                    time.sleep(0.001)
            if self.pad_lines>0:
                data=data[:sx*sy*3]
            try:
                image = np.fromstring(data,'uint8').reshape([sy,sx,3])[:,:,::-1].copy()
            except:
                image = np.zeros((sy, sx, 3), dtype=np.uint8)
        if len(select.select([self.pipe_264],[],[],0.0)[0])>0:
            data=os.read(self.pipe_264,1*1000*1000)
            if self.save_fd is not None:
                self.save_fd.write(data)
        return image

tsx,tsy=616,514
#tsx,tsy=616,516
#tsx,tsy=640,480
if __name__=='__main__' and sys.argv[1]=='s':
    zer=np.zeros((tsy,tsx,3),dtype='uint8')
    i=0
    cv2.namedWindow('gsttest',cv2.WINDOW_NORMAL)
    writer = Writer(7777,tsx,tsy,pad_lines=2)
    writer2 = Writer(7778,tsx,tsy,pad_lines=2)
    while True:
        im=zer.copy()
        cv2.circle(im,(i+100,i+100),i+10,(255,200,50),3)
        cv2.imshow('gsttest',im)
        writer.write(im)
        writer2.write(im)
        k=cv2.waitKey(1)
        if k%256==ord('q'):
            break
        i+=1
        i=i%100

if __name__=='__main__' and sys.argv[1]=='c':
    cv2.namedWindow('gstread',cv2.WINDOW_NORMAL)
    cv2.namedWindow('gstread2',cv2.WINDOW_NORMAL)
    reader=Reader('main',7777,tsx,tsy,pad_lines=2)
    reader2=Reader('main_depth',7778,tsx,tsy,pad_lines=2)
    while True:
        im=reader.get_img()
        im2=reader2.get_img()
        if im2 is not None:
            cv2.imshow('gstread2',im2)
        if im is not None:
            cv2.imshow('gstread',im)
        k=cv2.waitKey(1)
        if k%256==ord('q'):
            break
        time.sleep(0.001)
     
