# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
############# gst ########
#to watch
#gst-launch-1.0 -e -v udpsrc port=5700 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
#gst-launch-1.0 -e -v udpsrc port=5701 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
from subprocess import Popen,PIPE
import sys,time,select,os
import numpy as np
import config
import image_enc_dec
############# gst wirite #########
gst_pipes=None
send_cnt=[0,0]
def init_gst(sx,sy,npipes):
    global gst_pipes
    cmd="gst-launch-1.0 {{}}! x264enc threads=1 speed-preset={} tune=zerolatency  bitrate={} key-int-max=50 ! tcpserversink port={{}}".format(config.gst_speed_preset, config.gst_bitrate)
    gstsrc = 'fdsrc ! videoparse width={} height={} format=15 ! videoconvert ! video/x-raw, format=I420'.format(sx,sy) #! autovideosink'

    gst_pipes=[]
    for i in range(npipes):
        gcmd = cmd.format(gstsrc,config.gst_ports[i])
        p = Popen(gcmd, shell=True, bufsize=0,stdin=PIPE, stdout=sys.stdout, close_fds=False)
        print('gcmd = ',gcmd)
        gst_pipes.append(p)

def send_gst(imgs):
    global gst_pipes
    for i,im in enumerate(imgs):
        time.sleep(0.001)
        if len(select.select([],[gst_pipes[i].stdin],[],0.005)[1])>0:
            gst_pipes[i].stdin.write(im.tostring())
            send_cnt[i]+=1

def init_gst_files(sx,sy):
   pass


############# gst read #########
gst_pipes_264=None
sx,sy=config.cam_res_rgbx,config.cam_res_rgby
shape = (sx, sy, 3)

def init_gst_reader(npipes):
    global gst_pipes,gst_pipes_264
    if 1: #h264
        cmd='gst-launch-1.0 tcpclientsrc port={} ! identity sync=true  ! tee name=t ! queue ! filesink location=fifo_264_{}  sync=false  t. ! queue !'+\
        ' h264parse ! decodebin ! videoconvert ! video/x-raw,height={},width={},format=RGB ! filesink location=fifo_raw_{}  sync=false'
    if 0:
        cmd='gst-launch-1.0 -q udpsrc port={} ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,height={},width={},format=RGB ! fdsink'

    gst_pipes=[]
    gst_pipes_264=[]
    os.system('rm fifo_*')
    cmds=[]
    for i in range(npipes):
        fname_264='fifo_264_'+'lr'[i]
        os.mkfifo(fname_264)
        r = os.open(fname_264,os.O_RDONLY | os.O_NONBLOCK)
        fname_raw='fifo_raw_'+'lr'[i]
        os.mkfifo(fname_raw)
        r1 = os.open(fname_raw,os.O_RDONLY | os.O_NONBLOCK)
        gcmd = cmd.format(config.gst_ports[i],'lr'[i],sy,sx,'lr'[i])
        print(gcmd)
        cmds.append(gcmd)
        gst_pipes_264.append(r)
        gst_pipes.append(r1)
    for cmd in cmds: #start together
        Popen(cmd, shell=True, bufsize=0)

if config.camera_setup == 'stereo':
    images=[None,None]
    save_files_fds=[None,None]
else:
    images=[None]
    save_files_fds=[None]

def get_files_fds():
    return save_files_fds

def set_files_fds(fds):
    for i in [0,1]:
        save_files_fds[i]=fds[i]

def get_imgs():
    global images
    for i in range(len(images)):
        if len(select.select([ gst_pipes[i] ],[],[],0.005)[0])>0 :
            data=os.read(gst_pipes[i],sx*sy*3)
            try:
                images[i] = np.fromstring(data,'uint8').reshape([sy,sx,3])
            except:
                images[i]=np.zeros((sy, sx, 3), dtype=np.uint8)
        if len(select.select([ gst_pipes_264[i] ],[],[],0.005)[0])>0:
            data=os.read(gst_pipes_264[i],1*1000*1000)
            if save_files_fds[0] is not None:
                save_files_fds[i].write(data)
    return images


def save_main_camera_stream(fname):
    cmd='''gst-launch-1.0 -e udpsrc port=6753 ! application/x-rtp, encoding-name=H264, payload=96 ! rtph264depay ! h264parse ! mp4mux ! filesink location={} sync=false'''.format(fname)
    #return Popen(cmd, shell=True)
    cmdp=cmd.split()
    return Popen(cmdp)

############ gst from files #################
import glob
def read_image_from_pipe(p, prevcnt=-1):
    if len(select.select([p],[],[],5)[0])==0:
        print('got None!!')
        return None,-1
    data=os.read(p,sx*sy*3)
    if data:
        img=np.fromstring(data,'uint8').reshape([sy,sx,3])
        fmt_cnt=image_enc_dec.decode(img)
        if fmt_cnt is None:
            fmt_cnt = prevcnt+1
    else:
        #print('Error no data')
        return None,-1
    return img,fmt_cnt

def gst_file_reader(path, nosync):
    global images
    cmd='gst-launch-1.0 filesrc location={} ! '+\
        ' h264parse ! decodebin ! videoconvert ! video/x-raw,height={},width={},format=RGB ! filesink location=fifo_raw_{}  sync=false'
    gst_pipes=[]
    os.system('rm fifo_raw_*')

    for i in [0,1]:
        fname_raw='fifo_raw_'+'lr'[i]
        os.mkfifo(fname_raw)
        r1 = os.open(fname_raw,os.O_RDONLY | os.O_NONBLOCK)
        fname=glob.glob(path+'/*_'+'lr'[i]+'.mp4')[0]
        gcmd = cmd.format(fname,sy,sx,'lr'[i])
        print(gcmd)
        gst_pipes.append(r1)
        Popen(gcmd, shell=True)

    prevcnt=-1
    while 1:
        if len(select.select(gst_pipes,[],[],0.1)[0])==len(gst_pipes):
            im1,cnt1=read_image_from_pipe(gst_pipes[0],prevcnt)
            im2,cnt2=read_image_from_pipe(gst_pipes[1],prevcnt)
            #syncing frame numbers
            if not nosync:
                if cnt1 > 0  and cnt2 > 0:
                    while cnt2>cnt1:
                        im1,cnt1=read_image_from_pipe(gst_pipes[0],prevcnt)
                    while cnt1>cnt2:
                        im2,cnt2=read_image_from_pipe(gst_pipes[1],prevcnt)
            images=[im1,im2]
            prevcnt = cnt1
            yield images,cnt1
        else:
            time.sleep(0.001)
            yield None,-1
