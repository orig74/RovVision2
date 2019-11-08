import os
import pickle

if __name__=='__main__':
    dmap = {}
    rov_type = int(os.environ.get('ROV_TYPE','1'))
    for dev in ['/dev/ttyUSB%d'%i for i in [0,1,2]] + ['/dev/ttyACM0']:
        cmd = 'udevadm info '+dev
        line=os.popen(cmd).readline()
        
        if rov_type==1:
            if '1-7' in line:
                dmap['SONAR_USB']=dev
            if '1-5' in line:
                dmap['ESC_USB']=dev
            if '1-3' in line:
                dmap['VNAV_USB']=dev
            if '1-1' in line:
                dmap['PERI_USB']=dev
        
        if rov_type==2:
            if '1-7.1' in line:
                dmap['SONAR_USB']=dev
            if '1-7.2' in line:
                dmap['ESC_USB']=dev
            if '1-7.3' in line:
                dmap['VNAV_USB']=dev
            if '1-9' in line:
                dmap['PERI_USB']=dev

    with open('/tmp/devusbmap.pkl','wb') as fp:
        #print(dmap)
        pickle.dump(dmap,fp)
        print('device map = ',dmap)

else:
    if not os.path.isfile('/tmp/devusbmap.pkl'):
        print('Error cannot find /tmp/devusbmap.pkl please run detect_usb.py')
    devmap = pickle.load(open('/tmp/devusbmap.pkl','rb'))

