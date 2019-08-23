import os
import pickle

if __name__=='__main__':
    dmap = {}
    if 0:
        devices=os.popen('find /sys/bus/usb/devices/usb*/ -name dev |grep  tty').readlines()
        for l in devices:
            dname = l.strip().split('/')[-2]
            tofind='udevadm info -q property -p %s |grep ID_SERIAL='%l.strip()[:-4]
            lname = os.popen(tofind).read().split('=')[1].strip()
            #print(dname,lname)
            if '1a86' in lname:
                dmap['ESC_USB']='/dev/'+dname
            if 'Arduino_LLC_Arduino_Leonardo' in lname:
                dmap['PERI_USB']='/dev/'+dname
            if 'CP210' in lname:
                dmap['SONAR_USB']='/dev/'+dname
            if 'FTDI' in lname:
                dmap['VNAV_USB']='/dev/'+dname
    else:
        for dev in ['/dev/ttyUSB%d'%i for i in [0,1,2]]:
            cmd = 'udevadm info '+dev
            #print(cmd)
            line=os.popen(cmd).readline()
            if '1-7' in line:
                dmap['PERI_USB']=dev
            if '1-5' in line:
                dmap['ESC_USB']=dev
            if '1-3' in line:
                dmap['VNAV_USB']=dev

        with open('/tmp/devusbmap.pkl','wb') as fp:
            #print(dmap)
            pickle.dump(dmap,fp)

else:
    if not os.path.isfile('/tmp/devusbmap.pkl'):
        print('Error cannot find /tmp/devusbmap.pkl please run detect_usb.py')
    devmap = pickle.load(open('/tmp/devusbmap.pkl','rb'))

