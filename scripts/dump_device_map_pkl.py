import os
devices=os.popen('find /sys/bus/usb/devices/usb*/ -name dev |grep  tty').readlines()
for l in devices:
    dname = l.strip().split('/')[-2]
    tofind='udevadm info -q property -p %s |grep ID_SERIAL='%l.strip()[:-4]
    lname = os.popen(tofind).read().split('=')[1].strip()
    #print(dname,lname)
    if '1a86' in lname:
        print('ESC_USB=/dev/'+dname)
    if 'Arduino_LLC_Arduino_Leonardo' in lname:
        print('PERI_USB=/dev/'+dname)
    if 'CP210' in lname:
        print('SONAR_USB=/dev/'+dname)
    if 'FTDI' in lname:
        print('VNAV_USB=/dev/'+dname)

