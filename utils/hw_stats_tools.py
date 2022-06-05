import os
def get_cpu_temp():
    try:
        return float(os.popen('sensors |grep "Package id"').readline().strip().split(':')[1].strip().split()[0].strip()[:-2])
    except:
        return -1

def get_disk_usage():
    return int(os.popen('df -h /media/data | tail -n 1').readline().strip().split()[-2][:-1])

def get_cpu_usage():
    try:
        return 100-float(os.popen('mpstat 2 4 | grep Average').readline().strip().split()[-1])
    except:
        return -1

def get_cpu_freq():
    return float(os.popen('lscpu | grep MHz | grep "CPU MHz"').readline().split()[-1])

def get_hw_stats():
    return (get_cpu_temp(),get_disk_usage(),get_cpu_usage(), get_cpu_freq())

def get_hw_str(info):
    return 'CPU {:.1f} HD {}% TEMP {:.1f} FREQ {:.1f}'.format(*info)

if __name__=='__main__':
    print('cpu temp',get_cpu_temp())
    print('disk usage',get_disk_usage())
    print('cpu usage',get_cpu_usage())
    info = get_hw_stats()
    print(get_hw_str(info))
