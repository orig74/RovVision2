from vnpy import *
import time,sys
import os,zmq
sys.path.append('..')
sys.path.append('../utils')
import zmq_topics


pub_imu = utils.publisher(zmq_topics.topic_imu_port)


s=EzAsyncData.connect('/dev/'+os.environ['VNAV_USB'],115200)
s.sensor.write_async_data_output_frequency(40) #hz
def get_data():
    cd=s.current_data
    ret={}
    ypr=cd.yaw_pitch_roll
    rates=cd.angular_rate
    #acc=cd.acceleration
    ret['ypr']=(ypr.x,ypr.y,ypr.z)
    ret['rates']=(rates.x,rates.y,rates.z)
    return ret

async def recv_and_process():
    while 1:
        tic=timev.time()
        imu={'ts':tic}
        cd=s.current_data
        ret={}
        ypr=cd.yaw_pitch_roll
        rates=cd.angular_rate
        imu['yaw'],imu['pitch'],imu['roll']=(ypr.x,ypr.y,ypr.z)

        imu['rates']=(rates.x,rates.y,rates.z)


        print('dsim Y{:4.2f} P{:4.2f} R{:4.2f}'.format(imu['yaw'],imu['pitch'],imu['roll'])
                    +' X{:4.2f} Y{:4.2f} Z{:4.2f}'.format(*imu['rates']))
        pub_imu.send_multipart([zmq_topics.topic_imu,pickle.dumps(imu)])
        await asyncio.sleep(0.020)

async def main():
    await asyncio.gather(
            recv_and_process(),
            pubposition(),
            )

if __name__=='__main__':
#    asyncio.run(main())
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())

