# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import numpy as np
import zmq
import sys
import asyncio
import time
import pickle
import traceback

sys.path.append('..')
sys.path.append('../utils')
sys.path.append('../onboard')
import zmq_wrapper 
import zmq_topics
import config

from joy_mix import Joy_map  

if config.tracker=='rope':
    from tracker import rope_tracker as tracker
elif config.tracker=='local':
    from tracker import tracker

def printer(txt,c=None):
    print('printing:',txt)
    printer_source.send_pyobj({'txt':txt,'c':c})


async def recv_and_process():
    keep_running=True
    st=tracker.StereoTrack(printer=printer)
    jm=Joy_map()
    while keep_running:
        socks=zmq.select(subs_socks,[],[],0.005)[0]
        for sock in socks:
            ret=sock.recv_multipart()
            topic,data=ret[0],pickle.loads(ret[1])

            if topic==zmq_topics.topic_stereo_camera:
                frame_cnt,shape=data
                imgl=np.frombuffer(ret[2],'uint8').reshape(shape).copy()
                imgr=np.frombuffer(ret[3],'uint8').reshape(shape).copy()
                try:
                    ret=st(imgl,imgr)
                    ret['ts']=time.time()
                    ret['fnum']=frame_cnt
                    sock_pub.send_multipart([zmq_topics.topic_tracker,pickle.dumps(ret)])
                except:
                    print("Exception in tracker reseting tracker:")
                    traceback.print_exc(file=sys.stdout)
                    st.reset()
            if topic==zmq_topics.topic_button:
                jm.update_buttons(data)
                if jm.track_lock_event():
                    print('got lock event from joy')
                    st.reset()
            if topic==zmq_topics.topic_remote_cmd:
                if data['cmd']=='lock':
                    st.reset(data['click_pt'])
                if data['cmd']=='clear_freqs':
                    st.set_clear_freqs(data['data'],relative=data['rel'])
                if data['cmd']=='lock_max':
                    st.reset_max()
                if data['cmd']=='track_conf':
                    if data['rope_grey_func']=='hsv':
                        st.set_rope_detect_hsv()
                    if data['rope_grey_func']=='grey':
                        st.set_rope_detect_grey(chan=data['chan'])

            if topic==zmq_topics.topic_axes:
                jm.update_axis(data)
                if config.tracker=='rope':
                    if jm.inc_freqs_track_rope():
                        st.inc_clear_freqs()
                        print('inc clear freqs')
                    if jm.dec_freqs_track_rope():
                        st.dec_clear_freqs()
                        print('dec clear freqs')


        await asyncio.sleep(0.001)
 
async def main():
    await asyncio.gather(
            recv_and_process(),
            )

if __name__=='__main__':
    ### plugin inputs
    subs_socks=[]
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_stereo_camera],zmq_topics.topic_camera_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_axes,zmq_topics.topic_button,zmq_topics.topic_hat],
    zmq_topics.topic_joy_port))
    subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_remote_cmd],zmq_topics.topic_remote_cmd_port))
    printer_source = zmq_wrapper.push_source(zmq_topics.printer_sink_port)


    ### plugin outputs
    sock_pub=zmq_wrapper.publisher(zmq_topics.topic_tracker_port)
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
    #asyncio.run(main())


