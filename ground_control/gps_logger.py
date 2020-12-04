import sys
sys.path.append('../')
sys.path.append('../utils')
import zmq_topics
import zmq_wrapper
import zmq
import time
import pickle
import requests
import json


GPS_STATIC_URL='http://192.168.2.94'


pub_sock = zmq_wrapper.publisher(zmq_topics.topic_gps_port)
subs_socks=[]
subs_socks.append(zmq_wrapper.subscribe([zmq_topics.topic_depth],zmq_topics.topic_depth_port))

def get_data(url):
    try:
        r = requests.get(url)
    except requests.exceptions.RequestException as exc:
        print("Exception occured {}".format(exc))
        return None

    if r.status_code != requests.codes.ok:
        print("Got error {}: {}".format(r.status_code, r.text))
        return None

    return r.json()


def set_external(depth, temp):
    payload = {"depth":float(depth), "temp":float(temp)}
    r = requests.put('{}/api/v1/external/depth'.format(GPS_STATIC_URL), json=payload)
    if r.status_code != 200:
        print("Error setting depth: {} {}".format(r.status_code, r.text))


if __name__=='__main__':
    while 1:
        time.sleep(0.25)

        socks = zmq.select(subs_socks, [], [], 0.005)[0]
        for sock in socks:
            ret = sock.recv_multipart()
            topic, data = ret[0], pickle.loads(ret[1])

            if topic == zmq_topics.topic_depth:
                d_ts, depth = data['ts'], data['depth']
                set_external(depth, 20)
    
        acoustic_data=get_data("{}/api/v1/position/acoustic/filtered".format(GPS_STATIC_URL))
        global_data=get_data("{}/api/v1/position/global".format(GPS_STATIC_URL))
        print(global_data)

        if global_data != None and acoustic_data != None:
            gps_pub_data = {'ts': time.time(), 'gps': global_data, 'acoustic': acoustic_data}
            pub_sock.send_multipart([zmq_topics.topic_gps, pickle.dumps(gps_pub_data, -1)])