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

        try:
            socks = zmq.select(subs_socks, [], [], 0.005)[0]
            for sock in socks:
                ret = sock.recv_multipart()
                topic, data = ret[0], pickle.loads(ret[1])

                if topic == zmq_topics.topic_depth:
                    d_ts, depth, temp = data['ts'], data['depth'], data['temp']
                    set_external(depth, temp)
        
            acoustic_filtered_data=get_data("{}/api/v1/position/acoustic/filtered".format(GPS_STATIC_URL))
            acoustic_raw_data=get_data("{}/api/v1/position/acoustic/raw".format(GPS_STATIC_URL))
            global_locator_data=get_data("{}/api/v1/position/global".format(GPS_STATIC_URL))
            global_master_data=get_data("{}/api/v1/position/master".format(GPS_STATIC_URL))
            print(global_locator_data)

            if acoustic_filtered_data != None and acoustic_raw_data != None and global_locator_data != None and global_master_data != None:
                gps_pub_data = {'ts': time.time(), 'gps_master': global_master_data, 'gps_locator': global_locator_data,
                                                   'acoustic_raw': acoustic_raw_data, 'acoustic_filtered': acoustic_filtered_data}
                pub_sock.send_multipart([zmq_topics.topic_gps, pickle.dumps(gps_pub_data, -1)])
        except:
            print("Error...")
            time.sleep(5)