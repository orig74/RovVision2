# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import zmq
import asyncio
import time
import pickle
import sys

sys.path.append('..')
sys.path.append('../utils')
import zmq_wrapper as utils
import zmq_topics
import hw_stats_tools

pub_sock = utils.publisher(zmq_topics.topic_hw_stats_port)

while 1:
    stats = hw_stats_tools.get_hw_stats()
    pub_sock.send_multipart([zmq_topics.topic_hw_stats,pickle.dumps((time.time(),stats))])
    time.sleep(5)
