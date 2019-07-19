# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import zmq
import numpy as np
context = zmq.Context()

def subscribe(topics,port,ip='127.0.0.1'):
    zmq_sub = context.socket(zmq.SUB)
    zmq_sub.connect("tcp://%s:%d" % (ip,port))
    for topic in topics:
        zmq_sub.setsockopt(zmq.SUBSCRIBE,topic)
    return zmq_sub

def publisher(port,ip='127.0.0.1'):
    socket_pub = context.socket(zmq.PUB)
    socket_pub.bind("tcp://%s:%d" % (ip,port) )
    return socket_pub

def pull_sink(port,ip='127.0.0.1'):
    socket = context.socket(zmq.PULL)
    socket.bind("tcp://%s:%d" % (ip,port) )
    return socket

def push_source(port,ip='127.0.0.1'):
    socket = context.socket(zmq.PUSH)
    socket.connect("tcp://%s:%d" % (ip,port) ) 
    return socket


class ab_filt():
    def __init__(self,xv=[0,0],alpha=0.5,beta=0.1):
        self.alpha = alpha
        self.beta = beta
        self.reset(xv)

    def reset(self,xv):
        self.x,self.v=xv

    def __call__(self,xm,dt=1.0):
        self.x += self.v*dt

        rk = xm - self.x;

        self.x += self.alpha * rk;
        self.v += ( self.beta * rk ) / dt;
        return (self.x,self.v)



