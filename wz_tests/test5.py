import zmq
import ctypes
import struct
import random

def msg(nwords):
    res=b"WZDAQ1-D"  
    res+=struct.pack(">Q",nwords)
    for i in range(0,nwords):
        res += struct.pack(">Q",random.randint(0,(1<<64)-1))
    return res

ctx=zmq.Context()
sk=ctx.socket(zmq.PAIR)
sk.connect("tcp://localhost:3000")
#b=[ctypes.c_uint64(i) for i in [3,57,5]]
c=b"WZDAQ1-E"
c+=msg(1022)
sk.send(c)
c=msg(5133)
sk.send(c)
c=b"WZDAQ1-E"
c+=msg(31133)
c+=b"WZDAQ1-Q"
sk.send(c)



