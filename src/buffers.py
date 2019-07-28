import time
from serialization import pack, unpack

def name(key):
    return key[0]

def ip(key):
    return key[1]

def sid(key):
    return key[2]

def obj(key):
    return key[3]

def register_remote_buffers(client, keys):
    f = lambda x: client.registerRemoteBuffer(name(x), ip(x), sid(x))
    list(map(f, keys))
    time.sleep(0.5)

def update_remote_buffer(client, key):
    data, active = client.getRemoteBufferContents(name(key), ip(key), sid(key))
    if not active:
        print("WARNING: ", name(key), " not active")
    return unpack(obj(key), data)

def update_remote_buffers(client, keys):
    d = {}
    for key in keys:
        d[name(key)] = update_remote_buffer(client, key)
    return d
