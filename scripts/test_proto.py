#! /usr/bin/python
import trajectory_pb2
import zlib

f = open('/home/agoy/Documents/denso_calib_2/denso_calib_2_0.pbstream','r');
# f = open('/home/agoy/Documents/denso_calib_2/test.pbstream','r');
# f = open('/home/agoy/Documents/denso_calib_2/zlib.pbstream','r');
fs = f.read()
s = fs[0:8];
print list(bytearray(s))
print map(hex,bytearray(s))

# ignore those magic bytes already

# size is size_t 8 bytes
s = fs[8:16]
print list(bytearray(s))
print map(hex,bytearray(s))

# s = fs[16:54829+16]
s = fs[16:]
print len(s)
# # protobuf_stream = zlib.decompress(s, 15 + 32)
# protobuf_stream = zlib.decompress(s, zlib.MAX_WBITS | 16)
# protobuf_stream = zlib.decompress(fs[16:221616], zlib.MAX_WBITS | 16)
# with autoheader detection
protobuf_stream = zlib.decompress(s, zlib.MAX_WBITS | 32)

print len(protobuf_stream)
# print len(fs[16:])

traj = trajectory_pb2.Trajectory()

traj.ParseFromString(protobuf_stream)
# traj.ParseFromString(fs[16:])
# traj.ParseFromString(zlib.decompress(fs[16:]))

print len(traj.node)
print type(traj)

# traj.node.add()
# print len(traj.node)

for t in traj.node:
    print t.timestamp, t.pose
