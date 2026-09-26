import socket
import cv2
import numpy as np

# End-to-end TCP protocol test client for visionn_server.py -- unlike
# visionn_test.py/visionn_test2.py (which call visionn_model.py directly, in
# process), this exercises the actual VISIONN_HELLO/VISIONN_IMPRQ/VISIONN_IMPRS
# wire protocol over a real socket connection, the same way the legacy C++
# client (doc/istrobtx_2025/visionn_server/visionn.cpp/.h) talks to the
# server. Run visionn_server.py first, then run this against it.

HOST = '127.0.0.1'
PORT = 7001

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

def recv_exact(n):
    data = b''
    while len(data) < n:
        chunk = s.recv(n - len(data))
        if not chunk:
            raise RuntimeError("connection closed")
        data += chunk
    return data

def recv_skip(ch):
    while True:
        c = s.recv(1)
        if c == ch:
            return

# HELLO
s.sendall(b'{"VISIONN_HELLO":"client"}\n')
resp = s.recv(1024)
print("HELLO resp:", resp)
assert resp.startswith(b'{"VISIONN_HELLO":"server"}'), "unexpected HELLO response"

# IMPRQ
img = cv2.imread('img_input/testing_image.png')
buf = cv2.imencode('.png', img)[1].tobytes()
buflen = len(buf)
msg = b'{"VISIONN_IMPRQ":[' + f'{buflen:07d}'.encode() + b',"' + buf + b'"]}\n'
s.sendall(msg)

recv_skip(b'[')
resp_buflen = int(recv_exact(7).decode())
print("response buflen:", resp_buflen)
recv_exact(2)  # skip ,"
respbuf = recv_exact(resp_buflen)
recv_skip(b'}')

mask = cv2.imdecode(np.frombuffer(respbuf, dtype='uint8'), cv2.IMREAD_UNCHANGED)
print("decoded mask shape:", mask.shape, "dtype:", mask.dtype, "unique values:", np.unique(mask))
cv2.imwrite('img_output/visionn_test3_mask.png', mask)
print("OK - saved img_output/visionn_test3_mask.png")

s.close()
