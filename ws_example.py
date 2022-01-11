#!/usr/bin/python3

import threading
from socket import *
from base64 import b64encode
from hashlib import sha1


class ClientThread(threading.Thread):
    def __init__(self, sock):
        threading.Thread.__init__(self)
        self.s=sock
    def stop(self):
        self.running=0
        return self.join()
    def end_co(self):
        try:
            self.s.shutdown(2)
            self.s.close()
        except:
            pass
    def end_ws(self):
        self.end_co()
        print("end of websocket") ## debug
    def ws_send(self,data):
        opcode=130 # binary data
        l=len(data)
        if l<=125:
            b=bytes([opcode,l])
        elif l<2**16:
            b=bytes([opcode,126])+struct.pack('!H',l)
        else:
            b=bytes([opcode,127])+struct.pack('!Q',l)
        self.s.send(b+data)
    def ws_recv(self):
        opcode=130 # opcode for binary data (129=text)
        if(self.s.recv(1)[0]!=opcode): return -1
        l=self.s.recv(1)[0]&0x7f
        if l==126:
            l=struct.unpack('!H',self.s.recv(2))
        elif l==127:
            l=struct.unpack('!Q',self.s.recv(8))
        k=self.s.recv(4) # key
        data_enc=self.s.recv(l)
        if len(data_enc)!=l: return -2
        return bytes([data_enc[i]^k[i%4] for i in range(l)]) # xor with key
    def run(self):
        # parsing initial client request
        r=self.s.recv(1024)
        if b'upgrade:websocket' in r.lower().replace(b' ',b''):
            self.s.send(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n')
            k1=r.split(b'Sec-WebSocket-Key: ')[1].split(b'\r\n')[0]
            k2=b64encode(sha1(k1+b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest()) # no b64decode.. strange protocol
            print('websocket keys:',k1,k2) ## debug
            self.s.send(b'Sec-WebSocket-Accept: '+k2+b'\r\n\r\n')

            # now ws is open

            # ...
            self.ws_send(b"coucou")
            self.end_ws()

        else:
            print("got a non-websocket request: ",r.split(b'\r')[0],"...")
            self.end_co()


s0 = socket()
s0.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1) # easier rebind after crash
s0.bind(("0",8080))
s0.listen(1)

print("listen on 0:8080")

for i in range(5):
    # wait for client to connect:
    sock, addr = s0.accept()

    client = ClientThread(sock)
    client.start()

    client.join()


s0.shutdown(2)
s0.close()

print("finito")

