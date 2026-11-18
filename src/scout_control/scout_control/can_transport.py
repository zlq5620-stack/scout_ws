"""SocketCAN feedback plus explicitly gated AGX_V2 motion and explicit mode transmission."""
import socket
import struct
import time
from .autonomy_core import CAN_FRAME, encode_motion, encode_can_mode


class CanTransport:
    def __init__(self, interface):
        self.sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        mask=socket.CAN_EFF_FLAG|socket.CAN_RTR_FLAG|socket.CAN_SFF_MASK
        self.sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FILTER,
            b''.join(struct.pack('=II',i,mask) for i in (0x211,0x221,0x111)))
        # Own frames must not trigger the other-writer detector.
        self.sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_RECV_OWN_MSGS,0)
        try:self.sock.bind((interface,))
        except OSError:self.sock.close();raise
        self.sock.setblocking(False)
        self.system=None;self.motion=None;self.system_at=0.;self.motion_at=0.
        self.conflict=False;self.io_error=None;self.sent=0

    def receive(self):
        for _ in range(200):
            try:raw=self.sock.recv(CAN_FRAME.size)
            except BlockingIOError:return
            except OSError as error:self.io_error=str(error);return
            self.ingest(raw,time.monotonic())

    def ingest(self,raw,now):
        if len(raw)!=CAN_FRAME.size:return
        can_id,dlc,data=CAN_FRAME.unpack(raw)
        if can_id&0xE0000000 or dlc!=8:return
        if can_id==0x111:
            self.conflict=True # another application is sending chassis commands
        elif can_id==0x211:
            state,mode,voltage,error=struct.unpack('>BBHH',data[:6])
            self.system=(state,mode,voltage/10.,error);self.system_at=now
        elif can_id==0x221:
            v,w,_,_=struct.unpack('>hhhh',data)
            self.motion=(v/1000.,w/1000.);self.motion_at=now

    def healthy(self,now):
        return (not self.io_error and not self.conflict and self.system is not None
                and self.motion is not None and 0<=now-self.system_at<.30
                and 0<=now-self.motion_at<.20 and self.system[0]==0 and self.system[3]==0)

    @property
    def mode(self):return self.system[1] if self.system else None

    def send(self,v,w):
        self._write(encode_motion(v,w))

    def request_can_mode(self):
        # Flush a stale speed request before activating the CAN interface and
        # immediately request zero again. Caller must enforce disarmed/standby.
        self.send(0.,0.)
        self._write(encode_can_mode())
        self.send(0.,0.)

    def _write(self,raw):
        try:
            if self.sock.send(raw)!=len(raw):raise OSError('Partial CAN write')
            self.sent+=1
        except OSError as error:self.io_error=str(error);raise

    def close(self):self.sock.close()
