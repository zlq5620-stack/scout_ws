"""Explicit standby-only CAN selection; fake transport, no physical sockets."""
import sys,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scout_control.autonomy_core import CAN_FRAME,encode_can_mode,ControlLatch
from scout_control.can_transport import CanTransport
try:
    from scout_control.autonomy_controller import AutonomyController
except ImportError as error:raise unittest.SkipTest('Source ROS Humble: '+str(error))


class ConfigurationTests(unittest.TestCase):
    def node(self,mode=0,hardware=True):
        return SimpleNamespace(latch=ControlLatch(hardware),mode_request_until=None,
          can=SimpleNamespace(mode=mode,healthy=lambda _:True,motion=(0.,0.),receive=Mock(),request_can_mode=Mock()))
    def test_encoder_and_zero_mode_zero_order(self):
        ident,n,p=CAN_FRAME.unpack(encode_can_mode());self.assertEqual((ident,n,p),(0x421,8,b'\x01'+bytes(7)))
        transport=object.__new__(CanTransport);transport.sock=Mock();transport.sock.send.return_value=16;transport.sent=0;transport.io_error=None
        transport.request_can_mode()
        frames=[CAN_FRAME.unpack(c.args[0]) for c in transport.sock.send.call_args_list]
        self.assertEqual([f[0] for f in frames],[0x111,0x421,0x111])
        self.assertEqual(frames[0][2],bytes(8));self.assertEqual(frames[-1][2],bytes(8));self.assertEqual(transport.sent,3)
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=10.)
    def test_standby_request_does_not_arm(self,_):
        n=self.node();r=SimpleNamespace();AutonomyController.select_can_mode(n,None,r)
        self.assertTrue(r.success);self.assertFalse(n.latch.armed);n.can.request_can_mode.assert_called_once()
        self.assertEqual(n.latch.output(10.1,0,True),(True,(0.,0.)))
        n.can.mode=1;AutonomyController.update_mode_request(n)
        self.assertEqual(n.latch.state,'CAN_MODE_READY');self.assertFalse(n.latch.armed);self.assertIsNone(n.mode_request_until)
    def test_manual_mode_is_never_overridden(self):
        n=self.node(3);r=SimpleNamespace();AutonomyController.select_can_mode(n,None,r)
        self.assertFalse(r.success);n.can.request_can_mode.assert_not_called();self.assertFalse(n.latch.armed)
    def test_disabled_moving_fault_and_armed_are_rejected(self):
        cases=[self.node(hardware=False),self.node(),self.node(),self.node()]
        cases[1].can.motion=(.02,0.);cases[2].can.healthy=lambda _:False;cases[3].latch.arm(10.)
        for n in cases:
            r=SimpleNamespace();AutonomyController.select_can_mode(n,None,r)
            self.assertFalse(r.success);n.can.request_can_mode.assert_not_called()
    def test_existing_mode_is_idempotent(self):
        n=self.node(1);r=SimpleNamespace();AutonomyController.select_can_mode(n,None,r)
        self.assertTrue(r.success);n.can.request_can_mode.assert_not_called();self.assertFalse(n.latch.armed)
    @patch('scout_control.autonomy_controller.time.monotonic',return_value=12.)
    def test_unconfirmed_request_expires_without_arming(self,_):
        n=self.node();n.mode_request_until=11.;AutonomyController.update_mode_request(n)
        self.assertEqual(n.latch.state,'CAN_MODE_NOT_CONFIRMED');self.assertFalse(n.latch.armed)
    def test_can_mode_confirmation_keeps_zero_until_enable(self):
        n=self.node();n.mode_request_until=99.;n.latch.has_control=True;n.can.mode=1
        AutonomyController.update_mode_request(n)
        self.assertEqual(n.latch.output(10.,1,True),(True,(0.,0.)))

    def test_ready_is_revoked_when_mode_changes(self):
        for mode in (0,3):
            n=self.node(mode);n.latch.disarm('CAN_MODE_READY','previous confirmation');n.latch.has_control=True
            AutonomyController.update_mode_request(n)
            self.assertNotEqual(n.latch.state,'CAN_MODE_READY');self.assertFalse(n.latch.armed)
            send,command=n.latch.output(10.,mode,True);self.assertEqual(command,(0.,0.))
            if mode==3:self.assertFalse(send)
    def test_ready_is_revoked_on_stale_feedback(self):
        n=self.node(1);n.latch.disarm('CAN_MODE_READY','previous confirmation');n.can.healthy=lambda _:False
        AutonomyController.update_mode_request(n)
        self.assertEqual(n.latch.state,'CAN_NOT_READY');self.assertFalse(n.latch.armed)


if __name__=='__main__':unittest.main()
