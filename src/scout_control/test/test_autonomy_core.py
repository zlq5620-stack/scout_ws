import sys,unittest,math,struct
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scout_control.autonomy_core import ControlLatch,encode_motion,CAN_FRAME,rate_limit_command
from scout_control.can_transport import CanTransport


class ControlTests(unittest.TestCase):
    def test_default_no_writes(self):
        c=ControlLatch();self.assertFalse(c.arm(10));self.assertEqual(c.output(10,1,True),(False,(0.,0.)))
        c=ControlLatch(True);self.assertEqual(c.output(10,1,True),(False,(0.,0.)))
    def test_explicit_enable_and_expiry(self):
        c=ControlLatch(True);self.assertTrue(c.arm(10));self.assertEqual(c.output(10,1,True),(True,(0.,0.)))
        c.accept(10.1,.02,.08,10.3);self.assertEqual(c.output(10.2,1,True),(True,(.02,.08)))
        self.assertEqual(c.output(10.31,1,True),(True,(0.,0.)));self.assertFalse(c.armed)
        c.accept(10.4,.02,.08,10.6);self.assertFalse(c.armed)
    def test_fault_and_manual_override_latch(self):
        for mode,healthy in [(3,True),(0,True),(1,False)]:
            c=ControlLatch(True);c.arm(10);c.accept(10.01,.02,.08,10.3);send,cmd=c.output(10.1,mode,healthy)
            self.assertEqual(cmd,(0.,0.));self.assertFalse(c.armed)
            if mode==3:self.assertFalse(send)
            c.output(10.11,1,True);self.assertFalse(c.armed)
    def test_run_limit(self):
        c=ControlLatch(True,run_limit=1);c.arm(10);c.accept(10.9,.02,0.,11.2);self.assertEqual(c.output(11.01,1,True)[1],(0.,0.));self.assertEqual(c.state,'RUN_LIMIT')
    def test_encoder_limits(self):
        can_id,n,payload=CAN_FRAME.unpack(encode_motion(.12,-.4));self.assertEqual((can_id,n),(0x111,8));self.assertEqual(struct.unpack('>hhhh',payload),(120,-400,0,0))
        for v,w in [(math.nan,0),(-.01,0),(.121,0),(0,.401),(0,math.inf)]:
            with self.assertRaises(ValueError):encode_motion(v,w)
    def test_feedback_and_competing_writer(self):
        c=object.__new__(CanTransport);c.system=None;c.motion=None;c.system_at=c.motion_at=0.;c.io_error=None;c.conflict=False
        c.ingest(CAN_FRAME.pack(0x211,8,struct.pack('>BBHH',0,1,281,0)+b'\0\0'),10)
        c.ingest(CAN_FRAME.pack(0x221,8,struct.pack('>hhhh',0,0,0,0)),10)
        self.assertTrue(c.healthy(10.1));self.assertFalse(c.healthy(10.3))
        c.ingest(encode_motion(0,0),10.11);self.assertTrue(c.conflict);self.assertFalse(c.healthy(10.12))

    def test_slew_limits_and_delayed_callback_cap(self):
        for dt in (.1,.5,2.):
            v,w=rate_limit_command((.12,.4),(0.,0.),dt)
            self.assertAlmostEqual(v,.02);self.assertAlmostEqual(w,.08)
        v,w=rate_limit_command((.12,-.4),(.02,.08),.05)
        self.assertAlmostEqual(v,.03);self.assertAlmostEqual(w,.04)
        with self.assertRaises(ValueError):rate_limit_command((.12,.4),(0.,0.),-1)

    def test_pause_requests_only_zero_then_recovers_without_rearm(self):
        c=ControlLatch(True);c.arm(10.);c.accept(10.1,.02,.08,10.3)
        c.pause(10.2,'WAITING_FOR_SAFE_COMMAND','temporary zero',10.5)
        self.assertTrue(c.armed);self.assertEqual(c.output(10.3,1,True)[1],(0.,0.))
        c.accept(10.4,.02,.08,10.6);self.assertTrue(c.armed);self.assertIsNone(c.pause_started)
        self.assertEqual(c.output(10.5,1,True)[1],(.02,.08))
    def test_pause_cannot_be_refreshed_forever(self):
        c=ControlLatch(True);c.arm(10.)
        for now in [10.,10.2,10.4,10.6,10.8]:c.pause(now,'INPUT_RECOVERY_PAUSE','stale',now+.3)
        self.assertEqual(c.output(11.01,1,True)[1],(0.,0.));self.assertFalse(c.armed)
        self.assertEqual(c.state,'PAUSE_TIMEOUT')
        c.accept(11.1,.02,.08,11.3);self.assertFalse(c.armed)


if __name__=='__main__':unittest.main()
