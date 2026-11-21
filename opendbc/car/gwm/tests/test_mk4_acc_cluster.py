"""MK4 cluster ICC icon (0x2AB re-TX): shown while ACC is engaged, gas override included; hidden otherwise.

Route 1c8: while the driver held the gas with openpilot engaged, the re-TX passed the camera frame through
(0x12 "available" / 0x2a override), so the ICC icon and Vmax dropped while ACC was still on; with openpilot
off it showed the camera's stale set speed (115/101/93) as if ICC were on.
"""
import unittest
from types import SimpleNamespace

from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.gwmcan import CanBus


def _cc(enabled, long_active, set_kph=100.):
  hud = SimpleNamespace(setSpeed=set_kph / 3.6, speedVisible=True, leadDistanceBars=2)
  return SimpleNamespace(enabled=enabled, longActive=long_active, latActive=enabled, hudControl=hud)


def _raw(b17=0x02, b18=0x12, b21=0x22, b22=115):
  b = bytearray(64)
  b[17], b[18], b[21], b[22] = b17, b18, b21, b22
  return bytes(b)


class TestMk4AccCluster(unittest.TestCase):
  CAN = CanBus(None, fingerprint={0: {}, 1: {}, 2: {}})

  def setUp(self):
    self.cc = CarController.__new__(CarController)
    self.cc.is_mk4 = True
    self.cc.enabled_prev = False
    self.cc.long_active_prev = False
    self.cc.cancel_demote_frames = 0
    self.cc.lkas_standby_frames = 0
    self.cc.exit_chime_frames = 0
    self.cc.exit_flag_seen = False
    self.cc.acc_gas_override = False
    self.cc.acc_cluster_set_kph = None
    self.cc.acc_cluster_follow = None

  def _step(self, enabled, long_active, gas=False, n=1, set_kph=100.):
    for _ in range(n):
      CC = _cc(enabled, long_active, set_kph)
      self.cc.update_quiet_cancel(CC, gas)
      args = self.cc.acc_cluster_args(CC, v_ego=25.)
    return gwmcan.create_acc_cluster_mk4(self.CAN, _raw(), **args)[1]

  def test_engaged_passes_the_camera_frame_through(self):
    # 0x2AB carries a MAC (b56..63): any edited byte would invalidate it, so ICC-on frames are the camera's own
    out = self._step(True, True, n=5)
    self.assertEqual(out, _raw())

  def test_gas_override_passes_the_camera_frame_through(self):
    self._step(True, True, n=5)
    for _ in range(50):
      self.assertEqual(self._step(True, False, gas=True), _raw())
    self.assertTrue(self.cc.acc_gas_override)
    self.assertEqual(self._step(True, True), _raw())  # pedal released, ACC resumes

  def test_engage_with_gas_down_is_icc_on(self):
    # 1d4 36:36: engaged at 83 km/h with the pedal down; longActive never rose so the icon was hidden
    self._step(False, False, n=300)
    out = self._step(True, False, gas=True, n=20)
    self.assertTrue(self.cc.acc_gas_override)
    self.assertEqual(out, _raw())

  def test_edited_frame_never_sets_the_lead_flag(self):
    out = gwmcan.create_acc_cluster_mk4(self.CAN, _raw(), set_speed_kph=90, cruise_active=True)[1]
    self.assertEqual(out[17] & 0x80, 0)  # b17 0x80 = lead detected (camera: == b19 != 0)

  def test_gas_from_lat_only_does_not_show_icc(self):
    self._step(True, False, n=300)
    out = self._step(True, False, gas=True, n=20)
    self.assertNotEqual(out[18] & 0x3F, 0x1A)
    self.assertEqual(out[21] & 0x07, 0)

  def test_disengage_while_on_gas_drops_the_icon(self):
    self._step(True, True, n=5)
    self._step(True, False, gas=True, n=10)
    self._step(False, False, gas=True)
    self.assertFalse(self.cc.acc_gas_override)

  def test_brake_after_gas_is_a_normal_cancel(self):
    self._step(True, True, n=5)
    self._step(True, False, gas=True, n=10)
    out = self._step(True, False)  # pedal up, MADS brake keeps longActive off
    self.assertFalse(self.cc.acc_gas_override)
    self.assertNotEqual(out[18] & 0x3F, 0x1A)

  def test_openpilot_off_hides_icc(self):
    out = self._step(False, False, n=300)
    self.assertEqual(out[21] & 0x07, 0)  # 0 = OEM "disabled": no ICC icon / stale Vmax
    self.assertEqual(out[18], 0x12)      # state untouched: no new cluster transition
    self.assertEqual(out[16], gwmcan.checksum(out[17:24], 0x40))

  def test_lat_only_hides_icc(self):
    out = self._step(True, False, n=300)
    self.assertEqual(out[21] & 0x07, 0)

  def test_cancel_is_a_single_cluster_change(self):
    # route 1c8 replay: hiding only after the ~2 s grace showed the dashes again, a second change per cancel
    self._step(True, True, n=5)
    out = self._step(True, False)
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertEqual((out[18], out[21] & 0x07), (0x12, 0))
    out = self._step(True, False, n=200)
    self.assertEqual(self.cc.cancel_demote_frames, 0)
    self.assertEqual((out[18], out[21] & 0x07), (0x12, 0))


class TestMk4AccClusterFrame(unittest.TestCase):
  CAN = CanBus(None, fingerprint={0: {}, 1: {}, 2: {}})

  def test_hide_icc_clears_only_the_dashes(self):
    out = gwmcan.create_acc_cluster_mk4(self.CAN, _raw(b21=0x23), hide_icc=True)[1]
    self.assertEqual(out[21], 0x20)
    self.assertEqual(out[18], 0x12)
    self.assertEqual(out[22], 115)
    self.assertEqual(out[16], gwmcan.checksum(out[17:24], 0x40))

  def test_cruise_active_wins_over_hide(self):
    out = gwmcan.create_acc_cluster_mk4(self.CAN, _raw(), set_speed_kph=90, follow_dashes=2, cruise_active=True, hide_icc=True)[1]
    self.assertEqual(out[21] & 0x07, 2)
    self.assertEqual(out[18] & 0x3F, 0x1A)


if __name__ == "__main__":
  unittest.main()
