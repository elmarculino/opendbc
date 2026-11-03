import unittest
from types import SimpleNamespace

from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.carcontroller import CarController, MK4_HUD_STANDSTILL_SPEED
from opendbc.car.gwm.gwmcan import CanBus


def _cc(enabled, long_active, lat_active=None):
  return SimpleNamespace(enabled=enabled, longActive=long_active, latActive=enabled if lat_active is None else lat_active)


def _cs(v_ego):
  return SimpleNamespace(out=SimpleNamespace(vEgo=v_ego))


class TestMk4QuietCancel(unittest.TestCase):
  def setUp(self):
    self.cc = CarController.__new__(CarController)
    self.cc.is_mk4 = True
    self.cc.enabled_prev = False
    self.cc.long_active_prev = False
    self.cc.cancel_demote_frames = 0
    self.cc.lkas_standby_frames = 0
    self.cc.exit_chime_frames = 0
    self.cc.exit_flag_seen = False

  def _step(self, enabled, long_active, gas=False, n=1, cam_deactivated=False, exit_flag=False):
    for _ in range(n):
      self.cc.update_quiet_cancel(_cc(enabled, long_active), gas, cam_deactivated, exit_flag)

  def test_exit_chime_hidden_until_camera_clears_it(self):
    # route 1bc: brake drops ACC, camera raises b17 0x04 ~0.2 s later and drops it ~0.5-2 s after release
    self._step(True, True, n=5)
    self._step(True, False)
    self._step(True, False, n=20)
    self.assertGreater(self.cc.exit_chime_frames, 0)
    self._step(True, False, exit_flag=True, n=400)
    self.assertGreater(self.cc.exit_chime_frames, 0)
    self._step(True, False)
    self.assertEqual(self.cc.exit_chime_frames, 0)

  def test_exit_chime_after_stalk_up(self):
    self._step(True, True, n=5)
    self._step(False, False, exit_flag=True, n=100)
    self.assertGreater(self.cc.exit_chime_frames, 0)

  def test_exit_chime_mask_is_capped(self):
    self._step(True, True, n=5)
    self._step(True, False, exit_flag=True, n=3001)
    self.assertEqual(self.cc.exit_chime_frames, 0)

  def test_exit_chime_clears_on_resume_and_skips_gas(self):
    self._step(True, True, n=5)
    self._step(True, False, exit_flag=True, n=5)
    self._step(True, True)
    self.assertEqual(self.cc.exit_chime_frames, 0)
    self._step(True, False, gas=True, exit_flag=True, n=5)
    self.assertEqual(self.cc.exit_chime_frames, 0)

  def test_lat_only_brake_does_not_arm_exit_chime(self):
    self._step(True, False, n=300)
    self._step(True, False, exit_flag=True, n=5)
    self.assertEqual(self.cc.exit_chime_frames, 0)

  def test_mads_brake_arms_acc_demote_only(self):
    self._step(True, True, n=5)
    self._step(True, False)  # brake: MADS keeps lateral, only ACC drops
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertEqual(self.cc.lkas_standby_frames, 0)

  def test_gas_override_does_not_arm(self):
    self._step(True, True, n=5)
    self._step(True, False, gas=True)
    self.assertEqual(self.cc.cancel_demote_frames, 0)

  def test_full_disengage_arms_both(self):
    self._step(True, True, n=5)
    self._step(False, False)
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertGreater(self.cc.lkas_standby_frames, 0)

  def test_full_disengage_from_lat_only_arms_both(self):
    self._step(True, False, n=5)
    self._step(False, False)
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertGreater(self.cc.lkas_standby_frames, 0)

  def test_demote_expires_and_resume_clears(self):
    self._step(True, True, n=5)
    self._step(True, False, n=201)
    self.assertEqual(self.cc.cancel_demote_frames, 0)
    self._step(True, False)
    self.assertEqual(self.cc.cancel_demote_frames, 0)  # no new edge
    self._step(True, True, n=5)
    self._step(True, False)
    self._step(True, True)
    self.assertEqual(self.cc.cancel_demote_frames, 0)

  def test_mask_holds_while_brake_keeps_camera_deactivated(self):
    # route 1b5: camera sat at 0x0a for 2.4-4.1 s with the pedal down; the 2 s grace alone leaked it
    self._step(True, True, n=5)
    self._step(True, False, cam_deactivated=True, n=500)
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self._step(True, False, cam_deactivated=False)  # brake released: camera moves to 0x12 itself
    self._step(True, False, n=200)
    self.assertEqual(self.cc.cancel_demote_frames, 0)

  def test_long_hold_does_not_bank_the_grace(self):
    # a 60 s brake hold after UP must not leave a fresh 2 s grace for whatever the camera sends next
    self._step(True, True, n=5)
    self._step(False, False, cam_deactivated=True, n=6000)
    self._step(False, False)  # brake released
    self.assertEqual(self.cc.cancel_demote_frames, 0)

  def test_mask_holds_after_stalk_up_with_brake(self):
    self._step(True, True, n=5)
    self._step(False, False, cam_deactivated=True, n=500)
    self.assertGreater(self.cc.cancel_demote_frames, 0)

  def test_lat_only_brake_is_masked_without_an_edge(self):
    self._step(True, False, n=300)
    self._step(True, False, cam_deactivated=True)
    self.assertGreater(self.cc.cancel_demote_frames, 0)

  def test_disengaged_without_a_cancel_follows_the_camera(self):
    self._step(False, False, n=300)
    self._step(False, False, cam_deactivated=True, n=10)
    self.assertEqual(self.cc.cancel_demote_frames, 0)

  def test_lkas_standby_is_a_short_bridge(self):
    # route 1b9: the cluster draws LKAS_STATE 1 as a red lane icon, so it must not stay up after UP
    self._step(True, False, n=5)
    self._step(False, False, n=199)
    self.assertGreater(self.cc.lkas_standby_frames, 0)
    self._step(False, False, n=2)
    self.assertEqual(self.cc.lkas_standby_frames, 0)

  def test_lkas_standby_clears_on_engage(self):
    self._step(True, False, n=5)
    self._step(False, False, n=50)
    self._step(True, False)
    self.assertEqual(self.cc.lkas_standby_frames, 0)

def _raw(b17=0x02, b18=0x1a):
  b = bytearray(64)
  b[17], b[18] = b17, b18
  return bytes(b)


class TestMk4ClusterFrames(unittest.TestCase):
  CAN = CanBus(None, fingerprint={0: {}, 1: {}, 2: {}})

  def _hud(self, **kw):
    return gwmcan.create_hud_command(None, self.CAN, None, is_mk4=True, hud_stock_raw=_raw(), **kw)[1]

  def test_lkas_states(self):
    self.assertEqual(self._hud(steer_required=True)[17] & 0x38, 0x28)
    self.assertEqual(self._hud(steer_required=False, lkas_standby=True)[17] & 0x38, 0x08)
    self.assertEqual(self._hud(steer_required=False)[17] & 0x38, 0x00)

  def test_lkas_standby_keeps_camera_warnings(self):
    raw = _raw(b17=0x02 | (3 << 3))  # drift_out_left from the camera
    out = gwmcan.create_hud_command(None, self.CAN, None, False, is_mk4=True, hud_stock_raw=raw, lkas_standby=True)[1]
    self.assertEqual(out[17] & 0x38, 3 << 3)

  def test_acc_mask_shows_available_not_deactivated(self):
    out = gwmcan.create_acc_cluster_mk4(self.CAN, _raw(b18=0x0a), cancel_demote=True)[1]
    self.assertEqual(out[18], 0x12)
    self.assertEqual(out[16], gwmcan.checksum(out[17:24], 0x40))
    self.assertEqual(gwmcan.create_acc_cluster_mk4(self.CAN, _raw(b18=0x0a))[1][18], 0x0a)

  def test_exit_chime_flag_hidden(self):
    raw = _raw(b17=0x84, b18=0x0a)
    out = gwmcan.create_acc_cluster_mk4(self.CAN, raw, cancel_demote=True, hide_exit_chime=True)[1]
    self.assertEqual(out[17], 0x80)
    self.assertEqual(out[16], gwmcan.checksum(out[17:24], 0x40))
    self.assertEqual(gwmcan.create_acc_cluster_mk4(self.CAN, raw)[1][17], 0x84)


class TestMk4HudLkas(unittest.TestCase):
  def setUp(self):
    self.cc = CarController.__new__(CarController)
    self.cc.is_mk4 = True

  def test_lkas_held_at_standstill_while_engaged(self):
    self.assertTrue(self.cc.hud_steer_required(_cc(True, False, lat_active=False), _cs(0.)))

  def test_lkas_follows_lat_active_when_moving(self):
    self.assertFalse(self.cc.hud_steer_required(_cc(True, False, lat_active=False), _cs(MK4_HUD_STANDSTILL_SPEED + 1.)))
    self.assertTrue(self.cc.hud_steer_required(_cc(True, False, lat_active=True), _cs(10.)))

  def test_lkas_off_when_disengaged(self):
    self.assertFalse(self.cc.hud_steer_required(_cc(False, False), _cs(0.)))

  def test_mk3_unchanged(self):
    self.cc.is_mk4 = False
    self.assertFalse(self.cc.hud_steer_required(_cc(True, False, lat_active=False), _cs(0.)))


if __name__ == "__main__":
  unittest.main()
