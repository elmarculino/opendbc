import unittest
from types import SimpleNamespace

from opendbc.car.gwm.carcontroller import CarController, MK4_HUD_STANDSTILL_SPEED


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
    self.cc.lkas_demote_frames = 0

  def _step(self, enabled, long_active, gas=False, n=1):
    for _ in range(n):
      self.cc.update_quiet_cancel(_cc(enabled, long_active), gas)

  def test_mads_brake_arms_acc_demote_only(self):
    self._step(True, True, n=5)
    self._step(True, False)  # brake: MADS keeps lateral, only ACC drops
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertEqual(self.cc.lkas_demote_frames, 0)

  def test_gas_override_does_not_arm(self):
    self._step(True, True, n=5)
    self._step(True, False, gas=True)
    self.assertEqual(self.cc.cancel_demote_frames, 0)

  def test_full_disengage_arms_both(self):
    self._step(True, True, n=5)
    self._step(False, False)
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertGreater(self.cc.lkas_demote_frames, 0)

  def test_full_disengage_from_lat_only_arms_both(self):
    self._step(True, False, n=5)
    self._step(False, False)
    self.assertGreater(self.cc.cancel_demote_frames, 0)
    self.assertGreater(self.cc.lkas_demote_frames, 0)

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
