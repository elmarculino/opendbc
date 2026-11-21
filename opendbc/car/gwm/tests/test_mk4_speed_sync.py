"""MK4 set-speed sync: the cluster shows the camera's own set speed (0x2AB passed through untouched), which only moves
5 km/h per wheel-scroll click. After openpilot raises its set speed on a gas-pedal release, UP clicks (0x254) are sent
to the camera until it catches up (route 1e8: the cluster kept the old value).
"""
import unittest
from types import SimpleNamespace

from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.carcontroller import (CarController, MK4_SPEED_SYNC_GAP_FRAMES, MK4_SPEED_SYNC_MAX_CLICKS)
from opendbc.car.gwm.gwmcan import CanBus

ACTIVE, AVAILABLE, GAS_OVERRIDE = 0x1A, 0x12, 0x2A


def _cc(set_kph, long_active=True):
  hud = SimpleNamespace(setSpeed=set_kph / 3.6, speedVisible=True)
  return SimpleNamespace(longActive=long_active, hudControl=hud)


def _raw(cam_kph, state=ACTIVE):
  b = bytearray(64)
  b[18], b[22] = state, cam_kph
  return bytes(b)


class TestMk4SpeedSync(unittest.TestCase):
  def setUp(self):
    self.cc = CarController.__new__(CarController)
    self.cc.speed_sync_wait = 0
    self.cc.speed_sync_clicks = 0
    self.cc.speed_sync_target = None

  def _drive(self, op_kph, cam_kph, frames=2000, state=ACTIVE, gas=False, long_active=True, react=True):
    """Camera steps to the next multiple of 5 one frame after each click. Returns (clicks, final camera speed)."""
    clicks = 0
    for _ in range(frames):
      if self.cc.speed_sync_click(_cc(op_kph, long_active), _raw(cam_kph, state), gas):
        clicks += 1
        if react:
          cam_kph = (cam_kph // 5 + 1) * 5
    return clicks, cam_kph

  def test_gas_raise_is_clicked_up(self):
    self.assertEqual(self._drive(85, 60), (5, 85))

  def test_off_grid_camera_lands_on_grid(self):
    self.assertEqual(self._drive(45, 34), (3, 45))  # 34 -> 35 -> 40 -> 45 (1e8 28:30: two clicks took 34 to 40)

  def test_in_sync_or_camera_ahead_sends_nothing(self):
    self.assertEqual(self._drive(80, 80)[0], 0)
    self.assertEqual(self._drive(25, 30)[0], 0)  # low-speed engage: openpilot 25, camera floor 30
    self.assertEqual(self._drive(82, 80)[0], 0)  # never overshoot past openpilot

  def test_only_while_engaged_and_camera_active(self):
    self.assertEqual(self._drive(85, 60, gas=True)[0], 0)
    self.assertEqual(self._drive(85, 60, long_active=False)[0], 0)
    self.assertEqual(self._drive(85, 60, state=AVAILABLE)[0], 0)
    self.assertEqual(self._drive(85, 60, state=GAS_OVERRIDE)[0], 0)

  def test_clicks_are_spaced(self):
    n, _ = self._drive(145, 30, frames=MK4_SPEED_SYNC_GAP_FRAMES * 3, react=True)
    self.assertLessEqual(n, 3)

  def test_gives_up_if_the_camera_ignores_clicks(self):
    n, _ = self._drive(100, 60, frames=MK4_SPEED_SYNC_GAP_FRAMES * (MK4_SPEED_SYNC_MAX_CLICKS + 10), react=False)
    self.assertEqual(n, MK4_SPEED_SYNC_MAX_CLICKS)

  def test_click_frame(self):
    addr, dat, bus = gwmcan.create_speed_up_click_mk4(CanBus(None, fingerprint={0: {}, 1: {}, 2: {}}))
    self.assertEqual((addr, dat, bus), (0x254, bytes([0, 0, 0x10, 0, 0, 0, 0, 0]), 2))


if __name__ == "__main__":
  unittest.main()
