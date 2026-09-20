import unittest

from opendbc.car.gwm.mk4_stalk import update_mk4_down_gestures


def _step(enable, further, *, gear_d=True, v_ego=10.0, prev_enable=False, latch=False, fired=False, prev_engage=0):
  return update_mk4_down_gestures(
    enable_gesture=enable, further=further, gear_d=gear_d, v_ego=v_ego,
    prev_enable_gesture=prev_enable, engage_latch=latch, gesture_fired=fired, prev_engage=prev_engage,
  )


class TestMk4Stalk(unittest.TestCase):
  def test_gentle_release_is_lkas(self):
    engage, lkas, latch, fired = _step(True, False, gear_d=True, v_ego=10.0)
    self.assertEqual(engage, 0)
    self.assertEqual(lkas, 0)
    self.assertTrue(latch)
    self.assertFalse(fired)

    engage, lkas, latch, fired = _step(False, False, prev_enable=True, latch=True, fired=False, prev_engage=0)
    self.assertEqual(engage, 0)
    self.assertEqual(lkas, 1)
    self.assertFalse(fired)

  def test_detent_is_engage_on_press(self):
    engage, lkas, latch, fired = _step(True, True, gear_d=True, v_ego=10.0)
    self.assertEqual(engage, 1)
    self.assertEqual(lkas, 0)
    self.assertTrue(fired)

  def test_sweep_through_gentle_does_not_lkas(self):
    engage, lkas, latch, fired = _step(True, False, gear_d=True, v_ego=10.0)
    self.assertFalse(fired)
    engage, lkas, latch, fired = _step(True, True, prev_enable=True, latch=True, fired=False, prev_engage=0)
    self.assertEqual(engage, 1)
    self.assertTrue(fired)
    engage, lkas, _, _ = _step(False, False, prev_enable=True, latch=True, fired=True, prev_engage=1)
    self.assertEqual(engage, 0)
    self.assertEqual(lkas, 0)

  def test_park_shift_does_not_latch(self):
    engage, lkas, latch, fired = _step(True, False, gear_d=False, v_ego=10.0)
    self.assertFalse(latch)
    engage, lkas, _, _ = _step(False, False, prev_enable=True, latch=False, fired=False)
    self.assertEqual(lkas, 0)

  def test_standstill_does_not_latch(self):
    _, _, latch, _ = _step(True, True, gear_d=True, v_ego=0.0)
    self.assertFalse(latch)


if __name__ == "__main__":
  unittest.main()
