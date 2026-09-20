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

  def test_standstill_emits_no_gesture_at_all(self):
    # openpilot's car_events suppresses preEnableStandstill ("Release Brake to Engage") for this
    # platform so lateral can engage with a foot on the brake. That is only safe because no gesture
    # -- engage or lkas -- can be produced below 0.5 m/s. Keep the two in lockstep.
    for v_ego in (0.0, 0.4, -0.4):
      with self.subTest(v_ego=v_ego):
        engage, lkas, latch, fired = _step(True, True, gear_d=True, v_ego=v_ego)
        self.assertFalse(latch)
        self.assertEqual(engage, 0)
        self.assertEqual(lkas, 0)
        # gentle release of the same never-latched gesture must stay silent too
        _, lkas_release, _, _ = _step(False, False, prev_enable=True, latch=latch, fired=fired)
        self.assertEqual(lkas_release, 0)

  def test_gesture_latches_once_moving(self):
    _, _, latch, _ = _step(True, True, gear_d=True, v_ego=0.6)
    self.assertTrue(latch)

  def test_latch_decided_at_press_edge_survives_the_hold(self):
    # The latch is evaluated once, on the gesture's rising edge, and deliberately NOT re-checked
    # while the stalk is held: the engagement decision was legitimate when the driver made it.
    # Pinning this so a later change to the hold behaviour is a conscious one.
    engage, _, latch, fired = _step(True, True, gear_d=True, v_ego=10.0)
    self.assertTrue(latch)
    self.assertEqual(engage, 1)
    # car comes to a stop / leaves D mid-hold: the latch is kept, the gesture stays live
    engage, _, latch, fired = _step(True, True, gear_d=False, v_ego=0.0,
                                    prev_enable=True, latch=latch, fired=True, prev_engage=1)
    self.assertTrue(latch)
    self.assertEqual(engage, 1)
    # ...but a fresh press under those same conditions latches nothing
    _, _, latch, _ = _step(True, True, gear_d=False, v_ego=0.0)
    self.assertFalse(latch)


class TestMk4ButtonEnable(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    try:
      from opendbc.car.gwm.carstate import CarState
      from opendbc.car import structs
    except ImportError as e:
      raise unittest.SkipTest(str(e))
    cls.CarState = CarState
    cls.ButtonType = structs.CarState.ButtonEvent.Type

  def _cs(self):
    obj = self.CarState.__new__(self.CarState)
    obj.CP = type("CP", (), {"pcmCruise": False})()
    return obj

  def _be(self, typ, pressed):
    return type("BE", (), {"type": typ, "pressed": pressed})()

  def test_detent_press_enables(self):
    cs = self._cs()
    self.assertTrue(cs.update_button_enable([self._be(self.ButtonType.setCruise, True)]))
    self.assertFalse(cs.update_button_enable([self._be(self.ButtonType.setCruise, False)]))

  def test_wheel_does_not_enable(self):
    cs = self._cs()
    for typ in (self.ButtonType.accelCruise, self.ButtonType.decelCruise):
      for pressed in (True, False):
        self.assertFalse(cs.update_button_enable([self._be(typ, pressed)]))

  def test_lkas_does_not_enable(self):
    cs = self._cs()
    self.assertFalse(cs.update_button_enable([self._be(self.ButtonType.lkas, True)]))


if __name__ == "__main__":
  unittest.main()
