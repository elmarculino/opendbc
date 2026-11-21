"""MK4 personality follows the camera's follow-distance dash (0x2AB CAR_DISTANCE_SELECTION).

Route 1e8: the wheel distance buttons moved the camera's dash up/down while openpilot cycled its personality one way
on any press, so the cluster dashes and the personality drifted apart. With the camera's 0x2AB passed to the
cluster untouched, the dash is the source of truth: openpilot steps its personality (one release per gap-adjust
pulse, personality - 1 mod 3) until leadDistanceBars matches the dash (4 folded to 3).
"""
import unittest

from opendbc.car.gwm.interface import CarInterface
from opendbc.car.structs import CarState

GAP = CarState.ButtonEvent.Type.gapAdjustCruise


class TestMk4PersonalitySync(unittest.TestCase):
  def setUp(self):
    self.ci = CarInterface.__new__(CarInterface)
    self.ci.frame = 0
    self.ci.press_gac_button = False
    self.ci.last_gac_press_frame = -CarInterface.GAC_SYNC_INTERVAL
    self.ci.current_personality = 2  # leadDistanceBars

  def _run(self, dash, frames=300, react=True):
    """Simulate openpilot: on each gap-adjust release, bars cycle 3 -> 2 -> 1 -> 3 (personality - 1 mod 3)."""
    releases = 0
    self.ci.pcm_follow_distance = dash
    for _ in range(frames):
      for be in self.ci.personality_sync_events():
        self.assertEqual(be.type, GAP)
        if not be.pressed:
          releases += 1
          if react:
            self.ci.current_personality = (self.ci.current_personality - 2) % 3 + 1
    return releases

  def test_follows_each_dash(self):
    for dash, bars in ((1, 1), (2, 2), (3, 3), (4, 3), (2, 2), (1, 1), (3, 3)):
      self._run(dash)
      self.assertEqual(self.ci.current_personality, bars, dash)

  def test_in_sync_sends_nothing(self):
    self.assertEqual(self._run(2), 0)

  def test_uninitialized_dash_is_ignored(self):
    self.assertEqual(self._run(0), 0)
    self.assertEqual(self.ci.current_personality, 2)

  def test_pulses_are_rate_limited(self):
    # openpilot not reacting yet: at most one press per GAC_SYNC_INTERVAL frames
    n = self._run(1, frames=100, react=False)
    self.assertLessEqual(n, 100 // CarInterface.GAC_SYNC_INTERVAL + 1)
    self.assertGreater(n, 0)


if __name__ == "__main__":
  unittest.main()
