"""MK4 gear-stalk DOWN gestures: gentle = LKAS, detent = ACC+LKAS."""
from __future__ import annotations


def update_mk4_down_gestures(*, enable_gesture: bool, further: bool, gear_d: bool, v_ego: float,
                             prev_enable_gesture: bool, engage_latch: bool, gesture_fired: bool,
                             prev_engage: int) -> tuple[int, int, bool, bool]:
  """Return (engage, lkas, engage_latch, gesture_fired).

  Full engage on detent press. LKAS pulse on release of a press that never
  reached the detent, so a hard pull sweeping through gentle does not toggle LKAS.
  """
  if enable_gesture and not prev_enable_gesture:
    engage_latch = gear_d and abs(v_ego) > 0.5
    gesture_fired = False
  engage = int(enable_gesture and further and engage_latch)
  if engage and not prev_engage:
    gesture_fired = True
  lkas = int(prev_enable_gesture and not enable_gesture and engage_latch and not gesture_fired)
  return engage, lkas, engage_latch, gesture_fired
