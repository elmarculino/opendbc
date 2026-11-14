"""MK4 gas/brake anti-dither: holding set speed must not flip BRAKE_OR_GAS_REQ on planner noise.

Route 1d4: 405 REQ 12<->13 flips in 10 min engaged (every 0.1-0.4 s at 60-66 km/h on +/-0.01 m/s^2),
each one a +/-0.2-0.3 m/s^2 jolt. The fix must not bring back the 13e-146 OEM ACC fault: a brake
request held at the -41 brake-off baseline while accel >= 0.
"""
import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CAR, DBC
from opendbc.car.gwm.carcontroller import CarController, MK4_BRAKE_ON_MS2

V = 60 / 3.6
NOISE = [0.014, 0.005, -0.008, -0.008, 0.001, 0.02, 0.043, -0.002, -0.013, -0.0, -0.003, 0.003, 0.03, -0.024]


class TestMk4LongDither(unittest.TestCase):
  def setUp(self):
    self.cc = CarController.__new__(CarController)
    self.cc.brake_mode = False

  def _run(self, accels, v=V, active=True, stopping=False):
    return [self.cc.long_braking(a, active, stopping, v) for a in accels]

  def test_cruise_noise_stays_in_gas_mode(self):
    self.assertEqual(self._run(NOISE * 10), [False] * len(NOISE) * 10)

  def test_real_decel_brakes(self):
    self.assertEqual(self._run([0.0, MK4_BRAKE_ON_MS2, -0.5]), [False, True, True])

  def test_brake_mode_holds_until_accel_non_negative(self):
    self.assertEqual(self._run([-0.3, -0.05, -0.01, 0.0, -0.01]), [True, True, True, False, False])

  def test_never_brake_request_with_accel_non_negative(self):
    # the 13e-146 fault: REQ=13 held at the brake-off baseline
    seq = [-0.3, 0.0, 0.02, -0.5, 0.001, -0.09, 0.05] + NOISE * 5
    for a, braking in zip(seq, self._run(seq), strict=True):
      if a >= 0:
        self.assertFalse(braking, a)

  def test_low_speed_and_stopping_use_plain_sign(self):
    self.assertEqual(self._run([0.01, -0.01, 0.01], v=3.0), [False, True, False])
    self.assertEqual(self._run([0.01, -0.01, 0.01], stopping=True), [False, True, False])

  def test_inactive_resets(self):
    self._run([-0.5])
    self.assertTrue(self.cc.brake_mode)
    self._run([-0.5], active=False)
    self.assertFalse(self.cc.brake_mode)
    self.assertEqual(self._run([-0.01]), [False])

  def test_gas_mode_in_band_sends_zero_gas(self):
    # inside the band the frame is gas mode with GAS_CMD 0, the frame already sent at accel == 0
    packer = CANPacker(DBC[CAR.GWM_HAVAL_H6_MK4][Bus.pt])
    CAN = gwmcan.CanBus(None, fingerprint={0: {}, 1: {}, 2: {}})
    stock = {s: 0 for s in ("SPEED_REAL", "COUNTER_BRAKE", "BYPASSME_2", "BYPASS_ACC1", "COUNTER_ACC",
                             "BRAKE_GAS_STATE_2", "BRAKE_GAS_STATE", "BRAKE_OR_GAS_REQ", "STANDSTILL")}

    def frame(accel, braking):
      d = gwmcan.create_longitudinal_command(packer, CAN, stock, accel=accel, active=True, standstill=False,
                                             is_mk4=True, regen=False, braking=braking)[1]
      return d[9], 181 - d[13], (((d[27] & 0x1F) << 8) | d[28]) - 192  # REQ, brake_raw, gas_raw (as gwm.h)

    self.assertEqual(frame(-0.01, False), frame(0.0, False))
    self.assertEqual(frame(-0.01, False), (12, 41, 0))


if __name__ == "__main__":
  unittest.main()
