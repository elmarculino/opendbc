import unittest

from opendbc.can.packer import CANPacker
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.gwmcan import CanBus

CAMERA_STEER = {"BYPASSME_2": 0, "COUNTER_1": 0}


class TestMk4Counters(unittest.TestCase):
  def test_counter_wraps_at_15(self):
    # stock GWM counters run 0-14 and never send 15
    self.assertEqual([gwmcan.next_counter(c) for c in (0, 13, 14)], [1, 14, 0])

  def test_steer_command_never_sends_15(self):
    packer = CANPacker('gwm_haval_h6_mk4_generated')
    can = CanBus(None, fingerprint={0: {}, 1: {}, 2: {}})
    for cam in range(15):
      dat = gwmcan.create_steer_command_angle(packer, can, CAMERA_STEER | {"COUNTER_1": cam}, 0.0, True)[1]
      self.assertEqual(dat[15] & 0x0F, (cam + 1) % 15)  # COUNTER_1
      self.assertEqual(dat[23] & 0x0F, (cam + 1) % 15)  # COUNTER_2


if __name__ == "__main__":
  unittest.main()
