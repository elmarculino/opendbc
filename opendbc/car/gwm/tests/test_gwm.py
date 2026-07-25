from opendbc.can import CANPacker
from opendbc.car.car_helpers import interfaces
from opendbc.car.gwm.values import CAR, DBC
from opendbc.car.structs import CarState

ButtonType = CarState.ButtonEvent.Type
PLATFORM = CAR.GWM_HAVAL_H6_MK4


class TestMK4StalkModes:
  """The gear stalk's two DOWN gestures map onto the car's two OEM modes (see carstate.py):
  gentle DOWN toggles MADS lateral, the hard FURTHER_DOWN detent engages everything.
  """

  def setup_method(self):
    CI = interfaces[PLATFORM]
    CP = CI.get_params(PLATFORM, {0: {}, 2: {}}, [], False, False, False)
    CP_SP = CI.get_params_sp(CP, PLATFORM, {0: {}, 2: {}}, [], False, False, False)
    self.CI = CI(CP, CP_SP)
    self.packer = CANPacker(DBC[PLATFORM]['pt'])
    self.t = 0
    self.settle()

  def step(self, stalk_down, further, speed=30.0, gear=1, brake=0):
    self.t += 10_000_000  # 10 ms
    wheel = speed / 0.05924739
    msgs = [
      self.packer.make_can_msg("GEAR_STALK", 0, {"STALK_DOWN": stalk_down, "STALK_FURTHER": further}),
      self.packer.make_can_msg("DRIVE_GEAR", 0, {"DRIVE_MODE_GEAR_REAL": gear}),
      self.packer.make_can_msg("BRAKE2", 0, {"PEDAL_BRAKE_PRESSED": brake}),
      self.packer.make_can_msg("WHEEL_SPEEDS", 0, {f"{c}_WHEEL_SPEED": wheel for c in
                                                   ("FRONT_LEFT", "FRONT_RIGHT", "REAR_LEFT", "REAR_RIGHT")}),
    ]
    CS, _ = self.CI.update([(self.t, msgs)])
    return [be.type for be in CS.buttonEvents if be.pressed], CS.cruiseState.available

  def settle(self, n=20, **kwargs):
    for _ in range(n):
      self.step(0, 0, **kwargs)

  def test_gentle_down_is_lateral_only(self):
    # fires on release, so a hard pull sweeping through the gentle position doesn't toggle MADS
    assert self.step(1, 0) == ([], False)
    for _ in range(4):
      self.step(1, 0)
    assert self.step(0, 0) == ([ButtonType.lkas], True)

  def test_further_down_engages_everything(self):
    assert self.step(1, 1) == ([ButtonType.decelCruise], True)
    assert self.step(0, 0)[0] == []

  def test_sweep_through_gentle_does_not_toggle_lateral(self):
    self.step(1, 0)
    self.step(1, 0)
    assert self.step(1, 1)[0] == [ButtonType.decelCruise]
    assert self.step(0, 0)[0] == []

  def test_stalk_down_at_standstill_is_ignored(self):
    # DOWN is also the N->D shift gesture, so it must not engage anything while stopped
    self.settle(speed=0.0)
    self.step(1, 0, speed=0.0)
    assert self.step(0, 0, speed=0.0) == ([], False)

  def test_brake_keeps_main_switch_latched(self):
    # main_on is the ACC MAIN switch: it survives the brake so MADS decides what happens to lateral
    self.step(1, 1)
    self.step(0, 0)
    assert self.step(0, 0, brake=1)[1]
    assert self.step(0, 0, brake=0)[1]
