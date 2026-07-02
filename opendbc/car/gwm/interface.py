from opendbc.car import structs, get_safety_config, Bus, create_button_events
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.gwm.carcontroller import CarController
from opendbc.car.gwm.carstate import CarState
from opendbc.car.gwm.values import GwmSafetyFlags

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  # frames between synthetic gap button pulses, so openpilot's personality
  # feedback (hudControl.leadDistanceBars) can round-trip before pulsing again
  GAC_SYNC_INTERVAL = 25

  def __init__(self, CP):
    super().__init__(CP)
    self.lat_active = False
    self.isEPSobeying = True
    self.steer_fault_temporary_counter = 0
    self.current_personality = 0
    self.pcm_follow_distance = 0
    self.press_gac_button = False
    self.frame = 0
    self.last_gac_press_frame = -self.GAC_SYNC_INTERVAL

  def apply(self, CC, now_nanos):
    self.lat_active = CC.latActive
    hud_control = CC.hudControl
    self.current_personality = hud_control.leadDistanceBars
    return super().apply(CC, now_nanos)

  def update(self, can_packets):
    cp = self.can_parsers[Bus.main]
    self.isEPSobeying = cp.vl["RX_STEER_RELATED"]["A_RX_STEER_REQUESTED"] == 1
    self.steer_fault_temporary_counter = (self.steer_fault_temporary_counter + 1) if (self.lat_active and not self.isEPSobeying) \
                                          else 0

    cp_cam = self.can_parsers[Bus.cam]
    self.pcm_follow_distance = cp_cam.vl["ACC"]["CAR_DISTANCE_SELECTION"]

    ret = super().update(can_packets)
    ret.steerFaultTemporary |= self.steer_fault_temporary_counter > 100

    # The stock ACC cycles through 4 follow distances while openpilot cycles through 3
    # personalities (distances 3 and 4 both map to the farthest personality). While they
    # disagree, pulse gap-adjust button presses so openpilot cycles its personality to
    # match the distance selected on the stalk.
    prev_gac_button = self.press_gac_button
    if self.press_gac_button:
      # openpilot cycles personality on button release, so always complete a press
      self.press_gac_button = False
    else:
      target_personality = min(int(self.pcm_follow_distance), 3)
      out_of_sync = self.pcm_follow_distance > 0 and target_personality != self.current_personality
      if out_of_sync and (self.frame - self.last_gac_press_frame) >= self.GAC_SYNC_INTERVAL:
        self.press_gac_button = True
        self.last_gac_press_frame = self.frame
    ret.buttonEvents = create_button_events(int(self.press_gac_button), int(prev_gac_button), {1: ButtonType.gapAdjustCruise})
    self.frame += 1

    return ret

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'gwm'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.gwm)]

    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.4
    ret.steerAtStandstill = False

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[-1].safetyParam |= GwmSafetyFlags.LONG_CONTROL.value

      ret.longitudinalActuatorDelay = 0.25
      ret.vEgoStopping = 0.25
      ret.vEgoStarting = 0.25
      ret.stopAccel = -0.75
      ret.stoppingDecelRate = 0.75
      ret.longitudinalTuning.kiBP = [0.]
      ret.longitudinalTuning.kiV = [0.4]

    return ret
