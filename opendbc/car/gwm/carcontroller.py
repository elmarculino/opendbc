import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_steer_angle_limits_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.gwm import gwmcan
from opendbc.car.gwm.values import CarControllerParams, CAR

SteerControlType = structs.CarParams.SteerControlType

LongCtrlState = structs.CarControl.Actuators.LongControlState

MAX_USER_TORQUE = 100  # 1.0 Nm

# MK4 hands-on keepalive: driver torque we spoof to the camera/ADAS (via 0x147 on the camera bus) so it never
# runs its hands-off "hold the wheel" warning + safe-stop escalation and the EPS keeps steering hands-off.
# comma's DM camera is the attention monitor. The MK3 torque path does this dynamically (ea_simulated_torque =
# apply_torque*2). The first MK4 try used a FIXED 65 and the EPS still limped at highway speed (routes 110-112,
# verified transmitting @50Hz) -- because the OEM only recognizes "hands on" around |torque| > ~102, so 65 STILL
# read as hands-off. Fix: sit the FLOOR clearly above ~102 and scale up with the commanded angle like MK3 (more
# steering effort -> more apparent driver torque), capped below a real hard grab so a genuine override still
# passes through. Follows the commanded steer direction. Tunable on-road: raise the floor if it still limps,
# lower the cap if the ADAS flags implausible torque.
MK4_HANDS_ON_TORQUE = 120      # floor (was 65): above the OEM hands-on recognition point (~102)
MK4_HANDS_ON_TORQUE_MAX = 170  # cap: strong hands-on, still below the driver's real hard grabs (125-214)
MK4_HANDS_ON_ANGLE_GAIN = 8    # extra spoofed torque per deg of |apply_angle| (mimics MK3's dynamic scaling)

# MK4 carcontroller lat_active latch (NOT carstate.steeringPressed — that is the separate shared-code
# steerOverride event at torque>120). This path drops latActive for STEER_CMD only. Tuned for fully
# hands-off use. 100 sustained was the "soquinho" cause: the torque sensor reads EPS reaction torque
# up to ~134 hands-off in light curves (routes 56/57/58: >100 for 8-23% of curve time), so the latch
# false-tripped ~1x/36s engaged and released the wheel mid-curve for the 1 s hold. 100 Hz replay on
# route 1f: 130/170 keeps every real grab (2/2) with ZERO hands-off trips (sustained hands-off never
# exceeds 140; longest run >120 was 160 ms). Trade-off: a gentle grab below 130 sustained no longer
# hands off (the old reason for 100) — accepted, it cost a soquinho every ~36 s.
OVERRIDE_TORQUE = 130          # sustained |driver torque| to hand off; hands-off peaks (~134) are single frames, absorbed by debounce
OVERRIDE_INSTANT_TORQUE = 170  # firm grab -> release within one frame
OVERRIDE_FRAMES = 10           # ~100 ms @100 Hz
# Hold lat off ~1 s after override (OEM-style) so OP does not re-grab every torque dip.
OVERRIDE_HOLD_FRAMES = 100
# MK4 HUD: below this speed controlsd drops latActive (standstill), but the LKAS icon stays on while engaged.
MK4_HUD_STANDSTILL_SPEED = 0.5  # m/s (controlsd standstill gate is 0.3)


# MK4 regen-lead: BRAKE_GAS_STATE (regen enable) uses a light accel hysteresis so cruise chatter around
# 0 m/s^2 does not flip regen↔coast every frame (drive e6 jerk). BRAKE_OR_GAS_REQ / BRAKE_CMD still
# follow sign(accel) every frame — do NOT hold brake mode or pin GAS_CMD=0 across a deadband: that
# FAULTED the OEM ACC (drives 13e-146, "Cruise Fault: Restart the Car"). Regen MAGNITUDE stays the
# powertrain's call (driver Low/Normal/Heavy on the stalk selector); we only gate the enable state.
#
# Enter regen when accel <= REGEN_ON; leave when accel >= REGEN_OFF or long inactive. Band is small
# (~0.13 m/s^2) — enough for planner noise, not a deep coast-hold.
MK4_REGEN_ON_MS2 = -0.08
MK4_REGEN_OFF_MS2 = 0.05


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.packer = CANPacker(dbc_names[Bus.main])
    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.CAN = gwmcan.CanBus(CP)
    self.accel = 0.0
    self.is_mk4 = CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4
    self.VM = VehicleModel(CP)
    self.override_active = False   # MK4: debounced driver-override latch
    self.override_counter = 0
    self.override_hold = 0         # MK4: frames remaining in the post-override OEM-style hold-off
    self.regen_brake = False       # MK4: hysteretic REGEN state latch (BRAKE_GAS_STATE only; see MK4_REGEN_*)
    # OEM cluster (0x2AB): latch last good set-speed + follow dashes so the Vmax icon stays solid
    # while engaged (OEM: icon always on when ACC active). Avoid thrashing distance every frame.
    self.acc_cluster_set_kph: float | None = None
    self.acc_cluster_follow: int | None = None
    self.enabled_prev = False        # MK4: falling-edge detect for the quiet-cancel grace
    self.long_active_prev = False    # MK4: falling-edge detect for the MADS brake-cancel grace
    self.cancel_demote_frames = 0    # MK4: frames left masking the camera's 0x0a standby step post-cancel (0x2AB)
    self.lkas_standby_frames = 0     # MK4: frames left showing LKAS_STATE 1 (not 0) after a full disengage (0x23D)
    self.exit_chime_frames = 0       # MK4: frames left hiding the camera's ACC-exit chime flag (0x2AB b17 0x04)
    self.exit_flag_seen = False
    self.acc_gas_override = False    # MK4: gas pressed while ACC was engaged (cluster keeps showing ICC on)

  def update_quiet_cancel(self, CC, gas_pressed: bool, camera_deactivated: bool = False, camera_exit_flag: bool = False):
    # Quiet-cancel grace: on disengage, mask the camera's 0x1a->0x0a step on the 0x2AB re-TX (see
    # create_acc_cluster_mk4) and land on 0x12 ("available"). Re-engage clears it immediately.
    # MADS: a brake drops only longActive (enabled stays True for lateral), so the ACC grace must also
    # arm on the longActive falling edge (routes 107/109: 0x1a->0x0a OEM chime on every MADS brake).
    # Not on gas: the gas override path (0x1a<->0x2a) is silent and must keep following the camera.
    if CC.longActive:
      self.cancel_demote_frames = 0
    elif (self.enabled_prev and not CC.enabled) or (self.long_active_prev and not gas_pressed):
      self.cancel_demote_frames = 200  # ~2 s @100 Hz
    elif camera_deactivated and (CC.enabled or self.cancel_demote_frames > 0):
      # The camera holds CRUISE_STATE_2=1 (0x0a, "deactivated") for as long as the brake stays down, not
      # ~0.1 s, and right after a session the cluster sounds its loud take-over alarm (1 Hz double beep)
      # when it is shown 0x0a (off-state toggles with no recent session are silent) -- route 1b5 (cabin
      # mic): silent while 0x12 was masked, alarm 0.2 s after the 2 s grace ran out with the pedal still
      # down, alarm stopped at brake release. So hold the mask until the camera leaves 0x0a itself, and
      # also while lateral stays engaged (a brake in lat-only would leak it).
      self.cancel_demote_frames = max(self.cancel_demote_frames - 1, 1)
    else:
      self.cancel_demote_frames = max(0, self.cancel_demote_frames - 1)
    # LKAS standby is tied to the full disengage: MADS keeps lateral (and its HUD) through a brake.
    # Route 1b5: dropping LKAS_STATE 5 -> 0 on stalk UP set off the same alarm ~0.4 s later (the
    # steering request alone drops silently on every driver override); 5 -> 1 was silent on route 1b9.
    # But the cluster draws 1 as a red lane-keeping icon, so only bridge with it for ~2 s, then fall back
    # to the camera's idle 0.
    if CC.enabled:
      self.lkas_standby_frames = 0
    elif self.enabled_prev:
      self.lkas_standby_frames = 200  # ~2 s @100 Hz
    else:
      self.lkas_standby_frames = max(0, self.lkas_standby_frames - 1)
    # OEM "ACC exited" chime: on every ACC exit the camera sets 0x2AB b17 0x04 until ~0.5-2 s after the
    # brake is released, and the cluster double-beeps on it ~0.3 s after release unless the blinker is on.
    # Routes 1b5/1b9/1bb/1bc: 0x04 on all 12 ACC-exit brakes; chime on all but the blinker-held one and
    # one where 0x04 cleared during a standstill before release. Lateral-only brakes never set it and
    # are silent. The exit is the driver's own gesture and openpilot's UI already shows it, so hide the
    # flag from the cluster until the camera clears it (30 s cap). Forced disengages still alert via openpilot.
    if CC.longActive:
      self.exit_chime_frames = 0
    elif (self.enabled_prev and not CC.enabled) or (self.long_active_prev and not gas_pressed):
      self.exit_chime_frames = 3000  # 30 s cap @100 Hz
      self.exit_flag_seen = camera_exit_flag
    elif self.exit_chime_frames > 0:
      if camera_exit_flag:
        self.exit_flag_seen = True
      self.exit_chime_frames = 0 if (self.exit_flag_seen and not camera_exit_flag) else self.exit_chime_frames - 1
    # Gas override: longActive drops only while the pedal is down, ACC stays engaged. Route 1c8: passing
    # the camera through here showed 0x12/0x2a, so the ICC icon and Vmax went out while ACC was on.
    if not CC.enabled or not gas_pressed:
      self.acc_gas_override = False
    elif CC.longActive or self.long_active_prev:
      self.acc_gas_override = True
    self.enabled_prev = CC.enabled
    self.long_active_prev = CC.longActive

  def acc_cluster_args(self, CC, v_ego: float) -> dict:
    """Arguments for create_acc_cluster_mk4: ICC on (set speed + dashes) while ACC is engaged, gas
    override included; otherwise hide the ICC icon instead of the camera's stale 'available' set speed."""
    hud = CC.hudControl
    set_kph = None
    follow = None
    cruise_active = bool(CC.longActive or self.acc_gas_override)
    if cruise_active:
      raw = float(hud.setSpeed) * CV.MS_TO_KPH if hud.speedVisible else 0.0
      if 0.0 < raw < 200.0:
        self.acc_cluster_set_kph = raw
      # Hold last good set while engaged even if one frame publishes 255/unset
      set_kph = self.acc_cluster_set_kph
      if set_kph is None and v_ego > 0.5:
        # First-engage race before VCruiseHelper init: show vEgo floor 20 like cruise.py
        set_kph = float(np.clip(v_ego * CV.MS_TO_KPH, 20.0, 145.0))
        self.acc_cluster_set_kph = set_kph
      # Follow dashes: map personality 1..3 -> OEM 1..4; only update latch on change.
      # Default to 3 if HUD has not published bars yet — OEM dist 0 = "disabled" (no ICC icon).
      if hud.leadDistanceBars > 0:
        new_follow = 4 if hud.leadDistanceBars >= 3 else int(hud.leadDistanceBars)
        if self.acc_cluster_follow is None or new_follow != self.acc_cluster_follow:
          self.acc_cluster_follow = new_follow
      if self.acc_cluster_follow is None:
        self.acc_cluster_follow = 3
      follow = self.acc_cluster_follow
    else:
      self.acc_cluster_set_kph = None
      self.acc_cluster_follow = None
    return dict(
      set_speed_kph=set_kph,
      follow_dashes=follow,
      cruise_active=cruise_active,
      cancel_demote=self.cancel_demote_frames > 0,
      hide_exit_chime=self.exit_chime_frames > 0,
      # Also during the quiet-cancel grace, so a cancel is one cluster change (0x1a -> 0x12 without dashes).
      hide_icc=not cruise_active,
    )

  def hud_steer_required(self, CC, CS) -> bool:
    # MK4: latActive drops at standstill (steerAtStandstill=False) while lateral stays engaged, which
    # stepped LKAS 5->0->5 on the cluster at every stop (routes 107/109). Keep LKAS shown through a
    # stop while engaged; a real disengage still clears it via the LKAS demote.
    if self.is_mk4:
      return bool(CC.latActive or (CC.enabled and CS.out.vEgo < MK4_HUD_STANDSTILL_SPEED))
    return bool(CC.latActive)

  def update(self, CC, CS, now_nanos):
    can_sends = []
    actuators = CC.actuators
    if self.is_mk4:
      tq = abs(CS.out.steeringTorque)
      if tq > OVERRIDE_INSTANT_TORQUE:
        self.override_counter = OVERRIDE_FRAMES
      elif tq > OVERRIDE_TORQUE:
        self.override_counter = min(OVERRIDE_FRAMES, self.override_counter + 1)
      else:
        self.override_counter = max(0, self.override_counter - 1)
      if self.override_active:
        # OEM-style hold: keep control handed back for OVERRIDE_HOLD_FRAMES, then release only once the
        # driver torque has actually decayed.
        if self.override_hold > 0:
          self.override_hold -= 1
        if self.override_counter <= 0 and self.override_hold <= 0:
          self.override_active = False
      elif self.override_counter >= OVERRIDE_FRAMES:
        self.override_active = True
        self.override_hold = OVERRIDE_HOLD_FRAMES
      lat_active = CC.latActive and not self.override_active

      camera_deactivated = CS.acc_stock_raw is not None and (CS.acc_stock_raw[18] & 0x38) == 0x08
      camera_exit_flag = CS.acc_stock_raw is not None and bool(CS.acc_stock_raw[17] & 0x04)
      self.update_quiet_cancel(CC, CS.out.gasPressed, camera_deactivated, camera_exit_flag)
    else:
      lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # Increment counter so cancel is prioritized even without openpilot longitudinal
    if CC.cruiseControl.cancel:
      counter = gwmcan.next_counter(CS.steer_and_ap_stalk_msg['COUNTER'])
      can_sends.append(gwmcan.create_buttons_command(
        self.packer,
        self.CAN,
        counter,
        CS.steer_and_ap_stalk_msg,
        cancel_command=True,
      ))

    if self.frame % 2 == 0: # 50 Hz
      if self.CP.steerControlType == SteerControlType.angle:
        # MK4: 14-bit angle command (STEER_REQUEST) in STEER_CMD. Command the controller's wheel angle straight
        # through, then limit lateral jerk + accel via the vehicle model (Tesla-style). The helper holds the
        # current wheel angle when not lat_active, giving a bumpless hand-back on driver override.
        target_angle = actuators.steeringAngleDeg
        apply_angle = apply_steer_angle_limits_vm(target_angle, self.apply_angle_last,
                                                  CS.out.vEgoRaw, CS.out.steeringAngleDeg, lat_active,
                                                  CarControllerParams, self.VM)
        # MK4 low-speed hunt damp: VM jerk already softens with v^2, but routes 70/72/73 still had
        # cmd reversals 2–3× the wheel below ~40 kph. Extra rate schedule on top of MAX_ANGLE_RATE.
        if self.is_mk4 and lat_active:
          v_kph = float(CS.out.vEgoRaw) * CV.MS_TO_KPH
          max_rate = float(np.interp(
            v_kph,
            [CarControllerParams.MK4_ANGLE_RATE_V_LO, CarControllerParams.MK4_ANGLE_RATE_V_HI],
            [CarControllerParams.MK4_ANGLE_RATE_LOW, CarControllerParams.MK4_ANGLE_RATE_HIGH],
          ))
          apply_angle = float(np.clip(apply_angle,
                                      self.apply_angle_last - max_rate,
                                      self.apply_angle_last + max_rate))
        # MK4: stop the command from winding far past the measured wheel during EPS under-execution
        # (see MK4_ANGLE_ERROR_MAX). When the EPS is not granting angle authority (A_RX != 1), use a
        # tighter band so we don't keep pushing opposite-signed commands into a non-tracking EPS.
        # MK3 (torque path) is unaffected.
        if self.CP.carFingerprint == CAR.GWM_HAVAL_H6_MK4 and lat_active:
          eps_obeying = int(CS.eps_stock_values.get("A_RX_STEER_REQUESTED", 1)) == 1
          err_max = (CarControllerParams.MK4_ANGLE_ERROR_MAX if eps_obeying
                     else CarControllerParams.MK4_ANGLE_ERROR_MAX_NOT_OBEYING)
          apply_angle = float(np.clip(apply_angle,
                                      CS.out.steeringAngleDeg - err_max,
                                      CS.out.steeringAngleDeg + err_max))
        can_sends.append(gwmcan.create_steer_command_angle(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          apply_angle=apply_angle,
          lat_active=lat_active,
        ))
        self.apply_angle_last = apply_angle

        # MK4 hands-on keepalive: re-transmit the EPS 0x147 frame to the camera. The panda never forwards the
        # stock 0x147 main->camera while this safety mode is active (TX config has .check_relay=true), so we are
        # the camera's ONLY source of EPS feedback whenever onroad -- engaged OR not. Send every frame like the
        # MK3 create_wheel_touch does; while engaged patch in a dynamic hands-on torque (floor/cap/angle-gain
        # from highway limp analysis) so the ADAS never runs its hands-off warning/safe-stop escalation, and
        # let a real driver grab pass through. Disengaged, the real torque is re-encoded unchanged.
        if CS.eps_stock_raw is not None:
          spoof_torque = int(CS.out.steeringTorque)
          if CC.enabled:
            mag = min(MK4_HANDS_ON_TORQUE_MAX, MK4_HANDS_ON_TORQUE + MK4_HANDS_ON_ANGLE_GAIN * abs(apply_angle))
            if abs(spoof_torque) <= mag:
              spoof_torque = int(mag if apply_angle >= 0 else -mag)
          can_sends.append(gwmcan.create_wheel_touch_mk4(self.CAN, CS.eps_stock_raw, spoof_torque))
      else:
        new_torque = int(round(actuators.torque * self.params.STEER_MAX))
        apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
        # Prevent sending the same 'apply_torque = 1' torque repeatedly, as it can cause EPS faults.
        if abs(apply_torque) == 1:
          apply_torque = apply_torque * 2
        if not lat_active:
          apply_torque = 0
        can_sends.append(gwmcan.create_steer_command(
          self.packer,
          self.CAN,
          camera_stock_values=CS.camera_stock_values,
          steer=apply_torque,
          steer_req=lat_active,
        ))
        self.apply_torque_last = apply_torque

        ea_simulated_torque = float(np.clip(apply_torque * 2, -self.params.STEER_MAX, self.params.STEER_MAX))
        if abs(CS.out.steeringTorque) > abs(ea_simulated_torque):
          ea_simulated_torque = CS.out.steeringTorque
        can_sends.append(gwmcan.create_wheel_touch(
          self.packer,
          self.CAN,
          eps_stock_values=CS.eps_stock_values,
          ea_simulated_torque=ea_simulated_torque,
        ))

      # Longitudinal control
      if self.CP.openpilotLongitudinalControl:
        standstill = actuators.longControlState == LongCtrlState.stopping
        self.accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        if self.accel < 0:
          accel = - abs(self.accel / CarControllerParams.ACCEL_MIN)
        else:
          accel = self.accel / CarControllerParams.ACCEL_MAX
        # braking (BRAKE_OR_GAS_REQ / cmds): plain sign of accel every frame.
        # regen_brake (BRAKE_GAS_STATE only): light hysteresis — see MK4_REGEN_* above.
        braking = self.accel < 0
        if self.is_mk4:
          if not CC.longActive:
            self.regen_brake = False
          elif self.accel <= MK4_REGEN_ON_MS2:
            self.regen_brake = True
          elif self.accel >= MK4_REGEN_OFF_MS2:
            self.regen_brake = False
        can_sends.append(gwmcan.create_longitudinal_command(
          self.packer,
          self.CAN,
          longitudinal_stock_values=CS.longitudinal_stock_values,
          accel=accel,
          active=CC.longActive,
          standstill=standstill,
          is_mk4=self.is_mk4,
          regen=self.regen_brake,
          braking=braking,
          longitudinal_stock_raw=CS.acc_cmd_stock_raw if self.is_mk4 else None,
        ))

    if self.frame % 5 == 0:  # 20 Hz
      # HUD updates
      can_sends.append(gwmcan.create_hud_command(
        self.packer,
        self.CAN,
        hud_stock_values=CS.hud_stock_values,
        steer_required=self.hud_steer_required(CC, CS),
        is_mk4=self.is_mk4,
        hud_stock_raw=CS.hud_stock_raw if self.is_mk4 else None,
        lkas_standby=self.is_mk4 and self.lkas_standby_frames > 0,
      ))

    # MK4 OP_CRUISE: re-TX camera ACC (0x2AB) onto main with openpilot set speed so the Haval
    # cluster tracks VCruiseHelper. Match OEM camera rate (~10 Hz; panda RX check 10U).
    # Always re-TX while we hold the relay slot. While engaged: keep set-speed + CRUISE activated
    # latched (OEM keeps the Vmax icon solid whenever ACC is on — intermittent chrome was from
    # frozen camera state + thrashing distance on every personality tick).
    if self.frame % 10 == 0:  # 10 Hz
      if self.is_mk4 and self.CP.openpilotLongitudinalControl and CS.acc_stock_raw is not None:
        can_sends.append(gwmcan.create_acc_cluster_mk4(self.CAN, CS.acc_stock_raw, **self.acc_cluster_args(CC, CS.out.vEgo)))

    new_actuators = actuators.as_builder()
    if self.CP.steerControlType == SteerControlType.angle:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    self.frame += 1
    return new_actuators, can_sends
