from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fca_giorgio import fca_giorgiocan
from opendbc.car.fca_giorgio.values import CanBus, CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CANBUS = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])

    self.apply_torque_last = 0
    self.frame = 0
    self.signal_left_last = False
    self.signal_right_last = False
    self.lkas_enable_start_frame = 0
    self.lkas_disable_start_frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # ------------------------------------------------------------------
    # test spoofing center-console LKA enable button
    #   right turn signal: enable
    #   left turn signal: disable

    signal_left_edge = CS.out.leftBlinker and not self.signal_left_last
    signal_right_edge = CS.out.rightBlinker and not self.signal_right_last
    self.signal_left_last = CS.out.leftBlinker
    self.signal_right_last = CS.out.rightBlinker
    target_bus = self.CANBUS.cam

    if signal_right_edge and not self.lkas_enable_start_frame:
      print('** starting LKAS enable')
      # TODO: don't overwrite other signals in the message
      msg = self.packer_pt.make_can_msg("BCM_3", target_bus, {"LKA_BUTTON": 1})
      can_sends.append(msg)
      self.lkas_enable_start_frame = self.frame

    if self.lkas_enable_start_frame and self.frame - self.lkas_enable_start_frame > 20:
      msg = self.packer_pt.make_can_msg("BCM_3", target_bus, {"LKA_BUTTON": 0})
      can_sends.append(msg)
      self.lkas_enable_start_frame = 0
      print('** LKAS enable done')

    if signal_left_edge and not self.lkas_disable_start_frame:
      print('** starting LKAS disable')
      self.lkas_disable_start_frame = self.frame

    if self.lkas_disable_start_frame:
      d_frames = self.frame - self.lkas_disable_start_frame
      phase = d_frames // 20
      if phase in (0, 2):
        msg = self.packer_pt.make_can_msg("BCM_3", target_bus, {"LKA_BUTTON": 1})
        can_sends.append(msg)
      elif phase in (1, 3):
        msg = self.packer_pt.make_can_msg("BCM_3", target_bus, {"LKA_BUTTON": 0})
        can_sends.append(msg)
      else:
        self.lkas_disable_start_frame = 0
        print('** LKAS enable done')
    # ------------------------------------------------------------------


    # **** Steering Controls ************************************************ #

    if self.frame % self.CCP.STEER_STEP == 0:
      if CC.latActive:
        new_torque = int(round(actuators.torque * self.CCP.STEER_MAX))
        apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.CCP)
      else:
        apply_torque = 0

      self.apply_torque_last = apply_torque
      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, self.CANBUS.pt, apply_torque, CC.latActive))

    # **** HUD Controls ***************************************************** #

    # if self.frame % self.CCP.HUD_1_STEP == 0:
    #   can_sends.append(fca_giorgiocan.create_lka_hud_1_control(self.packer_pt, self.CANBUS.pt, CC.latActive))
    # if self.frame % self.CCP.HUD_2_STEP == 0:
    #   can_sends.append(fca_giorgiocan.create_lka_hud_2_control(self.packer_pt, self.CANBUS.pt, CC.latActive))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.CCP.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
