from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.fca_giorgio import fca_giorgiocan
from opendbc.car.fca_giorgio.values import CanBus, CarControllerParams

TORQUE_SCALE = 0.5


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CCP = CarControllerParams(CP)
    self.CANBUS = CanBus(CP)
    self.packer_pt = CANPacker(dbc_names[Bus.pt])
    self.frame = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    # Passthrough camera's LKA messages with scaled torque.
    # HUD messages pass through unmodified (not in safety TX list).
    if self.frame % self.CCP.STEER_STEP == 0:
      lka_torque = int(round(CS.cam_lka_torque * TORQUE_SCALE))
      lka2_torque = int(round(CS.cam_lka2_torque * TORQUE_SCALE))

      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, self.CANBUS.pt, "LKA_COMMAND",
                                                               lka_torque, CS.cam_lka_active, counter=CS.cam_lka_counter))
      can_sends.append(fca_giorgiocan.create_steering_control(self.packer_pt, self.CANBUS.pt, "LKA_COMMAND_2",
                                                               lka2_torque, CS.cam_lka2_active, counter=CS.cam_lka2_counter))

    new_actuators = actuators.as_builder()
    new_actuators.torque = 0
    new_actuators.torqueOutputCan = 0

    self.frame += 1
    return new_actuators, can_sends
