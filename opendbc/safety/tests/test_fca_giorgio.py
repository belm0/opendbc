#!/usr/bin/env python3
import unittest
from opendbc.car.structs import CarParams
from opendbc.car.fca_giorgio.fca_giorgiocan import lka_command_torque
import opendbc.safety.tests.common as common

class TestFcaGiorgio_Safety(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  DBC = "fca_giorgio"
  SAFETY_MODEL = CarParams.SafetyModel.fcaGiorgio

  RELAY_MALFUNCTION_ADDRS = {0: (0x117, 0x1F6, 0x547, 0x5A2)}

  # LKA_COMMAND_2 units (4x LKA_COMMAND)
  MAX_RATE_UP = 12
  MAX_RATE_DOWN = 12
  MAX_TORQUE_LOOKUP = [0], [1200]
  MAX_RT_DELTA = 448

  DRIVER_TORQUE_ALLOWANCE = 80
  DRIVER_TORQUE_FACTOR = 8

  TX_MSGS = [[0x117, 0], [0x1F6, 0], [0x547, 0], [0x5A2, 0], [0x2FA, 2]]
  STANDSTILL_THRESHOLD = 0
  FWD_BLACKLISTED_ADDRS = {2: [0x117, 0x1F6, 0x547, 0x5A2]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}

  def _button_msg(self, cancel=False, resume=False):
    values = {"CANCEL": cancel, "RESUME": resume, "UNKNOWN_1": 3, "UNKNOWN_2": 0x0E}
    return self.packer.make_can_msg_safety("ACC_BUTTON", 2, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_STATUS": enable}
    return self.packer.make_can_msg_safety("ACC_2", 2, values)

  def _speed_msg(self, speed):
    values = {"WHEEL_SPEED_%s" % s: speed for s in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("ABS_1", 0, values)

  def _user_gas_msg(self, gas):
   values = {"ACCEL_PEDAL_USER": gas}
   return self.packer.make_can_msg_safety("ENGINE_2", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PEDAL_SWITCH": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ABS_3", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"DRIVER_TORQUE": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND_2", 0, values)

  def _torque_cmd_1_msg(self, torque, steer_req=1):
    values = {"LKA_TORQUE": torque, "LKA_ACTIVE": steer_req}
    return self.packer.make_can_msg_safety("LKA_COMMAND", 0, values)

  def test_lka_command_mirrors_lka_command_2(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    for torque in (0, 5, 6, 12, 22, 10, 0, -2, -6, -12, -22):
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      # LKA_COMMAND_2 / 4, rounded either way
      for torque_1 in range(torque // 4 - 2, torque // 4 + 3):
        self.assertEqual(abs(torque_1 * 4 - torque) <= 2, self._tx(self._torque_cmd_1_msg(torque_1)), f"{torque=} {torque_1=}")
      # what carcontroller sends is allowed; mismatched LKA_ACTIVE is blocked
      self.assertTrue(self._tx(self._torque_cmd_1_msg(lka_command_torque(torque))))
      self.assertFalse(self._tx(self._torque_cmd_1_msg(lka_command_torque(torque), steer_req=0)))

  def test_lka_command_zero_after_blocked_lka_command_2(self):
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(0)
    self._rx(self._torque_driver_msg(0))
    self.assertTrue(self._tx(self._torque_cmd_msg(12)))
    # rate limit violation blocks LKA_COMMAND_2; LKA_COMMAND can't carry that torque or the last allowed one
    self.assertFalse(self._tx(self._torque_cmd_msg(12 + self.MAX_RATE_UP + 4)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg((12 + self.MAX_RATE_UP + 4) // 4)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg(12 // 4)))
    self.assertTrue(self._tx(self._torque_cmd_1_msg(0, steer_req=0)))

  def test_lka_command_without_controls(self):
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_1_msg(25)))
    self.assertTrue(self._tx(self._torque_cmd_msg(0, steer_req=0)))
    self.assertTrue(self._tx(self._torque_cmd_1_msg(0, steer_req=0)))
    self.assertFalse(self._tx(self._torque_cmd_1_msg(1, steer_req=0)))

  WINDDOWN_MAX_US = 1500000
  WINDDOWN_MAX_GAP_US = 50000

  def _engage_with_torque(self, torque):
    self.safety.set_controls_allowed(True)
    self._rx(self._torque_driver_msg(0))
    self._set_prev_torque(torque)
    self.assertTrue(self._tx(self._torque_cmd_msg(torque)))

  def test_winddown_after_disengage(self):
    for sign in (1, -1):
      self.safety.init_tests()
      self._engage_with_torque(sign * 120)
      self.safety.set_controls_allowed(False)
      # magnitude may hold or shrink, never grow
      self.assertTrue(self._tx(self._torque_cmd_msg(sign * 120)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * 121)))
      for torque in range(108, 0, -12):
        self.assertTrue(self._tx(self._torque_cmd_msg(sign * torque)))
      # winddown ends at zero
      self.assertTrue(self._tx(self._torque_cmd_msg(0)))
      self.assertFalse(self._tx(self._torque_cmd_msg(sign * 12)))

  def test_winddown_limits(self):
    # no sign change, and LKA_ACTIVE must stay set
    self._engage_with_torque(120)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(-12)))
    self.assertFalse(self._tx(self._torque_cmd_msg(108, steer_req=0)))
    # blocked frames don't end the winddown (openpilot may send a frame or two before it sees the brake)
    self.assertTrue(self._tx(self._torque_cmd_msg(108)))

  def test_winddown_time_limit(self):
    for elapsed in (self.WINDDOWN_MAX_US, self.WINDDOWN_MAX_US + 1):
      self.safety.init_tests()
      self._engage_with_torque(1200)
      self.safety.set_controls_allowed(False)
      # keep frames coming within the gap limit
      t = 0
      while t + self.WINDDOWN_MAX_GAP_US < elapsed:
        t += self.WINDDOWN_MAX_GAP_US
        self.safety.set_timer(t)
        self.assertTrue(self._tx(self._torque_cmd_msg(1200)))
      self.safety.set_timer(elapsed)
      self.assertEqual(elapsed <= self.WINDDOWN_MAX_US, self._tx(self._torque_cmd_msg(1200)))

  def test_winddown_gap_limit(self):
    for gap in (self.WINDDOWN_MAX_GAP_US, self.WINDDOWN_MAX_GAP_US + 1):
      self.safety.init_tests()
      self._engage_with_torque(120)
      self.safety.set_controls_allowed(False)
      self.safety.set_timer(gap // 2)
      # a blocked frame doesn't restart the gap timer
      self.assertFalse(self._tx(self._torque_cmd_msg(132)))
      self.safety.set_timer(gap)
      self.assertEqual(gap <= self.WINDDOWN_MAX_GAP_US, self._tx(self._torque_cmd_msg(108)))

  def test_no_winddown_after_violation(self):
    self._engage_with_torque(120)
    # rate limit violation while engaged
    self.assertFalse(self._tx(self._torque_cmd_msg(120 + self.MAX_RATE_UP + 1)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(108)))

  def test_no_winddown_without_prior_torque(self):
    self._engage_with_torque(0)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_cmd_msg(12)))

  def test_winddown_on_brake(self):
    self._rx(self._speed_msg(100))
    self._rx(self._user_brake_msg(False))
    self._engage_with_torque(120)
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._tx(self._torque_cmd_msg(132)))
    for torque in range(120, -1, -12):
      self.assertTrue(self._tx(self._torque_cmd_msg(torque)))
      self.assertTrue(self._tx(self._torque_cmd_1_msg(torque // 4)))
      self.assertFalse(self._tx(self._torque_cmd_1_msg(torque // 4 + 1)))

  def test_reengage_during_winddown(self):
    self._engage_with_torque(120)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._torque_cmd_msg(108)))
    self.safety.set_controls_allowed(True)
    # rate limits continue from the last winddown frame
    self.assertTrue(self._tx(self._torque_cmd_msg(108 + self.MAX_RATE_UP)))
    self.assertFalse(self._tx(self._torque_cmd_msg(120 + self.MAX_RATE_UP + 1)))

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # only cancel is allowed, regardless of controls allowed
      self.assertTrue(self._tx(self._button_msg(cancel=True)))
      self.assertFalse(self._tx(self._button_msg(resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=True, resume=True)))
      self.assertFalse(self._tx(self._button_msg(cancel=False, resume=False)))

  def test_rx_hook(self):
    for count in range(20):
      self.assertTrue(self._rx(self._speed_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_brake_msg(False)), f"{count=}")
      self.assertTrue(self._rx(self._torque_driver_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._user_gas_msg(0)), f"{count=}")
      self.assertTrue(self._rx(self._pcm_status_msg(False)), f"{count=}")

  def test_rx_checksum_counter(self):
    for name, bus, length in (("ABS_1", 0, 8), ("ABS_3", 0, 8), ("ENGINE_2", 0, 8), ("EPS_2", 0, 7), ("ACC_2", 2, 8)):
      with self.subTest(msg=name):
        # an expected counter sequence is always accepted, and clears the wrong counter count
        for _ in range(common.MAX_WRONG_COUNTERS + 1):
          self.assertTrue(self._rx(self.packer.make_can_msg_safety(name, bus, {})))

        # a bad checksum (the last byte) fails, and disables controls
        self.safety.set_controls_allowed(True)
        msg = self.packer.make_can_msg_safety(name, bus, {})
        msg[0].data[length - 1] ^= 0xFF
        self.assertFalse(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())

        # a stuck counter fails as well, its checksum is still correct
        msg[0].data[length - 1] ^= 0xFF
        for i in range(common.MAX_WRONG_COUNTERS):
          should_rx = i < common.MAX_WRONG_COUNTERS - 1
          self.assertEqual(should_rx, self._rx(msg))


if __name__ == "__main__":
  unittest.main()
