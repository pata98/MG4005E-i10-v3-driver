# MG4005E-i10 v3 CAN Driver
# Author        : Jiho Ryoo
# Affiliation   : POSTECH, CoCEL
# Contact       : yoopata@postech.ac.kr
# Date          : 2024.09.10

import can
import time
from typing import List, Optional, Dict

class MotorDriver:
    """Class to handle CAN communication for the MG4005E-i10 motor driver."""
    # Command Constants
    IDENTIFIER_BASE         = 0x140
    DATA_LENGTH             = 8

    CMD_MOTOR_OFF           = 0x80
    CMD_MOTOR_ON            = 0x88
    CMD_MOTOR_STOP          = 0x81
    CMD_TORQUE_CONTROL      = 0xA1
    CMD_SPEED_CONTROL       = 0xA2
    CMD_ANGLE_CONTROL_ML_1  = 0xA3
    CMD_ANGLE_CONTROL_ML_2  = 0xA4
    CMD_INCREMENT_ANGLE_1   = 0xA7
    CMD_INCREMENT_ANGLE_2   = 0xA8
    CMD_ANGLE_CONTROL_SL_1  = 0xA5
    CMD_ANGLE_CONTROL_SL_2  = 0xA6

    RESPOND_TEMP            = 1
    RESPOND_CURRENT_L       = 2
    RESPOND_CURRENT_H       = 3
    RESPOND_CTRL_L          = 4
    RESPOND_CTRL_H          = 5
    RESPOND_ENCODER_L       = 6
    RESPOND_ENCODER_H       = 7

    CW                      = 0x00
    CCW                     = 0x01

    def __init__(self, channel: str = 'can0'):
        """
        Initialize the MotorDriver with a CAN interface using MCP2515.
        
        Args:
            channel (str): The CAN channel (default: 'can0').
        """
        self.bus = can.interface.Bus(channel=channel, bustype='socketcan')

    #### CAN Message ###############################################################
    def _build_message(self, command: int, payload: Optional[List[int]] = None) -> List[int]:
        """Build a CAN message with the command byte and optional payload."""
        if payload is None:
            payload = []
        return [command] + payload + [0x00] * (self.DATA_LENGTH - len(payload) - 1)

    def _control_value_bytes(self, value: int, byte_length: int = 2, signed: bool = True) -> List[int]:
        """Convert a control value into its byte representation."""
        return list(value.to_bytes(byte_length, byteorder='little', signed=signed))

    def _send_message(self, message_id: int, data: List[int]) -> None:
        """Send a CAN message to the bus using MCP2515."""
        message = can.Message(arbitration_id=message_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
        except can.CanError as e:
            raise RuntimeError(f"Failed to send message: {e}")

    def _receive_message(self) -> Optional[can.Message]:
        """Receive a CAN message with a timeout."""
        try:
            message = self.bus.recv(1.0)
            if message is not None:
                return message
        except can.CanError as e:
            raise RuntimeError(f"Error occurred while receiving message: {e}")
        return None

    #### Response Parsing ##########################################################
    def _parse_response(self, message: can.Message) -> Dict[str, int]:
        """Parse response message to extract motor state values."""
        temp = message.data[self.RESPOND_TEMP]
        current = int.from_bytes([message.data[self.RESPOND_CURRENT_L], message.data[self.RESPOND_CURRENT_H]], byteorder='little', signed=True)
        encoder = int.from_bytes([message.data[self.RESPOND_ENCODER_L], message.data[self.RESPOND_ENCODER_H]], byteorder='little')
        control = int.from_bytes([message.data[self.RESPOND_CTRL_L], message.data[self.RESPOND_CTRL_H]], byteorder='little', signed=True)
        return {
            'temperature': temp,
            'torque_current': current,
            'angle': encoder,
            'control': control
        }

    #### Generalized Motor Command Function ########################################
    def _send_motor_command(self, motor_id: int, command: int, payload: Optional[List[int]] = None) -> Dict[str, int]:
        """General method to send a motor command and receive its response."""
        message_id = self.IDENTIFIER_BASE + motor_id
        data = self._build_message(command, payload)
        self._send_message(message_id, data)
        response = self._receive_message()
        return self._parse_response(response) if response else {}

    #### Motor Power Control #######################################################
    def motor_on(self, motor_id: int) -> Dict[str, int]:
        return self._send_motor_command(motor_id, self.CMD_MOTOR_ON)

    def motor_off(self, motor_id: int) -> Dict[str, int]:
        return self._send_motor_command(motor_id, self.CMD_MOTOR_OFF)

    def motor_stop(self, motor_id: int) -> Dict[str, int]:
        return self._send_motor_command(motor_id, self.CMD_MOTOR_STOP)

    #### Motor Movement Control ####################################################
    def torque_control(self, motor_id: int, torque_value: int) -> Dict[str, int]:
        torque_bytes = self._control_value_bytes(torque_value)
        return self._send_motor_command(motor_id, self.CMD_TORQUE_CONTROL, [0x00, 0x00, 0x00] + torque_bytes)

    def speed_control(self, motor_id: int, speed_value: int) -> Dict[str, int]:
        speed_bytes = self._control_value_bytes(speed_value, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_SPEED_CONTROL, [0x00, 0x00, 0x00] + speed_bytes)

    #### Angle Control #############################################################
    def angle_control_1(self, motor_id: int, angle_value: int) -> Dict[str, int]:
        angle_bytes = self._control_value_bytes(angle_value, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_ML_1, [0x00, 0x00, 0x00] + angle_bytes)

    def angle_control_2(self, motor_id: int, angle_value: int, max_speed: int) -> Dict[str, int]:
        angle_bytes = self._control_value_bytes(angle_value, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_ML_2, [0x00] + max_speed_bytes + angle_bytes)

    #### Incremental Angle Control #################################################
    def increment_angle_control_1(self, motor_id: int, angle_increment: int) -> Dict[str, int]:
        angle_increment_bytes = self._control_value_bytes(angle_increment, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_INCREMENT_ANGLE_1, [0x00, 0x00, 0x00] + angle_increment_bytes)

    def increment_angle_control_2(self, motor_id: int, angle_increment: int, max_speed: int) -> Dict[str, int]:
        angle_increment_bytes = self._control_value_bytes(angle_increment, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_INCREMENT_ANGLE_2, [0x00] + max_speed_bytes + angle_increment_bytes)

    #### Single Loop Angle Control #################################################
    def single_loop_angle_control_1(self, motor_id: int, spin_direction: int, angle_value: int) -> Dict[str, int]:
        angle_bytes = self._control_value_bytes(angle_value * 100, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_SL_1, [spin_direction, 0x00, 0x00] + angle_bytes)

    def single_loop_angle_control_2(self, motor_id: int, spin_direction: int, angle_value: int, max_speed: int) -> Dict[str, int]:
        angle_bytes = self._control_value_bytes(angle_value * 100, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_SL_2, [spin_direction] + max_speed_bytes + angle_bytes)


if __name__ == "__main__":
    # Instantiate the driver
    driver = MotorDriver(channel='can0')

    # Example motor ID
    motor_id = 1

    # Test sequence with returned status
    motor_status = driver.motor_on(motor_id)
    print("Motor On Status:", motor_status)
    time.sleep(1)

    motor_status = driver.single_loop_angle_control_2(motor_id, driver.CCW, 180, 360)  # Single loop angle control with speed limit (CCW, 180 degrees, 360 dps)
    print("Motor Status after Angle Control:", motor_status)
    time.sleep(3)

    motor_status = driver.single_loop_angle_control_2(motor_id, driver.CW, 0, 360)  # Single loop angle control with speed limit (CW, 0 degrees, 360 dps)
    print("Motor Status after Return to 0 Degrees:", motor_status)
    time.sleep(3)

    # Shutdown motor and get status
    motor_status = driver.motor_stop(motor_id)
    print("Motor Stop Status:", motor_status)
    motor_status = driver.motor_off(motor_id)
    print("Motor Off Status:", motor_status)
