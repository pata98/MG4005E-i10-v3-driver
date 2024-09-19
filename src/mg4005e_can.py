# MG4005E-i10 v3 CAN Driver
# Author        : Jiho Ryoo
# Affiliation   : POSTECH, CoCEL
# Contact       : yoopata@postech.ac.kr
# Date          : 2024.09.10

from package.MCP2515 import MCP2515
import time

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

    COMMAND_BYTE            = 0 
    RESPOND_TEMP            = 1
    RESPOND_CURRENT_L       = 2
    RESPOND_CURRENT_H       = 3
    RESPOND_SPEED_L         = 4
    RESPOND_SPEED_H         = 5
    RESPOND_ENCODER_L       = 6
    RESPOND_ENCODER_H       = 7

    CW                      = 0x00
    CCW                     = 0x01

    def __init__(self):
        """
        Initialize the MotorDriver with a CAN interface using MCP2515.
        
        Args:
            channel (str): The CAN channel (default: 'can0').
        """
        self.can = MCP2515()
        self.can.Init()

    #### CAN Message ###############################################################
    ## Tx/Rx
    def _send_message(self, message_id, data):
        """Send a CAN message to the bus using MCP2515."""
        message = self.can.Message(arbitration_id=message_id, data=data, is_extended_id=False)
        try:
            self.can.Send(message)
        except can.CanError as e:
            raise RuntimeError("Failed to send message: {}".format(e))

    def _receive_message(self):
        """Receive a CAN message with a timeout."""
        try:
            message = self.bus.recv(1.0)
            if message is not None:
                return message
        except self.can.CanError as e:
            raise RuntimeError("Error occurred while receiving message: {}".format(e))
        return None
    
    ## Processing
    def _build_message(self, command, payload=None):
        """Build a CAN message with the command byte and optional payload."""
        if payload is None:
            payload = []
        return [command] + payload + [0x00] * (self.DATA_LENGTH - len(payload) - 1)

    def _control_value_bytes(self, value, byte_length=2, signed=True):
        """Convert a control value into its byte representation."""
        return [(value >> (8 * i)) & 0xFF for i in range(byte_length)]

    #### Response Parsing ##########################################################
    def _parse_response(self, message):
        """Parse response message to extract motor state values."""
        command = message.data[self.COMMAND_BYTE]
        temp = message.data[self.RESPOND_TEMP]
        current = (message.data[self.RESPOND_CURRENT_H] << 8) | message.data[self.RESPOND_CURRENT_L]
        encoder = (message.data[self.RESPOND_ENCODER_H] << 8) | message.data[self.RESPOND_ENCODER_L]
        control = (message.data[self.RESPOND_SPEED_H] << 8) | message.data[self.RESPOND_SPEED_L]
        return {
            'command': command,
            'temperature': temp,
            'torque_current': current,
            'angle': encoder,
            'speed': control
        }

    #### Generalized Motor Command Function ########################################
    def _send_motor_command(self, motor_id, command, payload=None):
        """General method to send a motor command and receive its response."""
        message_id = self.IDENTIFIER_BASE + motor_id
        data = self._build_message(command, payload)
        self._send_message(message_id, data)
        response = self._receive_message()
        return self._parse_response(response) if response else {}

    #### Motor Power Control #######################################################
    def motor_on(self, motor_id):
        return self._send_motor_command(motor_id, self.CMD_MOTOR_ON)

    def motor_off(self, motor_id):
        return self._send_motor_command(motor_id, self.CMD_MOTOR_OFF)

    def motor_stop(self, motor_id):
        return self._send_motor_command(motor_id, self.CMD_MOTOR_STOP)

    #### Motor Movement Control ####################################################
    def torque_control(self, motor_id, torque_value):
        torque_bytes = self._control_value_bytes(torque_value)
        return self._send_motor_command(motor_id, self.CMD_TORQUE_CONTROL, [0x00, 0x00, 0x00] + torque_bytes)

    def speed_control(self, motor_id, speed_value):
        speed_bytes = self._control_value_bytes(speed_value, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_SPEED_CONTROL, [0x00, 0x00, 0x00] + speed_bytes)

    #### Angle Control #############################################################
    def angle_control_1(self, motor_id, angle_value):
        angle_bytes = self._control_value_bytes(angle_value, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_ML_1, [0x00, 0x00, 0x00] + angle_bytes)

    def angle_control_2(self, motor_id, angle_value, max_speed):
        angle_bytes = self._control_value_bytes(angle_value, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_ML_2, [0x00] + max_speed_bytes + angle_bytes)

    #### Incremental Angle Control #################################################
    def increment_angle_control_1(self, motor_id, angle_increment):
        angle_increment_bytes = self._control_value_bytes(angle_increment, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_INCREMENT_ANGLE_1, [0x00, 0x00, 0x00] + angle_increment_bytes)

    def increment_angle_control_2(self, motor_id, angle_increment, max_speed):
        angle_increment_bytes = self._control_value_bytes(angle_increment, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_INCREMENT_ANGLE_2, [0x00] + max_speed_bytes + angle_increment_bytes)

    #### Single Loop Angle Control #################################################
    def single_loop_angle_control_1(self, motor_id, spin_direction, angle_value):
        angle_bytes = self._control_value_bytes(angle_value * 100, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_SL_1, [spin_direction, 0x00, 0x00] + angle_bytes)

    def single_loop_angle_control_2(self, motor_id, spin_direction, angle_value, max_speed):
        angle_bytes = self._control_value_bytes(angle_value * 100, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_SL_2, [spin_direction] + max_speed_bytes + angle_bytes)


if __name__ == "__main__":
    print "CAN Communication tester"
    # Instantiate the driver
    driver = MotorDriver()

    # Example motor ID
    motor_id = 1

    # Test sequence with returned status
    print "Connection Test: Check CAN communication is well established."
    motor_status = driver.motor_on(motor_id)
    if motor_status['command'] == driver.CMD_MOTOR_ON:
        print 'PASS'  
    else:
        print "FAIL!" 
        exit(0)
    time.sleep(1)
