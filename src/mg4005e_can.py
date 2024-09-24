import can
import time

class MotorDriver:
    """Class to handle CAN communication for the MG4005E-i10 motor driver using python-can."""
    
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
    CMD_ZEROING             = 0x19
    CMD_READ_MOTOR_STATE    = 0x9C

    COMMAND_BYTE            = 0 
    RESPOND_TEMP            = 1
    RESPOND_CURRENT_L       = 2
    RESPOND_CURRENT_H       = 3
    RESPOND_SPEED_L         = 4
    RESPOND_SPEED_H         = 5
    RESPOND_ENCODER_L       = 6
    RESPOND_ENCODER_H       = 7
    RESPOND_SPEED_COEF      = 720/16383
    RESPOND_ENCODER_COEF    = 90/16383

    CW                      = 0x00
    CCW                     = 0x01

    ## Motor ID
    ID_MOTOR_PAN            = 0x01
    ID_MOTOR_TILT           = 0x02
    

    def __init__(self, bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000):
        """
        Initialize the MotorDriver with a CAN interface using python-can.
        """
        # Initialize python-can bus
        self.bus = can.interface.Bus(bustype=bustype, channel=channel, bitrate=bitrate)

    #### CAN Message ###############################################################
    def _build_message(self, command, payload=None):
        """Build a CAN message with the command byte and optional payload."""
        if payload is None:
            payload = []
        return [command] + payload + [0x00] * (self.DATA_LENGTH - len(payload) - 1)

    def _control_value_bytes(self, value, byte_length=2, signed=True):
        """Convert a control value into its byte representation."""
        return [(value >> (8 * i)) & 0xFF for i in range(byte_length)]

    def _send_motor_command(self, motor_id, command, payload=None):
        """Send a motor command using the python-can library."""
        message_id = self.IDENTIFIER_BASE + motor_id
        data = self._build_message(command, payload)

        msg = can.Message(arbitration_id=message_id, data=data, is_extended_id=False)

        try:
            self.bus.send(msg)
            # print(f"Message sent: ID={hex(message_id)}, Data={data}")
        except can.CanError:
            print("Message NOT sent")
        
        # Try to receive the response
        try:
            response = self.bus.recv(timeout=1.0)  # Wait for 1 second for a response
            if response is not None:
                return self._parse_response(response)
            else:
                print("No response received")
                return {}
        except Exception as e:
            raise RuntimeError(f"Error occurred while receiving message: {e}")

    def _parse_response(self, message):
        """Parse response message to extract motor state values."""
        command = message.data[self.COMMAND_BYTE]
        temp = message.data[self.RESPOND_TEMP]
        current = (message.data[self.RESPOND_CURRENT_H] << 8) | message.data[self.RESPOND_CURRENT_L]
        encoder = (message.data[self.RESPOND_ENCODER_H] << 8) | message.data[self.RESPOND_ENCODER_L]
        speed = (message.data[self.RESPOND_SPEED_H] << 8) | message.data[self.RESPOND_SPEED_L]

        # Sign
        speed = speed - 0x10000 if speed & 0x8000 else speed
        encoder = encoder * self.RESPOND_ENCODER_COEF if encoder * self.RESPOND_ENCODER_COEF <= 180 else (encoder * self.RESPOND_ENCODER_COEF - 360)

        return {
            'command': command,
            'temperature': temp,
            'torque_current': current,
            'angle': encoder,
            'speed': speed * self.RESPOND_SPEED_COEF
        }


    #### Motor Power Control / Setting #############################################
    def motor_on(self, motor_id):
        response = self._send_motor_command(motor_id, self.CMD_MOTOR_ON)
        motor_name='PAN' if motor_id == self.ID_MOTOR_PAN else 'TILT'
        if response.get('command') == self.CMD_MOTOR_ON:
            print(f'{motor_name} POWER ON') 
        else:
            print(f'{motor_name} ON FAILURE!')
        return response

    def motor_off(self, motor_id):
        response = self._send_motor_command(motor_id, self.CMD_MOTOR_OFF)
        motor_name='PAN' if motor_id == self.ID_MOTOR_PAN else 'TILT'
        if response.get('command') == self.CMD_MOTOR_OFF:
            print(f'{motor_name} POWER OFF') 
        else:
            print(f'{motor_name} OFF FAILURE!')
        return response

    def motor_stop(self, motor_id):
        response = self._send_motor_command(motor_id, self.CMD_MOTOR_STOP)
        motor_name='PAN' if motor_id == self.ID_MOTOR_PAN else 'TILT'
        if response.get('command') == self.CMD_MOTOR_STOP:
            print(f'{motor_name} STOP') 
        else:
            print(f'{motor_name} STOP FAILURE!')
        return response
    
    def motor_zeroing(self, motor_id):
        """Zeroing Motor. It writes to ROM, which will affect the chip life. DO NOT USE IT FREQUENTLY!!"""
        """  Also, note that motor should be powered off and on again to finish the zeroing procedure  """
        response = self._send_motor_command(motor_id, self.CMD_ZEROING)
        motor_name='PAN' if motor_id == self.ID_MOTOR_PAN else 'TILT'
        if response.get('command') == self.CMD_ZEROING:
            print(f'{motor_name} ZEROING SUCCESS') 
        else:
            print(f'{motor_name} ZEROING FAILURE')
        return response
    

    #### Motor Movement Control ####################################################
    def torque_control(self, motor_id, torque_value):
        """Still Need Dev."""
        torque_bytes = self._control_value_bytes(torque_value)
        return self._send_motor_command(motor_id, self.CMD_TORQUE_CONTROL, [0x00, 0x00, 0x00] + torque_bytes)

    def speed_control(self, motor_id, speed_value):
        speed_bytes = self._control_value_bytes(speed_value * 100, byte_length=4)   # Conver -> 1 input step = 0.01 dps/LSB
        return self._send_motor_command(motor_id, self.CMD_SPEED_CONTROL, [0x00, 0x00, 0x00] + speed_bytes)


    #### Monitor ###################################################################
    def read_motor_state(self, motor_id):
        return self._send_motor_command(motor_id, self.CMD_READ_MOTOR_STATE)
        

    ## Angle Control ###############################################################
    def angle_control_1(self, motor_id, angle_value):
        angle_bytes = self._control_value_bytes(angle_value, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_ML_1, [0x00, 0x00, 0x00] + angle_bytes)

    def angle_control_2(self, motor_id, angle_value, max_speed):
        angle_bytes = self._control_value_bytes(angle_value, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_ML_2, [0x00] + max_speed_bytes + angle_bytes)

    ## Incremental Angle Control ###################################################
    def increment_angle_control_1(self, motor_id, angle_increment):
        angle_increment_bytes = self._control_value_bytes(angle_increment, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_INCREMENT_ANGLE_1, [0x00, 0x00, 0x00] + angle_increment_bytes)

    def increment_angle_control_2(self, motor_id, angle_increment, max_speed):
        angle_increment_bytes = self._control_value_bytes(angle_increment, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_INCREMENT_ANGLE_2, [0x00] + max_speed_bytes + angle_increment_bytes)

    ## Single Loop Angle Control ###################################################
    def single_loop_angle_control_1(self, motor_id, spin_direction, angle_value):
        """Control angle"""
        angle_bytes = self._control_value_bytes(angle_value * 100, byte_length=4)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_SL_1, [spin_direction, 0x00, 0x00] + angle_bytes)

    def single_loop_angle_control_2(self, motor_id, spin_direction, angle_value, max_speed):
        """
        Control angle

        args: 
        """
        angle_value = angle_value if angle_value > 0 else (360 + angle_value)
        angle_bytes = self._control_value_bytes(angle_value * 1000, byte_length=4)
        max_speed_bytes = self._control_value_bytes(max_speed * 25, byte_length=2, signed=False)
        return self._send_motor_command(motor_id, self.CMD_ANGLE_CONTROL_SL_2, [spin_direction] + max_speed_bytes + angle_bytes)


if __name__ == "__main__":
    # Instantiate the driver
    driver = MotorDriver()

    # Test sequence with returned status
    motor_status = driver.motor_on(driver.ID_MOTOR_PAN)
    motor_status = driver.motor_on(driver.ID_MOTOR_TILT)

    print("=============== Move Motor ===============")
    print("Move PAN Motor to 10")
    target_angle = 20
    motor_status = driver.single_loop_angle_control_2(driver.ID_MOTOR_TILT, driver.CW, target_angle, 20)
    while abs(motor_status.get('angle') - target_angle) > 0.5:
        motor_status = driver.read_motor_state(driver.ID_MOTOR_TILT)
        print("angle: ", motor_status.get('angle'))
        print("speed: ", motor_status.get('speed'))
    
    # print("================ Monitor ================")
    # try:
    #     while True:
    #         motor_status = driver.read_motor_state(driver.ID_MOTOR_TILT)
    #         print("angle: ", motor_status.get('angle'))
    #         print("speed: ", motor_status.get('speed'))
    # except KeyboardInterrupt:
    #     print("Exit")


    motor_status = driver.motor_off(driver.ID_MOTOR_PAN)
    motor_status = driver.motor_off(driver.ID_MOTOR_TILT)
