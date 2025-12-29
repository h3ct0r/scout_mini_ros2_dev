import rclpy
from rclpy.logging import get_logger

from typing import Dict, Tuple

from dynamixel_sdk import *

from enum import Enum

class OperatingMode(Enum):
    CURRENT_CONTROL_MODE = 0
    VELOCITY_CONTROL_MODE = 1
    POSITION_CONTROL_MODE = 3
    EXTENDED_POSITION_CONTROL_MODE = 4
    CURRENT_BASED_POSITION_CONTROL_MODE = 5
    PWM_CONTROL_MODE = 16
    TORQUE_CONTROL_MODE = 100
    MULTI_TURN_MODE = 101


# {data_name: (address, size_byte)}
# https://emanual.robotis.com/docs/en/dxl/x/{MODEL}/#control-table
CTABLE = {
    "Model_Number": (0, 2),
    "Model_Information": (2, 4),
    "Firmware_Version": (6, 1),
    "ID": (7, 1),
    "Baud_Rate": (8, 1),
    "Return_Delay_Time": (9, 1),
    "Drive_Mode": (10, 1),
    "Operating_Mode": (11, 1),
    "Secondary_ID": (12, 1),
    "Protocol_Type": (13, 1),
    "Homing_Offset": (20, 4),
    "Moving_Threshold": (24, 4),
    "Temperature_Limit": (31, 1),
    "Max_Voltage_Limit": (32, 2),
    "Min_Voltage_Limit": (34, 2),
    "PWM_Limit": (36, 2),
    "Current_Limit": (38, 2),
    "Acceleration_Limit": (40, 4),
    "Velocity_Limit": (44, 4),
    "Max_Position_Limit": (48, 4),
    "Min_Position_Limit": (52, 4),
    "Shutdown": (63, 1),
    "Torque_Enable": (64, 1),
    "LED": (65, 1),
    "Status_Return_Level": (68, 1),
    "Registered_Instruction": (69, 1),
    "Hardware_Error_Status": (70, 1),
    "Velocity_I_Gain": (76, 2),
    "Velocity_P_Gain": (78, 2),
    "Position_D_Gain": (80, 2),
    "Position_I_Gain": (82, 2),
    "Position_P_Gain": (84, 2),
    "Feedforward_2nd_Gain": (88, 2),
    "Feedforward_1st_Gain": (90, 2),
    "Bus_Watchdog": (98, 1),
    "Goal_PWM": (100, 2),
    "Goal_Current": (102, 2),
    "Goal_Velocity": (104, 4),
    "Profile_Acceleration": (108, 4),
    "Profile_Velocity": (112, 4),
    "Goal_Position": (116, 4),
    "Realtime_Tick": (120, 2),
    "Moving": (122, 1),
    "Moving_Status": (123, 1),
    "Present_PWM": (124, 2),
    "Present_Current": (126, 2),
    "Present_Velocity": (128, 4),
    "Present_Position": (132, 4),
    "Velocity_Trajectory": (136, 4),
    "Position_Trajectory": (140, 4),
    "Present_Input_Voltage": (144, 2),
    "Present_Temperature": (146, 1),
}


class DynamixelMotor:
   
    def __init__(self, motor_id, device_name, baudrate):
        if rclpy.ok() is False:
            rclpy.init(args=None)
            
        self.DXL_ID = motor_id
        self.DEVICENAME = device_name
        self.BAUDRATE = baudrate
        self.PROTOCOL_VERSION = 2.0 # Assume Protocol 2.0 https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold
        self.DXL_MINIMUM_POSITION_VALUE = 0
        self.DXL_MAXIMUM_POSITION_VALUE = 4095 # Refer to the Minimum/Maximum Position Limit of product eManual
        self.COMM_SUCCESS = 0
        self.packetHandler = None
        self.portHandler = None
    
        self.logger = get_logger(f'motor{motor_id}_logger')

    def open_connection(self):
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        self.portHandler = PortHandler(self.DEVICENAME)

        if self.portHandler.openPort():
            self.logger.info(f"Succeeded to open the port {self.DEVICENAME} for DXL ID {self.DXL_ID}")
        else:
            self.logger.error(f"Failed to open the port '{self.DEVICENAME}' for DXL ID {self.DXL_ID}")
            raise IOError(f"Failed to open port {self.DEVICENAME}")

        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.logger.error(f"Failed to change the baudrate for DXL ID {self.DXL_ID}")
            raise IOError(f"Failed to set baudrate {self.BAUDRATE}")

        # set LED ON
        DynamixelMotor.writeTxRx(
            self.packetHandler, self.portHandler, self.DXL_ID, CTABLE, "LED", 1
        )

        self.packetHandler.reboot(self.portHandler, self.DXL_ID)
        time.sleep(0.5)

        self.set_operating_mode(OperatingMode.VELOCITY_CONTROL_MODE)
        self.set_torque_enabled(1)

    def get_input_voltage(self):
        read_status, read_msg = DynamixelMotor.readTxRx(
            self.packetHandler,  # type: ignore
            self.portHandler,  # type: ignore
            self.DXL_ID,
            CTABLE,
            "Present_Input_Voltage",
        )
        if read_status:
            self.logger.info(f"Present_Input_Voltage: {float(read_msg)/10}")

    def set_speed(self, speed_cmd, is_debug=False):
        write_status, write_msg = DynamixelMotor.writeTxRx(
            self.packetHandler,  # type: ignore
            self.portHandler,  # type: ignore
            self.DXL_ID,
            CTABLE,
            "Goal_Velocity",
            speed_cmd,
        )

        if write_status and is_debug:
            self.logger.info(f"DXL ID {self.DXL_ID}: write: Goal_Velocity set to {speed_cmd}")

    def set_torque_enabled(self, is_enabled):
        if is_enabled not in (0, 1):
            is_enabled = 0

        write_status, write_msg = DynamixelMotor.writeTxRx(
            self.packetHandler,  # type: ignore
            self.portHandler,  # type: ignore
            self.DXL_ID,
            CTABLE,
            "Torque_Enable",
            is_enabled,
        )
        if write_status:
            self.logger.info(f"DXL ID {self.DXL_ID}: Torque_Enable set to {is_enabled}")

    def set_operating_mode(self, mode):
        write_status, write_msg = DynamixelMotor.writeTxRx(
            self.packetHandler,  # type: ignore
            self.portHandler,  # type: ignore
            self.DXL_ID,
            CTABLE,
            "Operating_Mode",
            mode.value,
        )
        if write_status:
            self.logger.info(f"DXL ID {self.DXL_ID}: write Operating_Mode OK set to {mode}")

    def set_bus_watchdog(self, ms=2000):
        ms_unit_scale = 20
        if ms < 0 or ms > 2540:
            ms = 0

        write_status, write_msg = DynamixelMotor.writeTxRx(
            self.packetHandler,  # type: ignore
            self.portHandler,  # type: ignore
            self.DXL_ID,
            CTABLE,
            "Bus_Watchdog",
            int(ms / ms_unit_scale),
        )
        if write_status:
            self.logger.info(f"DXL ID {self.DXL_ID}: write Bus_Watchdog OK")

    def close_connection(self):
        # Chama set_speed(0) e set_torque_enabled(0) antes de fechar a porta!
        self.set_speed(0, is_debug=False)
        self.set_torque_enabled(0)
        if self.portHandler:
            self.portHandler.closePort()
            self.logger.info(f"DXL ID {self.DXL_ID}: Port closed.")
    
    @staticmethod
    def writeTxRx(
        packetHandler: Protocol2PacketHandler,
        portHandler: PortHandler,
        dxl_id: int,
        CTABLE: Dict[str, Tuple[int, int]],
        msg_id: str,
        msg_content: int,
    ) -> Tuple[bool, str]:
        if msg_id not in CTABLE:
            raise ValueError(f"ERROR: {msg_id} not in CTABLE")

        msg_addr, msg_bytes = CTABLE[msg_id]
        if msg_bytes == 1:
            comm_result, error = packetHandler.write1ByteTxRx(
                portHandler, dxl_id, msg_addr, msg_content
            )
        elif msg_bytes == 2:
            comm_result, error = packetHandler.write2ByteTxRx(
                portHandler, dxl_id, msg_addr, msg_content
            )
        else:
            comm_result, error = packetHandler.write4ByteTxRx(
                portHandler, dxl_id, msg_addr, msg_content
            )

        if comm_result != COMM_SUCCESS:
            msg = packetHandler.getTxRxResult(comm_result)
            return False, msg
        elif error != 0:
            return False, ""

        return True, ""

    @staticmethod
    def readTxRx(
        packetHandler: Protocol2PacketHandler,
        portHandler: PortHandler,
        dxl_id: int,
        CTABLE: Dict[str, Tuple[int, int]],
        msg_id: str,
    ) -> Tuple[bool, str]:
        if msg_id not in CTABLE:
            raise ValueError(f"ERROR: {msg_id} not in CTABLE")

        msg_addr, msg_bytes = CTABLE[msg_id]
        if msg_bytes == 1:
            read_res, comm_result, error = packetHandler.read1ByteTxRx(
                portHandler, dxl_id, msg_addr
            )
        elif msg_bytes == 2:
            read_res, comm_result, error = packetHandler.read2ByteTxRx(
                portHandler, dxl_id, msg_addr
            )
        else:
            read_res, comm_result, error = packetHandler.read4ByteTxRx(
                portHandler, dxl_id, msg_addr
            )

        if comm_result != COMM_SUCCESS:
            msg = packetHandler.getTxRxResult(comm_result)
            return False, msg
        elif error != 0:
            return False, ""

        return True, str(read_res)
