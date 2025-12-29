#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import serial

from std_msgs.msg import String
from fiber_tether.msg import BatteryMonitorStatus  # pyright: ignore[reportAttributeAccessIssue]

# Your udev link, place in /etc/udev/rules.d
# Rule for the USB-C-Serial adapter for the XY-DJ Battery Control Board
# Board operating in U-5 mode with upper limit UL = 21.0V and lower limit nL = 17.5V, OP = 000
# SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", SYMLINK+="ttyBattery", MODE="0660", GROUP="dialout"  


class BattMonitor(Node):
    def __init__(self):
        super().__init__('batt_monitor_tether_node')
        self.publisher_ = self.create_publisher(BatteryMonitorStatus, '/battery/status', 10)
        
        SERIAL_PORT = '/dev/ttyBattery'  
        BAUD_RATE = 115200                  
        LOOP_RATE_SECS = 1.0
        READ_TIMEOUT = 0.5                
        
        # Open serial port and send 'start' cmd to start automatic upload of data [cite: 87]
        self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT)
        self.ser.write('start\r\n'.encode('utf-8'))
        
        self.get_logger().info("Sent 'start' command to XY-DJ module.")
        
        self.timer = self.create_timer(LOOP_RATE_SECS, self.monitor_callback)
        
    def monitor_callback(self):

        try:
            # Ensures we always send 'get' if 'start' doesn't work consistently
            # The manual indicates that the 'start' command initiates the upload, but 'get' is more immediate.
            # Decoding Chinese characters in gbk or gb2312,

            raw = self.ser.readline() #byte
            line = raw.decode('gbk').strip()
            
            if line:
                status_msg = self.parse_xy_dj_data(line)
                if status_msg is not None:
                    self.publisher_.publish(status_msg)
            else:
                 self.get_logger().warn("Timeout reading serial data. Waiting for data...")
                
        except serial.SerialException as e:
            self.get_logger().error("Serial communication error: %s" % e)
            
    def close_serial_connection(self):
        try:
            # Send 'stop' before closing the serial port [cite: 88]
            self.ser.write('stop\r\n'.encode('utf-8'))
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        except:
            pass

    def parse_xy_dj_data(self, msg_line):
        """Analyzes the received serial string (e.g., '电压:19.2v ,04:12:02 h,OP') and populates the BatteryMonitorStatus object."""
        
        # Ex: '电压:19.2v ,04:12:02 h,OP'
        parts = msg_line.split(',')
    
        if len(parts) != 3:
            self.get_logger().warn("Received unexpected data format (parts count != 3): %s" % line)
            return None

        msg = BatteryMonitorStatus()

        # 1. Tension (ex: '电压:19.2v ')
        try:
            voltage_str = parts[0].split(':')[-1]
            voltage_str = voltage_str.strip().strip().lower().replace('v', '')
            msg.voltage = round(float(voltage_str), 1)
        except Exception as e:
            self.get_logger().error("Failed to parse voltage from '%s'. Error: %s" % (parts[0], e))
            return None

        # 2. Relay status (ex: 'OP' -> 1, 'CL' -> 0)
        status_str = parts[2].strip()
        msg.is_power_on = 1 if status_str == 'OP' else 0

        # 3. ON Time (ex: '04:12:02 h')
        try:
            time_str = parts[1].strip().replace(' h', '')
            h, m, s = map(int, time_str.split(':'))
            msg.power_on_time_sec = h * 3600 + m * 60 + s
        except Exception as e:
            self.get_logger().error("Failed to parse on_time from '%s'. Error: %s" % (parts[1], e))
            return None

        return msg
    
def main(args=None):
    rclpy.init(args=args)

    batt_monitor = BattMonitor()

    try:
        rclpy.spin(batt_monitor)
    except (SystemExit, KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done') # pyright: ignore[reportAttributeAccessIssue]

    batt_monitor.close_serial_connection()
    batt_monitor.destroy_node()
    
    rclpy.shutdown()    

if __name__ == '__main__':
    main()