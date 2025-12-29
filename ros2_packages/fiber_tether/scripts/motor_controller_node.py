#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String
from rclpy.duration import Duration

import math
import sys
import time

from dynamixel_motor import DynamixelMotor

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')
        
        self.declare_parameter('device_name', '/dev/ttyMotor')
        self.declare_parameter('baudrate', 1000000) 
        
        self.declare_parameter('motor_drum_id', 7) 
        self.declare_parameter('motor_screw_id', 2) 
        self.declare_parameter('teeth_drum', 40) 
        self.declare_parameter('teeth_drum_motor', 50) 
        self.declare_parameter('cable_diameter', 0.9) # mm
        self.declare_parameter('drum_diameter', 65) # mm
        self.declare_parameter('max_rpm_motor', 62) # per https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#specifications
        self.declare_parameter('unit_scale', 0.229) # 0.229 = 1 rpm, per https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#goal-velocity104
        self.declare_parameter('screw_thread_pitch', 16) # mm
    
        self.device_name = self.get_parameter('device_name').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.motor_drum_id = self.get_parameter('motor_drum_id').get_parameter_value().integer_value
        self.motor_screw_id = self.get_parameter('motor_screw_id').get_parameter_value().integer_value
        self.unit_scale =  self.get_parameter('unit_scale').get_parameter_value().double_value
       
        self.teeth_drum =  self.get_parameter('teeth_drum').get_parameter_value().integer_value
        self.teeth_drum_motor =  self.get_parameter('teeth_drum_motor').get_parameter_value().integer_value
        self.cable_diameter =  self.get_parameter('cable_diameter').get_parameter_value().double_value
        self.drum_diameter =  self.get_parameter('drum_diameter').get_parameter_value().integer_value
        self.max_rpm_motor =  self.get_parameter('max_rpm_motor').get_parameter_value().integer_value
        self.max_rps_motor = self.max_rpm_motor / 60.0
        
        # drum mechanical specs
        self.drum_circunference = math.pi * self.drum_diameter
        self.max_rpm_drum_output = self.max_rpm_motor * (self.teeth_drum_motor / self.teeth_drum) #rpm
        self.max_rps_drum_output = self.max_rpm_drum_output / 60.0
        self.max_speed_output_mm_s = int(self.max_rps_drum_output * self.drum_circunference) # mm/s

        # screw mechanical specs
        self.screw_thread_pitch =   self.get_parameter('screw_thread_pitch').get_parameter_value().integer_value

        # timeout constants (heartbeat, watchdog)
        self.timeout_seconds = 1  # 1 second, maximum time between two cmd_speed_mm_s commands
        self.last_msg_time = self.get_clock().now()
        self.timeout_duration = Duration(seconds=self.timeout_seconds) # usado para comparar com last_msg_time, no formato rospy.Duration()
        
        self.target_speed_mm_s = 0
        
        self.get_logger().info(f"Para initialization OK: drum_diameter:{self.drum_diameter}mm, screw_thread_pitch:{self.screw_thread_pitch}mm")

        # open motor connections
        # Note: Different motors on the same U2D2 serial cable use the same port/baud rate.
        try:
            self.motor_drum = DynamixelMotor(
                self.motor_drum_id, self.device_name, self.baudrate
            )
            self.motor_drum.open_connection()
            self.motor_drum.set_bus_watchdog(2000) # Ex: Watchdog of 1sec
            
            self.motor_screw = DynamixelMotor(
                self.motor_screw_id, self.device_name, self.baudrate
            )
            self.motor_screw.open_connection()
            self.motor_screw.set_bus_watchdog(2000)

        except IOError as e:
            self.get_logger().fatal(f"Failed to initialize Dynamixel motors: {e}")
            sys.exit(1)

        self.cmd_sub = self.create_subscription(
            Float32,
            '/tether_control/cmd_speed_mm_s',
            self.cmd_callback,
            10)
        self.status_pub = self.create_publisher(String, '/tether_control/status', 10)
        
        self.get_logger().info("Tether Dynamixel Motor Controller Node Initialized.")
        self.get_logger().warn(f"Safety Check is turned on!")
        self.get_logger().warn(f"Motors STOPS automatically if time between cmd_speed_mm_s exceeded the timeout of {self.timeout_seconds:.1f} seconds")
        
        self.motor_cmd_timer = self.create_timer(1.0, self.send_motor_cmd)  
        
    def send_motor_cmd(self):
        if self.target_speed_mm_s > 0 and self.get_clock().now() - self.last_msg_time >= Duration(seconds=2):
            self.target_speed_mm_s = 0
            self.get_logger().warn(f"Speed set to 0 due delay in cmd command")
            
        self.get_logger().info(f"self.last_msg_time:{self.last_msg_time}, self.target_speed_mm_s:{self.target_speed_mm_s} {self.get_clock().now() - self.last_msg_time}")
            
        drum_cmd, screw_cmd = self.calculate_motor_commands(self.target_speed_mm_s)
        
        self.motor_drum.set_speed(drum_cmd, is_debug=False)
        self.motor_screw.set_speed(screw_cmd, is_debug=False)

        self.status_pub.publish(String(data=f"Running at {self.target_speed_mm_s:.2f} mm/s. DXL CMD: {drum_cmd}"))
        pass

    def calculate_motor_commands(self, target_output_speed_mm_s):
        """Calcula os comandos de velocidade Dynamixel (CMD) a partir da velocidade do cabo (mm/s)."""
        
        # Limita a velocidade desejada para a máxima suportada
        if target_output_speed_mm_s > self.max_speed_output_mm_s:
            self.get_logger().warn(f"Velocidade desejada {target_output_speed_mm_s:.2f} mm/s acima do permitido, limitando ao máximo {self.max_speed_output_mm_s:.2f} mm/s.")
            target_output_speed_mm_s = self.max_speed_output_mm_s

        # Calcula o RPM do motor do drum
        motor_drum_rpm = ((target_output_speed_mm_s / self.drum_circunference) * 60) * (
            self.teeth_drum / self.teeth_drum_motor
            # self.teeth_drum_motor / self.teeth_drum
        )
        #motor_drum_rpm = min(motor_drum_rpm, self.max_rpm_motor)
        if motor_drum_rpm > self.max_rpm_motor:
            self.get_logger().warn(f"Drum RPM Calculado {motor_drum_rpm:.2f} rpm acima do permitido, limitanto ao máximo {self.max_rpm_motor:.2f} rpm.")
            motor_drum_rpm = self.max_rpm_motor
        
        # Calcula o comando Dynamixel. O sinal define a direção (enrolar/desenrolar)
        motor_drum_cmd = int(motor_drum_rpm / self.unit_scale)
        
        # Se o comando original era negativo, mantenha o sinal (assumindo que o sinal define a direção)
        # if target_output_speed_mm_s < 0:
        #     motor_drum_cmd *= -1

        # Lógica Screw (Motor 2)
        # Assumindo que o motor screw (guia do cabo) deve girar a uma velocidade proporcional
        # à velocidade que enrola/desenrola o cabo, de modo a mover o diâmetro do cabo (0.9mm) 
        # a cada volta do drum.
        motor_screw_rpm = ((target_output_speed_mm_s / self.drum_circunference) * self.cable_diameter) * (1 / self.screw_thread_pitch) * 60
        motor_screw_cmd = int(motor_screw_rpm / self.unit_scale) # O sinal se mantem conforme target_output_speed_mm_s
        
        # O screw deve girar numa velocidade que a cada comprimento do drum deve ser movido 0.9mm
        # if target_output_speed_mm_s == 0:
        #     motor_screw_cmd = 0
        #self.get_logger().info(f"calculate_motor_commands::Motor Drum CMD: {motor_drum_cmd} ; Motor Screw CMD: {motor_screw_cmd}")
        #self.get_logger().info(f"calculate_motor_commands::Motor Drum RPM: {motor_drum_rpm:.3f} ; Motor Screw RPM: {motor_screw_rpm:.3f}")
        #self.get_logger().info(f"calculate_motor_commands::Motor Drum RPS: {motor_drum_rpm/60.0:.3f} ; Motor Screw RPS: {motor_screw_rpm/60.0:.3f}")
        self.get_logger().info(f"calculate_motor_commands::Drum mm/s: {(motor_drum_rpm/60.0)*(self.teeth_drum_motor / self.teeth_drum):.3f} ; Screw mm/s: {(motor_screw_rpm/60.0)*self.screw_thread_pitch:.3f}")

        return motor_drum_cmd, motor_screw_cmd

    def cmd_callback(self, msg: Float32):
        """Callback chamada quando uma nova mensagem de velocidade (Float32) é recebida."""
        self.target_speed_mm_s = msg.data # Velocidade desejada em mm/s (positivo para desenrolar, negativo para enrolar)
        self.last_msg_time = self.get_clock().now()
        self.get_logger().info(f"Speed received: {self.target_speed_mm_s:.2f} mm/s")

    def shutdown(self):
        """Executado quando o nó é desligado (ex: Ctrl+C) para desabilitar torque e fechar a porta."""
        self.get_logger().warn("ROS Node Shutting Down. Disabling motor torque...")
        if hasattr(self, 'motor_drum') and self.motor_drum:
            self.motor_drum.close_connection()
        if hasattr(self, 'motor_screw') and self.motor_screw:
            self.motor_screw.close_connection()
        self.get_logger().info("Tether Dynamixel Node successfully shut down.")

    
def main(args=None):
    rclpy.init(args=args)

    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except (SystemExit, KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done')  # pyright: ignore[reportAttributeAccessIssue]

    motor_controller_node.shutdown()
    motor_controller_node.destroy_node()
    
    rclpy.shutdown()    

if __name__ == '__main__':
    main()