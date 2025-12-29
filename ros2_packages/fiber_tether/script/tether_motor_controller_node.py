import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String

import math
import sys
import time

from dynamixel_motor import DynamixelMotor

class MotorControllerNode:
    def __init__(self):

        rospy.init_node('motor_controller_node', anonymous=False) 

        self.motor_drum_id = rospy.get_param('~motor_drum_id', 7)
        self.motor_screw_id = rospy.get_param('~motor_screw_id', 2)
        self.device_name = rospy.get_param('~device_name', '/dev/ttyUSB0') # Use USB0 como fallback
        self.baudrate = rospy.get_param('~baudrate', 1000000)

        self.teeth_drum = rospy.get_param('~teeth_drum', 40)
        self.teeth_drum_motor = rospy.get_param('~teeth_drum_motor', 50)
        self.cable_diameter = rospy.get_param('~cable_diameter', 0.9)  # mm
        self.drum_diameter = rospy.get_param('~drum_diameter', 65)  # mm
        self.max_rpm_motor = rospy.get_param('~max_rpm_motor', 62) # rotações por minuto # per https://emanual.robotis.com/docs/en/dxl/mx/mx-64/#specifications
        self.max_rps_motor = self.max_rpm_motor / 60.0 # rotações por segundo
        self.unit_scale = rospy.get_param('~unit_scale', 0.229) # Escala de velocidade Dynamixel 0.229 = 1 rpm https://emanual.robotis.com/docs/en/dxl/mx/mx-64-2/#goal-velocity104

        # Constantes Mecânicas Drum
        self.drum_circunference = math.pi * self.drum_diameter
        self.max_rpm_drum_output = self.max_rpm_motor * (self.teeth_drum_motor / self.teeth_drum) #rpm
        self.max_rps_drum_output = self.max_rpm_drum_output / 60.0
        self.max_speed_output_mm_s = int(self.max_rps_drum_output * self.drum_circunference) # mm/s

        # Constantes Mecânicas Screw
        self.screw_thread_pitch = rospy.get_param('~screw_thread_pitch', 16)  # mm. Distancia percorrida pelo guia a cada volta do motor


        # Contantes do Timeout (heartbeat, watchdog)
        self.timeout_seconds = 1.0  # 1 segundo, tempo máximo entre dois comandos cmd_speed_mm_s
        self.last_msg_time = rospy.Time.now()
        self.timeout_duration = rospy.Duration(self.timeout_seconds) # usado para comparar com last_msg_time, no formato rospy.Duration()

        # 3. Inicializar e Abrir Conexão dos Motores
        try:
            self.motor_drum = DynamixelMotor(
                self.motor_drum_id, self.device_name, self.baudrate
            )
            self.motor_drum.open_connection()
            # self.motor_drum.set_bus_watchdog(1000) # Exemplo: 1 segundo de Watchdog
            
            self.motor_screw = DynamixelMotor(
                self.motor_screw_id, self.device_name, self.baudrate
            )
            # Nota: Motores diferentes no mesmo cabo serial U2D2 usam a mesma porta/baudrate.
            self.motor_screw.open_connection()
            # self.motor_screw.set_bus_watchdog(1000)

        except IOError as e:
            rospy.logfatal(f"Failed to initialize Dynamixel motors: {e}")
            # Se a conexão falhar, interrompa o nó ROS
            sys.exit(1)


        # 4. Subscriber para comandos (velocidade linear do cabo em mm/s)
        self.cmd_sub = rospy.Subscriber(
            '/tether_control/cmd_speed_mm_s',
            Float32,
            self.cmd_callback
        )
        rospy.loginfo("Subscribing to /tether_control/cmd_speed_mm_s")

        # 5. Publisher para status/telemetria (opcional, mas recomendado)
        self.status_pub = rospy.Publisher(
            '/tether_control/status',
            String, # Pode ser String, ou criar uma mensagem customizada
            queue_size=1
        )
        
        # 6. Configurar função de desligamento seguro
        rospy.on_shutdown(self.shutdown)

        # 7. Configurar função de timeout
        # self.safety_timer = rospy.Timer(rospy.Duration(self.check_repeat_seconds), self.safety_check)
        
        # 7. Status msg
        rospy.loginfo("Tether Dynamixel Motor Controller Node Initialized.")
        rospy.logwarn(f"Safety Check is turned on!")
        rospy.logwarn(f"Motors STOPS automatically if time between cmd_speed_mm_s exceeded the timeout of {self.timeout_seconds:.1f} seconds")
        
        # 8. Rodar o nó ROS
        self.rate = rospy.Rate(10.0) # Hz, frequencia de execução do loop principal
        while not rospy.is_shutdown():
            self.safety_check()
            self.rate.sleep() # Run at 10Hz

        # rospy.spin()

    def test_enrolar(self, test_speed_mm_s):
        # Teste por tempo determinado para verificar comprimento enrolado
        tempo = 5 # seconds
        # test_speed_mm_s = -10 # mm/s
        motor_drum_cmd, motor_screw_cmd = self.calculate_motor_commands(test_speed_mm_s)
        rospy.loginfo(f"Test Start::Enrolando por {tempo} segundos @ {test_speed_mm_s} mm/s")
        rospy.loginfo(f"Testing at {test_speed_mm_s:.2f} mm/s. Motor Drum CMD: {motor_drum_cmd} ; Motor Screw CMD: {motor_screw_cmd}")
        for _ in range(2*tempo):
            self.motor_drum.set_speed(motor_drum_cmd)
            self.motor_screw.set_speed(motor_screw_cmd)
            time.sleep(0.5)
        self.stop_motors()
        rospy.loginfo("Test Finish")

        


    def calculate_motor_commands(self, target_output_speed_mm_s):
        """Calcula os comandos de velocidade Dynamixel (CMD) a partir da velocidade do cabo (mm/s)."""
        
        # Limita a velocidade desejada para a máxima suportada
        if target_output_speed_mm_s > self.max_speed_output_mm_s:
            rospy.logwarn(f"Velocidade desejada {target_output_speed_mm_s:.2f} mm/s acima do permitido, limitando ao máximo {self.max_speed_output_mm_s:.2f} mm/s.")
            target_output_speed_mm_s = self.max_speed_output_mm_s

        # Calcula o RPM do motor do drum
        motor_drum_rpm = ((target_output_speed_mm_s / self.drum_circunference) * 60) * (
            self.teeth_drum / self.teeth_drum_motor
            # self.teeth_drum_motor / self.teeth_drum
        )
        #motor_drum_rpm = min(motor_drum_rpm, self.max_rpm_motor)
        if motor_drum_rpm > self.max_rpm_motor:
            rospy.logwarn(f"Drum RPM Calculado {motor_drum_rpm:.2f} rpm acima do permitido, limitanto ao máximo {self.max_rpm_motor:.2f} rpm.")
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
        rospy.loginfo(f"calculate_motor_commands::Motor Drum CMD: {motor_drum_cmd} ; Motor Screw CMD: {motor_screw_cmd}")
        rospy.loginfo(f"calculate_motor_commands::Motor Drum RPM: {motor_drum_rpm:.3f} ; Motor Screw RPM: {motor_screw_rpm:.3f}")
        rospy.loginfo(f"calculate_motor_commands::Motor Drum RPS: {motor_drum_rpm/60.0:.3f} ; Motor Screw RPS: {motor_screw_rpm/60.0:.3f}")
        rospy.loginfo(f"calculate_motor_commands::Drum mm/s: {(motor_drum_rpm/60.0)*(self.teeth_drum_motor / self.teeth_drum):.3f} ; Screw mm/s: {(motor_screw_rpm/60.0)*self.screw_thread_pitch:.3f}")

        return motor_drum_cmd, motor_screw_cmd

    def stop_motors(self):
        # Envia comando de parar (stop) os motores
        drum_cmd = 0
        screw_cmd = 0
        self.motor_drum.set_speed(drum_cmd)
        self.motor_screw.set_speed(screw_cmd)
          

    def cmd_callback(self, msg: Float32):
        """Callback chamada quando uma nova mensagem de velocidade (Float32) é recebida."""
        target_speed_mm_s = msg.data # Velocidade desejada em mm/s (positivo para desenrolar, negativo para enrolar)
        
        rospy.loginfo(f"Speed received: {target_speed_mm_s:.2f} mm/s")
        
        # 1. Calcular comandos Dynamixel
        drum_cmd, screw_cmd = self.calculate_motor_commands(target_speed_mm_s)
        # rospy.loginfo(f"Calculated Command :: Drum CMD: {drum_cmd} | Screw CMD: {screw_cmd}")
        
        # 2. Enviar comandos para os motores
        self.motor_drum.set_speed(drum_cmd, is_debug=False)
        self.motor_screw.set_speed(screw_cmd, is_debug=False)

        # 3. alimenta timeout para o safety_check comparar se não excedeu self.timeout_duration
        self.last_msg_time = rospy.Time.now()

        # Teste por tempo determinado para verificar comprimento enrolado
        # self.test_enrolar(target_speed_mm_s)

        # 4. Publicar Feedback de status (opcional)
        self.status_pub.publish(String(data=f"Running at {target_speed_mm_s:.2f} mm/s. DXL CMD: {drum_cmd}"))

    def safety_check(self):
        if (rospy.Time.now() - self.last_msg_time) > self.timeout_duration:
            self.stop_motors()
            # rospy.logwarn(f"Safety Check stopped the motors. Time between cmd_speed_mm_s exceeded the timeout of {self.timeout_seconds:.2f} seconds")

    def shutdown(self):
        """Executado quando o nó é desligado (ex: Ctrl+C) para desabilitar torque e fechar a porta."""
        rospy.logwarn("ROS Node Shutting Down. Disabling motor torque...")
        if hasattr(self, 'motor_drum') and self.motor_drum:
            self.motor_drum.close_connection()
        if hasattr(self, 'motor_screw') and self.motor_screw:
            self.motor_screw.close_connection()
        rospy.loginfo("Tether Dynamixel Node successfully shut down.")


# def main():
#     teeth_drum = 40
#     teeth_drum_motor = 50
#     cable_diameter = 0.9  # mm
#     drum_diameter = 65  # mm
#     drum_circunference = math.pi * drum_diameter  # mm
#     screw_thread_pitch = 16  # mm
#     max_rpm_drum_output = 65 * (teeth_drum_motor / teeth_drum)  # rpm
#     max_rps_output = max_rpm_drum_output / 60.0
#     max_speed_output_mm_s = int(max_rps_output * drum_circunference)
#     target_output_speed = 100  # mm/s
#     max_rpm_motor = 65
#     max_rps_motor = max_rpm_motor / 60.0
#     unit_scale = 0.229

#     target_output_speed = min(target_output_speed, max_speed_output_mm_s)

#     motor_drum_rpm = ((target_output_speed / drum_circunference) * 60) * (
#         teeth_drum / teeth_drum_motor
#     )
#     motor_drum_rpm = min(motor_drum_rpm, max_rpm_motor)
#     motor_drum_cmd = int(motor_drum_rpm / unit_scale) * -1

#     motor_screw_rpm = cable_diameter * (1 / screw_thread_pitch) * 60
#     motor_screw_cmd = int(motor_screw_rpm / unit_scale) * -1

#     print(f"max_rpm_drum_output: {max_rpm_drum_output}")
#     print(f"max_rps_output: {max_rps_output}")
#     print(f"max_speed_output_mm_s: {max_speed_output_mm_s} mm/s")
#     print(f"drum_circunference: {drum_circunference} mm\n")

#     print(f"motor_drum_rpm: {motor_drum_rpm} ({motor_drum_rpm/max_rpm_motor:.2f}%)")
#     print(f"motor_screw_rpm: {motor_screw_rpm} ({motor_screw_rpm/max_rpm_motor:.2f}%)")

#     print(f"motor_drum_cmd:{motor_drum_cmd}")
#     print(f"motor_screw_cmd:{motor_screw_cmd}")

#     motor_drum = DynamixelMotor(7, "/dev/ttyMotor", 1000000)
#     motor_drum.open_connection()
#     # reset watchdog send 0, wait and send desired ms
#     # motor_drum.set_bus_watchdog(0)
#     motor_drum.set_bus_watchdog(1000)

#     motor_screw = DynamixelMotor(2, "/dev/ttyMotor", 1000000)
#     motor_screw.open_connection()
#     # reset watchdog send 0, wait and send desired ms
#     # motor_screw.set_bus_watchdog(0)
#     motor_screw.set_bus_watchdog(1000)

#     print("Sleeeping...")
#     #for _ in tqdm(range(1)):
#     for _ in range(2):
#         motor_drum.set_speed(motor_drum_cmd, is_debug=True)
#         motor_screw.set_speed(motor_screw_cmd, is_debug=True)
#         time.sleep(0.5)
#     print("Finish")

#     motor_drum.set_speed(0)
#     motor_drum.set_torque_enabled(0)
#     motor_drum.close_connection()

#     motor_screw.set_speed(0)
#     motor_screw.set_torque_enabled(0)
#     motor_screw.close_connection()
    
def main(args=None):
    rclpy.init(args=args)

    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')

    motor_controller_node.destroy_node()
    
    rclpy.shutdown()    

if __name__ == '__main__':
    main()