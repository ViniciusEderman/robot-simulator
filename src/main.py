import pybullet as p
import pybullet_data
import time
import paho.mqtt.client as mqtt
import json
import math

# Configurações MQTT
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
COMMAND_TOPIC = "r2d2/command"


class R2D2Controller:
    def __init__(self):
        self.init_simulation()
        self.load_robot()
        self.setup_mqtt()
        self.add_obstacles()

        self.speed = 18          
        self.turn_speed = 12  
        self.current_action = "stop"
        self.obstacle_detected = False
        self.obstacle_distance_threshold = 0.5  

    def init_simulation(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.loadURDF("plane.urdf")

    def load_robot(self):
        start_pos = [0, 0, 0.5]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot = p.loadURDF("r2d2.urdf", start_pos, start_orientation)

        self.left_wheels = []
        self.right_wheels = []

        for joint_index in range(p.getNumJoints(self.robot)):
            joint_info = p.getJointInfo(self.robot, joint_index)
            joint_name = joint_info[1].decode('utf-8').lower()

            if "left" in joint_name and "wheel" in joint_name:
                self.left_wheels.append(joint_index)
            elif "right" in joint_name and "wheel" in joint_name:
                self.right_wheels.append(joint_index)

        print(f"Rodas esquerda: {self.left_wheels}, direita: {self.right_wheels}")
        p.changeDynamics(self.robot, -1, lateralFriction=2.0)

    def add_obstacles(self):
        """Adiciona obstáculos ao ambiente."""
        self.obstacles = [
            p.loadURDF("cube.urdf", [3, 0, 0.5], globalScaling=1.0),
            p.loadURDF("cube.urdf", [-3, 0, 0.5], globalScaling=1.0),
            p.loadURDF("cube.urdf", [0, 3, 0.5], globalScaling=1.0),
            p.loadURDF("cube.urdf", [0, -3, 0.5], globalScaling=1.0)
        ]
        
        for obstacle in self.obstacles:
            p.changeVisualShape(obstacle, -1, rgbaColor=[1, 0, 0, 1])

    def setup_mqtt(self):
        """Configura o cliente MQTT."""
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.subscribe(COMMAND_TOPIC)
        self.mqtt_client.loop_start()

    def on_message(self, client, userdata, msg):
        """comandos MQTT."""
        try:
            payload = json.loads(msg.payload.decode())
            action = payload.get("action", "").lower()
            if action in ["forward", "backward", "left", "right", "stop"]:
                self.current_action = action
                print(f"Comando recebido: {action}")
            else:
                print(f"Comando inválido: {action}")
        except Exception as e:
            print(f"Erro no processamento da mensagem MQTT: {e}")

    def check_obstacle_ahead(self):
        """Verifica se há obstáculos à frente do robô usando ray casting."""
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot)
        
        euler_angles = p.getEulerFromQuaternion(robot_orn)
        yaw = euler_angles[2] 
        
        ray_direction = [
            -math.sin(yaw),
            math.cos(yaw),  
            0              
        ]
        
        ray_start = [
            robot_pos[0] + ray_direction[0] * 0.3,
            robot_pos[1] + ray_direction[1] * 0.3,
            robot_pos[2] + 0.3  # Altura do sensor
        ]
        
        ray_end = [
            ray_start[0] + ray_direction[0] * self.obstacle_distance_threshold,
            ray_start[1] + ray_direction[1] * self.obstacle_distance_threshold,
            ray_start[2]
        ]
        
        ray_result = p.rayTest(ray_start, ray_end)
        
        if ray_result[0][0] != -1 and ray_result[0][0] != p.getBodyUniqueId(0):
            p.addUserDebugLine(ray_start, ray_end, [1, 0, 0], lifeTime=0.1)
            return True
        else:
            p.addUserDebugLine(ray_start, ray_end, [0, 1, 0], lifeTime=0.1)
            return False

    def move(self, left_speed, right_speed):
        """Aplica velocidade às rodas."""
        for wheel in self.left_wheels:
            p.setJointMotorControl2(self.robot, wheel, p.VELOCITY_CONTROL,
                                    targetVelocity=left_speed, force=50)

        for wheel in self.right_wheels:
            p.setJointMotorControl2(self.robot, wheel, p.VELOCITY_CONTROL,
                                    targetVelocity=right_speed, force=50)

    def execute_action(self):
        # Verifica se há obstáculos à frente apenas quando está se movendo para frente
        if self.current_action == "forward":
            self.obstacle_detected = self.check_obstacle_ahead()
        else:
            self.obstacle_detected = False

        if self.obstacle_detected and self.current_action == "forward":
            print("Obstáculo detectado! Parando...")
            self.move(0, 0)
        elif self.current_action == "backward":
            self.move(self.speed, self.speed)
        elif self.current_action == "forward":
            self.move(-self.speed, -self.speed)
        elif self.current_action == "right":
            self.move(-self.turn_speed, self.turn_speed)
        elif self.current_action == "left": 
            self.move(self.turn_speed, -self.turn_speed)
        else:
            self.move(0, 0)

    def run(self):
        try:
            while True:
                self.execute_action()
                p.stepSimulation()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Encerrando simulação...")
        finally:
            self.shutdown()

    def shutdown(self):
        """Finaliza a simulação e desconecta MQTT."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        p.disconnect()

if __name__ == "__main__":
    controller = R2D2Controller()
    controller.run()