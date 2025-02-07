import pybullet as p
import pybullet_data
import time

# Iniciar el simulador en modo GUI
physicsClient = p.connect(p.GUI)

# Establecer la ubicación de los modelos URDF de PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Configurar el mundo
p.setGravity(0, 0, -9.81)  # Gravedad estándar

# Cargar el suelo plano
plane_id = p.loadURDF("plane.urdf")

# Posición inicial del robot
startPosition = [0, 0, 0.2]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Cargar el robot con base fija para que no se mueva
robotId = p.loadURDF("robot1.urdf", startPosition, startOrientation, useFixedBase=True)

# Obtener el número de joints para verificar el índice correcto
num_joints = p.getNumJoints(robotId)
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    print(f"Joint {i}: {joint_info[1].decode('utf-8')}")

# Asegurar que el joint correcto permita rotación en Z
jointIndex = 1  # Cambia esto si el joint correcto tiene otro índice

# Desactivar el motor para permitir giro libre
p.setJointMotorControl2(robotId, jointIndex, controlMode=p.VELOCITY_CONTROL, force=0)

# Crear sliders en la GUI de PyBullet para ajustar fricción y torque
friction_slider = p.addUserDebugParameter("jointFriction", 0.0, 5.0, 0.2)
torque_slider = p.addUserDebugParameter("jointTorque", -5.0, 5.0, 0.0)

# Mantener la simulación corriendo
while True:
    p.stepSimulation()
    
    # Leer valores de los sliders en cada iteración
    friction_value = p.readUserDebugParameter(friction_slider)
    torque_value = p.readUserDebugParameter(torque_slider)

    # Aplicar fricción al joint dinámicamente
    p.changeDynamics(robotId, jointIndex, jointDamping=friction_value)

    # Aplicar torque al joint
    p.setJointMotorControl2(robotId, jointIndex, controlMode=p.TORQUE_CONTROL, force=torque_value)

    time.sleep(1.0 / 240.0)  # Pequeña pausa para la simulación
