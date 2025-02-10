import pybullet as p
import pybullet_data
import time

# Parámetros de la simulación
g = 9.81  # Gravedad en m/s²
dt = 0.01  # Paso de tiempo en segundos
y0 = 3.0   # Altura inicial en metros
v0 = 0.0   # Velocidad inicial
radius = 0.2  # Radio de la esfera

# Inicializar PyBullet en modo GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Crear el suelo
plane_id = p.loadURDF("plane.urdf")

# Crear la esfera en la posición inicial (ajustada para que toque el suelo correctamente)
sphere_visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius)
sphere_collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
sphere_id = p.createMultiBody(baseMass=10,
                              baseCollisionShapeIndex=sphere_collision_shape_id,
                              baseVisualShapeIndex=sphere_visual_shape_id,
                              basePosition=[0, 0, y0])

# Desactivar la gravedad de PyBullet para hacer la simulación manualmente
p.setGravity(0, 0, 0)

# Simulación manual usando MRUA
y = y0
v = v0
t = 0

while y > radius:  # 🔹 Ahora la esfera se detiene cuando su base toca el suelo
    time.sleep(dt)  # Mantener tiempo real

    # Aplicar ecuaciones de MRUA
    y = y0 + v0 * t - 0.5 * g * t**2  # Posición en función del tiempo
    v = v0 - g * t  # Velocidad en función del tiempo
    t += dt

    # Asegurar que la esfera no se mueva en otras direcciones
    p.resetBasePositionAndOrientation(sphere_id, [0, 0, y], [0, 0, 0, 1])

# Esperar un poco antes de cerrar
time.sleep(2)
p.disconnect()
