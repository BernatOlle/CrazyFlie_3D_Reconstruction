import numpy as np

def rotation_matrix_3d(yaw, pitch, roll):
    # Convertir los ángulos de grados a radianes
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)

    # Construir las matrices de rotación individuales
    R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                      [np.sin(yaw), np.cos(yaw), 0],
                      [0, 0, 1]])

    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])

    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

    # Combinar las matrices de rotación en el orden ZYX
    R_combined = np.matmul(R_yaw, np.matmul(R_pitch, R_roll))

    return R_combined

# Ejemplo de ángulos en grados (Yaw, Pitch, Roll)
yaw_angle = 0
pitch_angle = 0
roll_angle = 90

# Calcular la matriz de rotación 3D
rotation_matrix = rotation_matrix_3d(yaw_angle, pitch_angle, roll_angle)
print(rotation_matrix)