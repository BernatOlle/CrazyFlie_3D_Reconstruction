"""import pickle
import numpy as np
import matplotlib.pyplot as plt


with open('tran_matrix_final.pickle', 'rb') as file:
            transform_array_f = pickle.load(file)


print(transform_array_f[3])

x_coordinates = [x[0] for x in transform_array_f.values()]
y_coordinates = [x[1] for x in transform_array_f.values()]

plt.scatter(x_coordinates, y_coordinates)
plt.show()

"""

import numpy as np
import matplotlib.pyplot as plt
import pickle
from math import atan2, asin, acos

def rotation_matrix_to_euler_angles(R):
    # Calcula el ángulo de yaw (Y)
    yaw = atan2(R[1, 0], R[0, 0])

    # Calcula el ángulo de pitch (X)
    pitch = atan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))

    # Calcula el ángulo de roll (Z)
    roll = atan2(R[2, 1], R[2, 2])

    return yaw, pitch, roll

def add_yaw_90_degrees(rot_matrix):
    angle_degrees = 90
    angle_radians = np.radians(angle_degrees)

    cos_theta = np.cos(angle_radians)
    sin_theta = np.sin(angle_radians)

    # Matriz de rotación adicional alrededor del eje z
    additional_rotation = np.array(
        [[cos_theta, -sin_theta, 0], [sin_theta, cos_theta, 0], [0, 0, 1]]
    )

    # Sumar la rotación adicional a la matriz de rotación existente
    new_rot_matrix = additional_rotation @ rot_matrix

    return new_rot_matrix


with open("tran_matrix_final_ron.pickle", "rb") as file:
    tran_matrix_final = pickle.load(file)

with open("pose_expected_dron.pickle", "rb") as file:
    expected_pose_dron = pickle.load(file)     


print (tran_matrix_final)                

'''with open("tran_matrix_final.pickle0", "rb") as file:
    tran_matrix_final_0 = pickle.load(file)

with open("tran_matrix_final.pickle1", "rb") as file:
    tran_matrix_final_1 = pickle.load(file)

tran_matrix_final = {}

# Combinar el primer diccionario sin cambios
tran_matrix_final.update(tran_matrix_final_0)

# Combinar el segundo diccionario con cambio de keys
for key, value in tran_matrix_final_1.items():
    new_key = key + len(tran_matrix_final_0)  # Cambiamos la key para evitar colisiones
    tran_matrix_final[new_key] = value'''

etiquetas = list(tran_matrix_final.keys())
expected_pose_dron = np.array(list(expected_pose_dron.values()))

tran_matrix_final = dict(tran_matrix_final)

x_coordinates = [x[0][0] for x in tran_matrix_final.values()]
y_coordinates = [x[0][1] for x in tran_matrix_final.values()]
z_coordinates = [x[0][2] for x in tran_matrix_final.values()]
rot_matrices = [x[1] for x in tran_matrix_final.values()]

x_coordinates = [matriz[0] for matriz in x_coordinates]
y_coordinates = [matriz[0] for matriz in y_coordinates]
z_coordinates = [matriz[0] for matriz in z_coordinates]

'''x = expected_pose_dron[:,0]
y = expected_pose_dron[:,1]
z = expected_pose_dron[:,2]'''

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Dibuja los puntos en 3D
ax.scatter(x_coordinates, y_coordinates, z_coordinates, c='blue')
'''ax.scatter(x, y, z, c='red')'''

'''for etiqueta, xi, yi, zi in zip(etiquetas, x, y, z):
    ax.text(xi, yi, zi, etiqueta)'''

for etiqueta, xi, yi, zi in zip(etiquetas, x_coordinates, y_coordinates, z_coordinates):
    ax.text(xi, yi, zi, etiqueta)
    print (etiqueta)


for label, coords in tran_matrix_final.items():

    # Obtener la matriz de rotación asociada con el punto actual
    rot_matrix = rot_matrices[label - 1]

    # Calcular la dirección de la flecha usando la matriz de rotación
    direction = rot_matrix @ np.array([1, 0, 0])

    # Calcular la longitud de la flecha (por ejemplo, 0.2 veces la escala)
    arrow_length = 0.1
    # Dibujar la flecha en 3D
    ax.quiver(
        coords[0][0][0],
        coords[0][1][0],
        coords[0][2][0],
        direction[0] * arrow_length,
        direction[1] * arrow_length,
        direction[2] * arrow_length,
        color="red",
    )

plt.show()
