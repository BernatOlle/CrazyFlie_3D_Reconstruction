def divide_point_cloud(points, center_x, center_y, center_z):
    # Crear listas vacías para cada cuadrante
    quadrants = {
        '000': [],
        '001': [],
        '010': [],
        '011': [],
        '100': [],
        '101': [],
        '110': [],
        '111': []
    }

    # Calcular los puntos medios para cada eje
    mid_x, mid_y, mid_z = center_x, center_y, center_z

    # Asignar cada punto al cuadrante correspondiente
    for point in points:
        x, y, z = point
        quadrant = ''

        if x >= mid_x:
            quadrant += '1'  # Cuadrante 1, 5, 6, 8
        else:
            quadrant += '0'

        if y >= mid_y:
            quadrant += '1'  # Cuadrante 1, 3, 6, 7
        else:
            quadrant += '0'

        if z >= mid_z:
            quadrant += '1'  # Cuadrante 1, 2, 5, 6
        else:
            quadrant += '0'

        # Agregar el punto al cuadrante correspondiente
        quadrants[quadrant].append(point)

    return quadrants

def calculate_point_cloud_center(point_cloud):
    # Inicializar las coordenadas X, Y y Z acumuladas
    sum_x, sum_y, sum_z = 0.0, 0.0, 0.0

    # Contar el número total de puntos en la nube
    total_points = len(point_cloud)

    # Sumar las coordenadas X, Y y Z de todos los puntos en la nube
    for point in point_cloud:
        x, y, z = point
        sum_x += x
        sum_y += y
        sum_z += z

    # Calcular el promedio de las coordenadas X, Y y Z para obtener el centro
    center_x = sum_x / total_points
    center_y = sum_y / total_points
    center_z = sum_z / total_points

    print(center_x, center_y, center_z)

    return center_x, center_y, center_z

# Ejemplo de uso
if __name__ == "__main__":
    # Lista de puntos en formato (x, y, z)
    points = [
        (1.0, 2.0, 3.0),
        (3.5, 2.3, 1.7),
        (-2.0, -1.0, 4.0),
        (-3.0, 2.0, -2.5),
        # Agregar más puntos aquí...
    ]
    center_x, center_y, center_z = calculate_point_cloud_center(points)
    # Límites del cubo contenedor (x_min, y_min, z_min, x_max, y_max, z_max)
    min_bound = (-5.0, -5.0, -5.0)
    max_bound = (5.0, 5.0, 5.0)

    # Dividir la nube de puntos en cuadrantes
    quadrants = divide_point_cloud(points, center_x, center_y, center_z)

    # Imprimir los puntos en cada cuadrante
    for quadrant, points_in_quadrant in quadrants.items():
        print(f'Cuadrante {quadrant}: {(points_in_quadrant)}')

