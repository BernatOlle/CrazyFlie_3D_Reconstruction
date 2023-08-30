'''import os
import matplotlib.pyplot as plt
import numpy as np
import pickle
import open3d as o3d
import time

def to_show(point_cloud):
        x,y,z = calculate_point_cloud_center(point_cloud)

        cloud_x = point_cloud[:, 0]
        cloud_y = point_cloud[:, 1]
        cloud_z = point_cloud[:, 2]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
 
        ax.set_xlim(x - 400, x + 400)
        ax.set_ylim(y - 400, y + 400)
        ax.set_zlim(z - 400, z + 400)

        # Dibujar los planos en dirección X, Y y Z
        ax.plot([x, x], [y - 600, y + 600], [z, z], color='blue', linestyle='dashed', linewidth=5.0)  # Plano YZ
        ax.plot([x - 600, x + 600], [y, y], [z, z], color='green', linestyle='dashed', linewidth=5.0)  # Plano XZ
        ax.plot([x, x], [y, y], [z - 600, z + 600], color='red', linestyle='dashed', linewidth=5.0)  # Plano XY


        # Dibujar el punto en el centro
        ax.scatter(x, y, z, color='red', marker='o')


        ax.scatter(cloud_x, cloud_y, cloud_z, color='blue', alpha=0.1)


        # Etiquetas de los ejes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Mostrar el gráfico
        plt.show()

def distancia_euclidiana(p1,p2):
    d = p2 - p1
    

    dist = (d[0]**2 + d[1]**2 + d[2]**2) ** 0.5
    return dist

def points_inside_sphere(point_cloud, center, r):
    puntos_dentro = 0
    punts_fora = 0

    for point in point_cloud:
        distancia = distancia_euclidiana(center, point)
        
        if distancia <= r:
            puntos_dentro += 1
        else:
            punts_fora += 1
    
    return puntos_dentro, punts_fora

def reduce_noise(point_cloud,point_cloud_color, r):
    new_point_cloud = np.zeros((1, 3))
    new_point_cloud_color = np.zeros((1, 3))

    for i,point in enumerate(point_cloud):

        points_inside, punts_fora = points_inside_sphere(point_cloud, point, r)
        print(points_inside, punts_fora, i)
        if(points_inside > 100):
            new_point_cloud = np.vstack([new_point_cloud, point])
            new_point_cloud_color = np.vstack([new_point_cloud_color, point_cloud_color[i]])

    return new_point_cloud, new_point_cloud_color

def to_ply(path, point_cloud, colors, name) -> None:
    """
    Generates the .ply which can be used to open the point cloud
    """
    out_points = (
        point_cloud.reshape(-1, 3) * 200 #ply
    )  # el 200 per fer la imatge 200 cops mes gran
    out_colors = colors.reshape(-1, 3)
    # print(out_colors.shape, out_points.shape)
    verts = np.hstack([out_points, out_colors])

    mean = np.mean(verts[:, :3], axis=0)
    scaled_verts = verts[:, :3] - mean
    dist = np.sqrt(
        scaled_verts[:, 0] ** 2 + scaled_verts[:, 1] ** 2 + scaled_verts[:, 2] ** 2
    )
    indx = np.where(dist < np.mean(dist) + 300)
    verts = verts[indx]
    ply_header = """ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar blue
        property uchar green
        property uchar red
        end_header
        """
    with open(path + "/" + name + ".ply", "w") as f:
        f.write(ply_header % dict(vert_num=len(verts)))
        np.savetxt(f, verts, "%f %f %f %d %d %d")
        
def calculate_density(point_cloud, cube_min, cube_max):
  
    point_cloud_array = np.array(point_cloud)

    
    points_min = np.min(point_cloud_array, axis=0)
    points_max = np.max(point_cloud_array, axis=0)

    cube_min = np.minimum(cube_min, points_min)
    cube_max = np.maximum(cube_max, points_max)
    print(cube_max,cube_min)

    points_inside_cube = point_cloud_array[
        (point_cloud_array[:, 0] >= cube_min[0]) & (point_cloud_array[:, 0] <= cube_max[0]) &
        (point_cloud_array[:, 1] >= cube_min[1]) & (point_cloud_array[:, 1] <= cube_max[1]) &
        (point_cloud_array[:, 2] >= cube_min[2]) & (point_cloud_array[:, 2] <= cube_max[2])
    ] 
   
    num_points_inside_cube = len(points_inside_cube)

    cube_volume = (cube_max[0] - cube_min[0]) * (cube_max[1] - cube_min[1]) * (cube_max[2] - cube_min[2])
    print(cube_volume)


    density = num_points_inside_cube / cube_volume

    return density

def divide_point_cloud(points, center_x, center_y, center_z):

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


    mid_x, mid_y, mid_z = center_x, center_y, center_z


    for point in points:
        x, y, z = point
        quadrant = ''

        if x >= mid_x:
            quadrant += '1' 
        else:
            quadrant += '0'

        if y >= mid_y:
            quadrant += '1' 
        else:
            quadrant += '0'

        if z >= mid_z:
            quadrant += '1'
        else:
            quadrant += '0'


        quadrants[quadrant].append(point)

    return quadrants

def calculate_point_cloud_center(point_cloud):
 
    sum_x, sum_y, sum_z = 0.0, 0.0, 0.0

    total_points = len(point_cloud)

   
    for point in point_cloud:
        x, y, z = point
        sum_x += x
        sum_y += y
        sum_z += z

   
    center_x = sum_x / total_points
    center_y = sum_y / total_points
    center_z = sum_z / total_points

    print(center_x, center_y, center_z)

    return center_x, center_y, center_z

if __name__ == "__main__":
    
    with open("final_point.pickle","rb") as file:
        point_cloud = pickle.load(file)

    with open("final_point_color.pickle","rb") as file:
        point_cloud_color = pickle.load(file)

    max_dif = 0
    dif = [0,0,0]
    k = 0
    new_point_cloud = np.zeros((1, 3))
    new_point_cloud_color = np.zeros((1, 3))

    init_time = time.time()

    new_point_cloud, new_point_cloud_color = reduce_noise(point_cloud,point_cloud_color, 0.5)
    
    print("Time", time.time()-init_time)
    for i,x in enumerate(point_cloud):
        if i > 0:

            # z
            dif = abs(x- first)
            print("Dif",dif,i)
            if (dif[2] > (previous_dif[2] * 2)) and not(previous_dif[2] == 0):
                print("Eliminar_z:",x,i)
                k += 1
            
            elif (dif[1] > (previous_dif[1] * 2)) and not(previous_dif[1] == 0):
                print("Eliminar_y:",x,i)
                k += 1
            elif (dif[0] > (previous_dif[0] * 2)) and not(previous_dif[0] == 0):
                print("Eliminar_x:",x,i)
                k += 1
            else:
                print(x)
                new_point_cloud = np.vstack([new_point_cloud, x])
                new_point_cloud_color = np.vstack([new_point_cloud_color, point_cloud_color[i]])
        else:
            new_point_cloud = np.vstack([new_point_cloud, x])
            new_point_cloud_color = np.vstack([new_point_cloud_color, point_cloud_color[i]])

        previous_dif = dif    
        first = x
    print("Before len:", len(point_cloud))
   
    print("After len:", len(new_point_cloud))
    
    img_final = os.getcwd()
    to_ply(
            img_final, new_point_cloud, new_point_cloud_color, "final_point_reduce_no"
        )
    
    #to_show(new_point_cloud)
    #point_cloud = new_point_cloud

    print("MAX",k)
    indices_ordenados = np.argsort(point_cloud[:, 2])[::-1]

    point_cloud_ordenado = point_cloud[indices_ordenados]

    np.savetxt("final_point_ordenado.txt",point_cloud_ordenado, fmt='%f')

    center_x, center_y, center_z = calculate_point_cloud_center(point_cloud)

    quadrants = divide_point_cloud(point_cloud, center_x, center_y, center_z)

    
    for quadrant, points_in_quadrant in quadrants.items():
        #print(f'Cuadrante {quadrant}: {(points_in_quadrant)}')
        
        cube_min = [float('inf'), float('inf'), float('inf')]
        cube_max = [float('-inf'), float('-inf'), float('-inf')]

        density = calculate_density(points_in_quadrant, cube_min, cube_max)
        print(f'Cuadrante {quadrant} densidad del cubo:', density)
'''    

import os
import time
import matplotlib.pyplot as plt
import numpy as np
import pickle
import open3d as o3d

def to_show(point_cloud):
        x,y,z = calculate_point_cloud_center(point_cloud)

        cloud_x = point_cloud[:, 0]
        cloud_y = point_cloud[:, 1]
        cloud_z = point_cloud[:, 2]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
 
        ax.set_xlim(x - 400, x + 400)
        ax.set_ylim(y - 400, y + 400)
        ax.set_zlim(z - 400, z + 400)

        # Dibujar los planos en dirección X, Y y Z
        ax.plot([x, x], [y - 600, y + 600], [z, z], color='blue', linestyle='dashed', linewidth=5.0)  # Plano YZ
        ax.plot([x - 600, x + 600], [y, y], [z, z], color='green', linestyle='dashed', linewidth=5.0)  # Plano XZ
        ax.plot([x, x], [y, y], [z - 600, z + 600], color='red', linestyle='dashed', linewidth=5.0)  # Plano XY


        # Dibujar el punto en el centro
        ax.scatter(x, y, z, color='red', marker='o')


        ax.scatter(cloud_x, cloud_y, cloud_z, color='blue', alpha=0.1)


        # Etiquetas de los ejes
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        # Mostrar el gráfico
        plt.show()

def distancia_euclidiana(p1,p2):
    d = p2 - p1
    

    dist = (d[0]**2 + d[1]**2 + d[2]**2) ** 0.5
    return dist

def points_inside_sphere(point_cloud, center, r):
    puntos_dentro = 0
    points_dentro_array = np.zeros((1,3))
    pos = []
    punts_fora = 0

    for i,point in enumerate(point_cloud):
        distancia = distancia_euclidiana(center, point)
        
        if distancia <= r:
            puntos_dentro += 1
            points_dentro_array = np.vstack([points_dentro_array, point])
            pos.append(i)
        else:
            punts_fora += 1
    
    return puntos_dentro, pos, points_dentro_array

def reduce_noise(point_cloud,point_cloud_color, r):
    point_cloud_list = point_cloud.tolist()
    point_cloud_dic = {index: value for index, value in enumerate(point_cloud_list)}

    after_key_list = 0

    copy_point_cloud = np.copy(point_cloud)
    i = 0
    actual_key = 0
    total_pos = []
    while(i< len(point_cloud_dic)):
        
        point = copy_point_cloud[actual_key]
        points_inside, pos, points_array = points_inside_sphere(copy_point_cloud, point, r)
        before_key_list = list(point_cloud_dic.keys())
        print(points_inside, actual_key)
        

        if(points_inside > 10):
            
            con_total_pos = set(total_pos)
            con_pos = set(pos)

            all_pos = con_total_pos.union(con_pos)

            total_pos = list(all_pos)

            pos.sort(reverse=True)
            for j in pos:
                if j != actual_key and j in point_cloud_dic:
                    point_cloud_dic.pop(j)

            
        after_key_list = list(point_cloud_dic.keys())
        actual_index = before_key_list.index(actual_key)
        next_index = (actual_index + 1) % len(after_key_list)
        actual_key = after_key_list[next_index]
        print("Actual key",actual_key)
        i +=1
        print("len", len(point_cloud_dic))
        
    return total_pos

def to_ply(path, point_cloud, colors, name) -> None:
    """
    Generates the .ply which can be used to open the point cloud
    """
    out_points = (
        point_cloud.reshape(-1, 3) * 200 #ply
    )  # el 200 per fer la imatge 200 cops mes gran
    out_colors = colors.reshape(-1, 3)
    # print(out_colors.shape, out_points.shape)
    verts = np.hstack([out_points, out_colors])

    mean = np.mean(verts[:, :3], axis=0)
    scaled_verts = verts[:, :3] - mean
    dist = np.sqrt(
        scaled_verts[:, 0] ** 2 + scaled_verts[:, 1] ** 2 + scaled_verts[:, 2] ** 2
    )
    indx = np.where(dist < np.mean(dist) + 300)
    verts = verts[indx]
    ply_header = """ply
        format ascii 1.0
        element vertex %(vert_num)d
        property float x
        property float y
        property float z
        property uchar blue
        property uchar green
        property uchar red
        end_header
        """
    with open(path + "/" + name + ".ply", "w") as f:
        f.write(ply_header % dict(vert_num=len(verts)))
        np.savetxt(f, verts, "%f %f %f %d %d %d")
        
def calculate_density(point_cloud, cube_min, cube_max):
  
    point_cloud_array = np.array(point_cloud)

    
    points_min = np.min(point_cloud_array, axis=0)
    points_max = np.max(point_cloud_array, axis=0)

    cube_min = np.minimum(cube_min, points_min)
    cube_max = np.maximum(cube_max, points_max)
    print(cube_max,cube_min)

    points_inside_cube = point_cloud_array[
        (point_cloud_array[:, 0] >= cube_min[0]) & (point_cloud_array[:, 0] <= cube_max[0]) &
        (point_cloud_array[:, 1] >= cube_min[1]) & (point_cloud_array[:, 1] <= cube_max[1]) &
        (point_cloud_array[:, 2] >= cube_min[2]) & (point_cloud_array[:, 2] <= cube_max[2])
    ] 
   
    num_points_inside_cube = len(points_inside_cube)

    cube_volume = (cube_max[0] - cube_min[0]) * (cube_max[1] - cube_min[1]) * (cube_max[2] - cube_min[2])
    print(cube_volume)


    density = num_points_inside_cube / cube_volume

    return density

def divide_point_cloud(points, center_x, center_y, center_z):

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


    mid_x, mid_y, mid_z = center_x, center_y, center_z


    for point in points:
        x, y, z = point
        quadrant = ''

        if x >= mid_x:
            quadrant += '1' 
        else:
            quadrant += '0'

        if y >= mid_y:
            quadrant += '1' 
        else:
            quadrant += '0'

        if z >= mid_z:
            quadrant += '1'
        else:
            quadrant += '0'


        quadrants[quadrant].append(point)

    return quadrants

def calculate_point_cloud_center(point_cloud):
 
    sum_x, sum_y, sum_z = 0.0, 0.0, 0.0

    total_points = len(point_cloud)

   
    for point in point_cloud:
        x, y, z = point
        sum_x += x
        sum_y += y
        sum_z += z

   
    center_x = sum_x / total_points
    center_y = sum_y / total_points
    center_z = sum_z / total_points

    print(center_x, center_y, center_z)

    return center_x, center_y, center_z

if __name__ == "__main__":
    
    with open("final_point.pickle","rb") as file:
        point_cloud = pickle.load(file)

    with open("final_point_color.pickle","rb") as file:
        point_cloud_color = pickle.load(file)

    max_dif = 0
    dif = [0,0,0]
    k = 0
    new_point_cloud = np.zeros((1, 3))
    new_point_cloud_color = np.zeros((1, 3))
    
    init_time = time.time()
    pos = reduce_noise(point_cloud,point_cloud_color, 0.2)

    print("Time", time.time()-init_time)
    pos_np = np.array(pos)

    new_point_cloud = point_cloud[pos_np]
    new_point_cloud_color = point_cloud_color[pos_np]
    
    
    '''for i,x in enumerate(point_cloud):
        if i > 0:

            # z
            dif = abs(x- first)
            print("Dif",dif,i)
            if (dif[2] > (previous_dif[2] * 2)) and not(previous_dif[2] == 0):
                print("Eliminar_z:",x,i)
                k += 1
            
            elif (dif[1] > (previous_dif[1] * 2)) and not(previous_dif[1] == 0):
                print("Eliminar_y:",x,i)
                k += 1
            elif (dif[0] > (previous_dif[0] * 2)) and not(previous_dif[0] == 0):
                print("Eliminar_x:",x,i)
                k += 1
            else:
                print(x)
                new_point_cloud = np.vstack([new_point_cloud, x])
                new_point_cloud_color = np.vstack([new_point_cloud_color, point_cloud_color[i]])
        else:
            new_point_cloud = np.vstack([new_point_cloud, x])
            new_point_cloud_color = np.vstack([new_point_cloud_color, point_cloud_color[i]])

        previous_dif = dif    
        first = x
    print("Before len:", len(point_cloud))
   
    print("After len:", len(new_point_cloud))
    '''
    img_final = os.getcwd()
    to_ply(
            img_final, new_point_cloud, new_point_cloud_color, "final_point_reduced"
        )
    
    #to_show(new_point_cloud)
    #point_cloud = new_point_cloud

    print("MAX",k)
    indices_ordenados = np.argsort(point_cloud[:, 2])[::-1]

    point_cloud_ordenado = point_cloud[indices_ordenados]

    np.savetxt("final_point_ordenado.txt",point_cloud_ordenado, fmt='%f')

    center_x, center_y, center_z = calculate_point_cloud_center(point_cloud)

    quadrants = divide_point_cloud(point_cloud, center_x, center_y, center_z)

    
    for quadrant, points_in_quadrant in quadrants.items():
        #print(f'Cuadrante {quadrant}: {(points_in_quadrant)}')
        
        cube_min = [float('inf'), float('inf'), float('inf')]
        cube_max = [float('-inf'), float('-inf'), float('-inf')]

        density = calculate_density(points_in_quadrant, cube_min, cube_max)
        print(f'Cuadrante {quadrant} densidad del cubo:', density)
    