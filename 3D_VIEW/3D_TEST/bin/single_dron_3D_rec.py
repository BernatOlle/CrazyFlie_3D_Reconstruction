import multiprocessing as mp
from rembg import remove
import matplotlib.pyplot as plt
import time
import cv2
import numpy as np
import os
from scipy.optimize import least_squares
from tomlkit import boolean
import matplotlib
import pickle
from camera_socket import Network
import threading
import argparse

# from take_photo import Take
matplotlib.use("TkAgg")
# import pymeshlab

from swarm_drones import Dron, AIDECK


class Image_loader:
    def __init__(self, img_dir: str, downscale_factor: float):
        # loading the Camera intrinsic parameters K
        with open("images/" + img_dir + "/K.txt") as f:
            self.K = np.array(
                list(
                    (
                        map(
                            lambda x: list(
                                map(lambda x: float(x), x.strip().split(" "))
                            ),
                            f.read().split("\n"),
                        )
                    )
                )
            )

        self.path = os.getcwd()
        self.factor = downscale_factor
        self.downscale()

    def downscale(self) -> None:
        """
        Downscales the Image intrinsic parameter acc to the downscale factor
        """
        self.K[0, 0] /= self.factor
        self.K[1, 1] /= self.factor
        self.K[0, 2] /= self.factor
        self.K[1, 2] /= self.factor


class Sfm:
    def __init__(self, img_dir: str, downscale_factor: float = 1.0) -> None:
        """
        Initialise and Sfm object.
        """
        parser = argparse.ArgumentParser(description='Parameters for the 3D reconstruction')
        parser.add_argument("-d", type= int, default="1", metavar="number", help="Number of drones")  
        parser.add_argument("-t", type= int, default='1', metavar="trajectory", help="Predefined or dynamic trajectory")#1 = Dynamic trajectory, 2 = Predefined trajectory
        parser.add_argument('-p', type= int, default="1", metavar="position", help="Get or estimate camera position") #1 = Get for the drons, 2 = Estimate position
        args = parser.parse_args()

        n_drones = args.d
        trajectory = args.t
        positioning = args.p

        print(n_drones,trajectory,positioning)

        '''self.dron = Dron()
        self.cam = AIDECK("Test_1")'''

        self.img_obj = Image_loader(img_dir, downscale_factor)
        self.img_dir = img_dir
        print("DIR" + self.img_dir)

    def downscale_image(self, image):
        """
        Downscales image acc to the downscale factorcv2.
        pyrDown()" function is a pyramid reduction operation,
        which reduces the size of the image by a factor of 2.
        """
        for _ in range(1, int(self.img_obj.factor / 2) + 1):
            image = cv2.pyrDown(image)

        return image

    def remove_bg(self, img):
        # input_path = glob.glob(folder+"/*.JPG")
        # k=0
        # self.image_list_BG = []

        # output_path = folder+"BG/result"+str(10+k)+".jpg"
        # input = cv2.imread(img)

        output = remove(img)

        # eliminel la 4 component ja que es el valor alhpa que sobra en les nostres imatges
        output = np.delete(output, 3, axis=2)

        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        # Threshold the image to create a mask
        _, mask = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)

        # Invert the mask
        mask_inv = cv2.bitwise_not(mask)

        # Apply the mask to the image
        foreground = cv2.bitwise_and(output, output, mask=mask)

        # Create a white background image
        background = np.full(output.shape, 255, dtype=np.uint8)

        # Apply the inverted mask to the background
        background = cv2.bitwise_and(background, background, mask=mask_inv)

        # Combine the foreground and background
        result = cv2.add(foreground, background)

        # self.image_list_BG.append(result)
        # k+=1
        return result

    '''def mesh_representation(self, name):
        ms = pymeshlab.MeshSet()
        ms.load_new_mesh(self.img_obj.path + "/res/" + name + ".ply")

        ms.generate_simplified_point_cloud(
            samplenum=10000, bestsampleflag=True, bestsamplepool=10, exactnumflag=False
        )
        ms.compute_normal_for_point_clouds(
            k=3, smoothiter=0, flipflag=False, viewpos=[0, 0, 0]
        )
        ms.surface_reconstruction_ball_pivoting(
            clustering=20, creasethr=90, deletefaces=False
        )
        ms.meshing_remove_connected_component_by_face_number(
            mincomponentsize=25, removeunref=True
        )
        ms.save_current_mesh(self.img_obj.path + "/res_fin/final_" + name + ".ply")

        # cloud = o3d.io.read_triangle_mesh(self.img_obj.path+'/res_fin/final_'+name+'.ply')
        # o3d.visualization.draw_geometries([cloud])
'''
    def triangulation(
        self, point_2d_1, point_2d_2, projection_matrix_1, projection_matrix_2
    ) -> tuple:
        """
        Triangulates 3d points from 2d vectors and projection matrices
        returns projection matrix of first camera, projection matrix of second camera, point cloud
        """

        # performs triangulation of 3D points from multiple 2D views
        pt_cloud = cv2.triangulatePoints(
            point_2d_1, point_2d_2, projection_matrix_1.T, projection_matrix_2.T
        )

        projection_matrix_1 = projection_matrix_1.T
        projection_matrix_2 = projection_matrix_2.T
        pt_cloud /= pt_cloud[3]  # quan es volen obtenir les coordenades
        # en una escala real, es divideixen les
        # coordenades (x, y, z) per w, de manera
        # que les primeres tres columnes de la matriu
        # representin les coordenades tridimensionals
        # reals (x, y, z). Homogenius matrix

        return projection_matrix_1, projection_matrix_2, pt_cloud

    def PnP(self, obj_point, image_point, K, dist_coeff, rot_vector, initial) -> tuple:
        """
        Camera pose estimation using the RANSAC scheme!
        Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
        returns rotational matrix, translational matrix, image points, object points, rotational vector

        obj_point: An array of N 3D points in the object coordinate system.
        image_point: An array of N 2D points in the image coordinate system.
        K: The 3x3 intrinsic matrix of the camera.
        dist_coeff: The distortion coefficients of the camera.
        rot_vector: An array of N rotation vectors.
        initial: An integer flag indicating the orientation of the input data.

        """
        if initial == 1:
            obj_point = obj_point[:, 0, :]
            image_point = image_point.T
            rot_vector = rot_vector.T

        # print(obj_point.shape)
        # print(image_point.shape)
        _, rot_vector_calc, tran_vector, inlier = cv2.solvePnPRansac(
            obj_point, image_point, K, dist_coeff, cv2.SOLVEPNP_ITERATIVE
        )
        # Converts a rotation matrix to a rotation vector or vice versa
        rot_matrix, _ = cv2.Rodrigues(rot_vector_calc)

        if inlier is not None:
            image_point = image_point[inlier[:, 0]]
            obj_point = obj_point[inlier[:, 0]]
            rot_vector = rot_vector[inlier[:, 0]]
        return rot_matrix, tran_vector, image_point, obj_point, rot_vector

    def reprojection_error(
        self, obj_points, image_points, transform_matrix, K, homogenity
    ) -> tuple:
        """
        Calculates the reprojection error ie the distance between the projected points and the actual points.
        returns total error, object points
        """
        rot_matrix = transform_matrix[:3, :3]
        tran_vector = transform_matrix[:3, 3]
        rot_vector, _ = cv2.Rodrigues(rot_matrix)

        # print("Berfore Dimensiones de matrix:",obj_points.ndim)
        if homogenity == 1:
            obj_points = cv2.convertPointsFromHomogeneous(obj_points.T)

        # print("After Dimensiones de matrix:",obj_points.ndim)
        # function to project the 3D object points onto the image plane
        image_points_calc, _ = cv2.projectPoints(
            obj_points, rot_vector, tran_vector, K, None
        )
        image_points_calc = np.float32(image_points_calc[:, 0, :])
        # image points are then compared to the input 2D image points
        total_error = cv2.norm(
            image_points_calc,
            np.float32(image_points.T) if homogenity == 1 else np.float32(image_points),
            cv2.NORM_L2,
        )
        # divided by the number of points to get the average error per point
        return total_error / len(image_points_calc), obj_points

    def optimal_reprojection_error(self, obj_points) -> np.array:
        """
        calculates of the reprojection error during bundle adjustment
        returns error
        More or less the same as the fuction on the top
        """
        transform_matrix = obj_points[0:12].reshape((3, 4))
        K = obj_points[12:21].reshape((3, 3))
        rest = int(len(obj_points[21:]) * 0.4)
        p = obj_points[21 : 21 + rest].reshape((2, int(rest / 2))).T
        obj_points = obj_points[21 + rest :].reshape(
            (int(len(obj_points[21 + rest :]) / 3), 3)
        )
        rot_matrix = transform_matrix[:3, :3]
        tran_vector = transform_matrix[:3, 3]
        rot_vector, _ = cv2.Rodrigues(rot_matrix)
        image_points, _ = cv2.projectPoints(
            obj_points, rot_vector, tran_vector, K, None
        )
        image_points = image_points[:, 0, :]
        error = [(p[idx] - image_points[idx]) ** 2 for idx in range(len(p))]
        return np.array(error).ravel() / len(p)

    def bundle_adjustment(
        self, _3d_point, opt, transform_matrix_new, K, r_error
    ) -> tuple:
        """
        Bundle adjustment for the image and object points
        returns object points, image points, transformation matrix
        """

        # ravel transforms all into 1D dimension

        opt_variables = np.hstack((transform_matrix_new.ravel(), K.ravel()))
        opt_variables = np.hstack((opt_variables, opt.ravel()))
        opt_variables = np.hstack((opt_variables, _3d_point.ravel()))

        # opt_variables is a matrix that contains the transform_matrix_new,
        # K,opt,_3d_point

        values_corrected = least_squares(
            self.optimal_reprojection_error, opt_variables, gtol=r_error
        ).x
        K = values_corrected[12:21].reshape((3, 3))
        rest = int(len(values_corrected[21:]) * 0.4)

        points_3d = values_corrected[21 + rest :].reshape(
            (int(len(values_corrected[21 + rest :]) / 3), 3)
        )
        mask = values_corrected[21 : 21 + rest].reshape((2, int(rest / 2))).T
        transform_matrix = values_corrected[0:12].reshape((3, 4))

        return points_3d, mask, transform_matrix

    def to_ply(self, path, point_cloud, colors) -> None:
        """
        Generates the .ply which can be used to open the point cloud
        """
        out_points = (
            point_cloud.reshape(-1, 3) #* 200
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
        with open(path + "/res/" + self.img_dir + ".ply", "w") as f:
            f.write(ply_header % dict(vert_num=len(verts)))
            np.savetxt(f, verts, "%f %f %f %d %d %d")

    def to_show(self, index, mlist, nrows, ncols, rotation, final, fig):
        act_index = 0
        while final.value == False:
            if index.value > act_index:
                number = index.value

                xmin, xmax = -900, 800  # set x axis limit
                ymin, ymax = -800, 1200  # set x axis limit
                zmin, zmax = 1000, 2100  # set x axis limit
                point_cloud = np.array(mlist[:])
                # print("Print point_clouds "+str(number) +": "+str(point_cloud.shape))
                point_cloud = point_cloud.reshape(nrows.value, ncols.value)

                """
                xmin, xmax = -10000, 10000 # set x axis limit
                ymin, ymax = -10000, 10000 # set x axis limit
                zmin, zmax = -10000, 10000 # set x axis limit
                """
                ax = fig.add_subplot(111, projection="3d")
                ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], s=1)

                ax.set_xlim(xmin, xmax)
                ax.set_ylim(ymin, ymax)
                ax.set_zlim(zmin, zmax)

                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")

                ax.view_init(elev=70 + rotation.value, azim=rotation.value)

                plt.show(block=False)
                plt.pause(1)
                act_index = index.value
            plt.clf()
        plt.close()
        print("Final thread")

    # feature_1, features_cur, features_2
    # I don't really know what it do, or how this fuction can be useful

    def common_points(self, image_points_1, image_points_2, image_points_3) -> tuple:
        cm_points_1 = []
        cm_points_2 = []
        for i in range(image_points_1.shape[0]):
            a = np.where(image_points_2 == image_points_1[i, :])
            # print(a[0])
            if a[0].size != 0:
                cm_points_1.append(i)
                cm_points_2.append(a[0][0])

        mask_array_1 = np.ma.array(image_points_2, mask=False)
        mask_array_1.mask[cm_points_2] = True
        mask_array_1 = mask_array_1.compressed()
        mask_array_1 = mask_array_1.reshape(int(mask_array_1.shape[0] / 2), 2)

        mask_array_2 = np.ma.array(image_points_3, mask=False)
        mask_array_2.mask[cm_points_2] = True
        mask_array_2 = mask_array_2.compressed()
        mask_array_2 = mask_array_2.reshape(int(mask_array_2.shape[0] / 2), 2)
        return np.array(cm_points_1), np.array(cm_points_2), mask_array_1, mask_array_2

    def find_features(self, image_0, image_1, file_0, file_1) -> tuple:
        """
        Feature detection using the sift algorithm and KNN
        return keypoints(features) of image_1 and image2
        """

        # sift = cv2.xfeatures2d.SIFT_create()
        sift = cv2.SIFT_create()
        key_points_0, desc_0 = sift.detectAndCompute(
            cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY), None
        )
        key_points_1, desc_1 = sift.detectAndCompute(
            cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY), None
        )

        bf = cv2.BFMatcher()
        matches = bf.knnMatch(desc_0, desc_1, k=2)
        feature = []
        pts1 = []
        pts2 = []
        img_kp = cv2.drawKeypoints(image_0, key_points_0, None)
        img_kp1 = cv2.drawKeypoints(image_1, key_points_1, None)

        cv2.imshow("file_1", img_kp)
        cv2.imshow("file_0", img_kp1)

        cv2.waitKey(1000)

        for m, n in matches:
            if m.distance < 0.70 * n.distance:
                feature.append(m)
                pts1.append(key_points_0[m.queryIdx].pt)
                pts2.append(key_points_1[m.trainIdx].pt)

        return np.array(pts1), np.array(pts2)

    def recal(self, imgs, image_2, file_1):
        t_points_3d = []
        max = 0
        pose_array = self.img_obj.K.ravel()
        print(self.vect.keys())
        down = 0

        for k, file in enumerate(imgs[2:-2], start=2):
            print("Recal", file)
            dir = "images/" + self.img_dir
            file_path = os.path.join(dir, file)
            print(file_path)

            img1 = cv2.imread(file_path)
            image_1 = self.downscale_image(img1)
            image_1 = self.remove_bg(image_1)

            print("Insert new points recal", k)
            pose_0, pose_1, feature_0, feature_1 = self.vect[k]

            features_cur, features_2 = self.find_features(
                image_1, image_2, file, file_1
            )

            try:
                feature_0, feature_1, points_3d = self.triangulation(
                    pose_0, pose_1, feature_0, feature_1
                )

                feature_1 = feature_1.T
                points_3d = cv2.convertPointsFromHomogeneous(points_3d.T)
                points_3d = points_3d[:, 0, :]

                cm_points_0, cm_points_1, cm_mask_0, cm_mask_1 = self.common_points(
                    feature_1, features_cur, features_2
                )
                # print("Previous parameters \n", feature_0, feature_1, features_cur, features_2)
                # print(k,cm_points_0,cm_points_1)

                cm_points_2 = features_2[cm_points_1]
                cm_points_cur = features_cur[cm_points_1]

                # print(cm_points_2,cm_points_cur)

                (
                    rot_matrix,
                    tran_matrix,
                    cm_points_2,
                    points_3d,
                    cm_points_cur,
                ) = self.PnP(
                    points_3d[cm_points_0],
                    cm_points_2,
                    self.img_obj.K,
                    np.zeros((5, 1), dtype=np.float32),
                    cm_points_cur,
                    initial=0,
                )

                transform_matrix_1 = np.hstack((rot_matrix, tran_matrix))
                pose_2 = np.matmul(self.img_obj.K, transform_matrix_1)

                error, points_3d = self.reprojection_error(
                    points_3d,
                    cm_points_2,
                    transform_matrix_1,
                    self.img_obj.K,
                    homogenity=0,
                )

                cm_mask_0, cm_mask_1, points_3d = self.triangulation(
                    pose_1, pose_2, cm_mask_0, cm_mask_1
                )

                error, points_3d = self.reprojection_error(
                    points_3d,
                    cm_mask_1,
                    transform_matrix_1,
                    self.img_obj.K,
                    homogenity=1,
                )
                # print("Dimensiones de matrix:",points_3d.ndim)
                print("Reprojection Error: ", error)
                print("New points: ", len(points_3d))
                # print("Dimensions recal final:", points_3d.ndim)

            except Exception as e:
                print("ERROR recal fin", e)
                continue
            print("Length t_points", len(t_points_3d))

            if len(t_points_3d) < len(points_3d):
                print("total " + str(len(t_points_3d)) + " curr " + str(len(points_3d)))
                t_points_3d = np.copy(points_3d)
                ret_cm_mask_1 = np.copy(cm_mask_1)
                max = k
                max_img = file
                print(max)
                down = 0
            elif ((len(t_points_3d)) - len(points_3d)) > 300:
                down += 1
            if down == 1:
                return ret_cm_mask_1, t_points_3d
                break
            print("Down " + str(down))
            print()
        return ret_cm_mask_1, t_points_3d

    def __call__(self, enable_bundle_adjustment: boolean = False):
        dir = "images/" + self.img_dir
        transform_matrix_0 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        transform_matrix_1 = np.empty((3, 4))
        pose_array = self.img_obj.K.ravel()
        fig = plt.figure(figsize=(7, 15))
        n_points = 20
        max_points_quality = 60000
        
        '''self.dron.init_dron()
        self.dron.do_take_off()
        seq = self.dron.sequence_point(n_points,0.6,0.6,0)'''
        pose_array_final = {}
        transform_matrix_1_array = {}

        rotation = 0
        pose_0 = np.matmul(self.img_obj.K, transform_matrix_0)

        pose_1 = np.empty((3, 4))
        total_points = np.zeros((1, 3))
        total_colors = np.zeros((1, 3))
        imgs = sorted(
            [file for file in os.listdir(dir) if file.lower().endswith(".jpg")]
        )
        print(imgs)
        i = 0
        tran_matrix_final = {}
        tran_matrix_final_drone = {}
        self.vect = {}
        start_time = time.time()
        pose_array_final[i] = pose_0
        while True:
            print("------------------------------------------")
            '''if not(len(seq) == 1):
                point = seq[1]
            stop_event = threading.Event()
            get_pos = threading.Event()  
            thread = threading.Thread(target=self.dron.do_next_point, args=(seq,0,stop_event,get_pos))
            thread.start()
            time.sleep(2)    
            self.cam.take_photo()
            get_pos.set()
            self.dron.do_get_pos(get_pos)
            print("Get dron position ")'''
            
            if len(imgs) > i:
                print("Inici",i)
                file_0 = imgs[i]
                print(file_0)
                if file_0.lower().endswith(".jpg"):
                    if i < 2:
                        file_path = os.path.join(dir, file_0)
                        img0 = cv2.imread(file_path)
                        image_1 = self.remove_bg(img0)
                        image_1 = self.downscale_image(image_1)

                        if i > 0:
                            feature_0, feature_1 = self.find_features(
                                image_0, image_1, file_0, file_1
                            )

                            essential_matrix, em_mask = cv2.findEssentialMat(
                                feature_0,
                                feature_1,
                                self.img_obj.K,
                                method=cv2.RANSAC,
                                prob=0.999,
                                threshold=0.4,
                                mask=None,
                            )
                            feature_0 = feature_0[em_mask.ravel() == 1]
                            feature_1 = feature_1[em_mask.ravel() == 1]

                            # nomes em quedo amb la essential_matrix

                            '''tran_matrix_drone = np.array(self.dron.position)[:3].reshape(-1, 1)
                            rot_matrix_drone = self.dron.get_rotation_matrix(self.dron.angle)

                            tran_matrix_final_drone[i] = [tran_matrix_drone, rot_matrix_drone]
'''
                            _, rot_matrix, tran_matrix, em_mask = cv2.recoverPose(
                                essential_matrix, feature_0, feature_1, self.img_obj.K
                            )


                            tran_matrix_final[i] = [tran_matrix, rot_matrix]

                            feature_0 = feature_0[em_mask.ravel() > 0]
                            feature_1 = feature_1[em_mask.ravel() > 0]

                            transform_matrix_1[:3, :3] = np.matmul(
                                rot_matrix, transform_matrix_0[:3, :3]
                            )  # els 3x3 primers la multiplicacio dels primers 3x3 de la matriu de trans 0 i la matriu de rotacio
                            transform_matrix_1[:3, 3] = transform_matrix_0[
                                :3, 3
                            ] + np.matmul(
                                transform_matrix_0[:3, :3], tran_matrix.ravel()
                            )


                            transform_matrix_1_array[i] = transform_matrix_1

                            pose_1 = np.matmul(self.img_obj.K, transform_matrix_1)

                            pose_array_final[i] = pose_1

                            feature_0, feature_1, points_3d = self.triangulation(
                                pose_0, pose_1, feature_0, feature_1
                            )

                            error, points_3d = self.reprojection_error(
                                points_3d,
                                feature_1,
                                transform_matrix_1,
                                self.img_obj.K,
                                homogenity=1,
                            )
                            print("REPROJECTION ERROR: ", error)

                            # print("1r cop Dimensiones de matrix:",points_3d.ndim)

                            _, _, feature_1, points_3d, _ = self.PnP(
                                points_3d,
                                feature_1,
                                self.img_obj.K,
                                np.zeros((5, 1), dtype=np.float32),
                                feature_0,
                                initial=1,
                            )

                            # print("1r cop Dimensiones de matrix:",points_3d.ndim)
                            pose_array = np.hstack(
                                (
                                    np.hstack((pose_array, pose_0.ravel())),
                                    pose_1.ravel(),
                                )
                            )

                            threshold = 0.5
                            index = 1

                            manager = mp.Manager()

                            nrows = mp.Value("i", 0)
                            ncols = mp.Value("i", 0)
                            nrows.value, ncols.value = points_3d.shape
                            temp_points_3d = points_3d.reshape(-1)
                            copia_points_3d = manager.list()
                            copia_points_3d[:] = temp_points_3d

                            rotation = mp.Value("i", 0)
                            final = mp.Value("b", False)
                            index = mp.Value("i", 1)
                            print(
                                "Input point_cloud"
                                + str(index.value)
                                + ": "
                                + str(temp_points_3d.shape)
                            )
                            p = mp.Process(
                                target=self.to_show,
                                args=(
                                    index,
                                    copia_points_3d,
                                    nrows,
                                    ncols,
                                    rotation,
                                    final,
                                    fig,
                                ),
                            )
                            #p.start()
                            '''n_points = max_points_quality/len(points_3d)
                            seq = self.dron.sequence_point(int(n_points),0.6,0.6,point[3])
'''

                        image_0 = np.copy(image_1)
                        file_1 = file_0
                        i += 1
                        print("Continue")
                        ''' print("Event set")
                        stop_event.set()
                        self.dron.do_stop_event(stop_event)
                        
                        thread.join()'''
                        
                        continue

                    # print("Agafo totes les imatges")
                    file_path = os.path.join(dir, file_0)

                    rotation.value += 20

                    img1 = cv2.imread(file_path)
                    image_2 = self.downscale_image(img1)
                    image_2 = self.remove_bg(image_2)

                    features_cur, features_2 = self.find_features(
                        image_1, image_2, file_0, file_1
                    )

                    if i != 2:
                        
                        feature_0, feature_1, points_3d = self.triangulation(
                            pose_0, pose_1, feature_0, feature_1
                        )

                        feature_1 = feature_1.T
                        points_3d = cv2.convertPointsFromHomogeneous(points_3d.T)
                        points_3d = points_3d[:, 0, :]

                    cm_points_0, cm_points_1, cm_mask_0, cm_mask_1 = self.common_points(
                        feature_1, features_cur, features_2
                    )

                    # print("Features", feature_0,feature_1, features_cur, features_2)

                    # print(i,cm_points_0,cm_points_1)
                    '''tran_matrix_drone = np.array(self.dron.position)[:3].reshape(-1, 1)
                    rot_matrix_drone = self.dron.get_rotation_matrix(self.dron.angle)

                    tran_matrix_final_drone[i] = [tran_matrix_drone, rot_matrix_drone]
'''
                    cm_points_2 = features_2[cm_points_1]
                    cm_points_cur = features_cur[cm_points_1]

                    try:
                        (
                            rot_matrix,
                            tran_matrix,
                            cm_points_2,
                            points_3d,
                            cm_points_cur,
                        ) = self.PnP(
                            points_3d[cm_points_0],
                            cm_points_2,
                            self.img_obj.K,
                            np.zeros((5, 1), dtype=np.float32),
                            cm_points_cur,
                            initial=0,
                        )
                        # nomes depenen de K rot_matrix i tran_matrix
                        tran_matrix_final[i] = [tran_matrix, rot_matrix]
                        transform_matrix_1 = np.hstack(
                            (rot_matrix, tran_matrix)
                        )  # depen de K

                        transform_matrix_1_array[i] = transform_matrix_1  # depen de K
                        pose_2 = np.matmul(
                            self.img_obj.K, transform_matrix_1
                        )  # depen de K

                        pose_array_final[i] = pose_2


                        error, points_3d = self.reprojection_error(
                            points_3d,
                            cm_points_2,
                            transform_matrix_1,
                            self.img_obj.K,
                            homogenity=0,
                        )  # nomes error depen de K

                        cm_mask_0, cm_mask_1, points_3d = self.triangulation(
                            pose_1, pose_2, cm_mask_0, cm_mask_1
                        )

                        error, points_3d = self.reprojection_error(
                            points_3d,
                            cm_mask_1,
                            transform_matrix_1,
                            self.img_obj.K,
                            homogenity=1,
                        )  # nomes error depen de K
                        # print("Dimensiones de matrix:",points_3d.ndim)
                        print("Reprojection Error: ", error)
                        print("New points: ", len(points_3d))
                        pose_array = np.hstack((pose_array, pose_2.ravel()))
                        if len(points_3d) < 200:
                            cm_mask_1, points_3d = self.recal(imgs, image_2, file_0)

                    except Exception as e:
                        print(self.vect[e])
                        print("Error main", e)

                        cm_mask_1, points_3d = self.recal(imgs, image_2, file_0)

                    # takes a long time to run
                    color_vector = []
                    if enable_bundle_adjustment:
                        (
                            points_3d,
                            cm_mask_1,
                            transform_matrix_1,
                        ) = self.bundle_adjustment(
                            points_3d,
                            cm_mask_1,
                            transform_matrix_1,
                            self.img_obj.K,
                            threshold,
                        )
                        pose_2 = np.matmul(self.img_obj.K, transform_matrix_1)
                        error, points_3d = self.reprojection_error(
                            points_3d,
                            cm_mask_1,
                            transform_matrix_1,
                            self.img_obj.K,
                            homogenity=0,
                        )
                        print("Bundle Adjusted error: ", error)
                        total_points = np.vstack((total_points, points_3d))
                        points_left = np.array(cm_mask_1, dtype=np.int32)
                        print("Print:", points_left.T)
                        print("Image:", image_2.shape)
                        for l in points_left.T:
                            row = l[0]
                            col = l[1]

                            print("Row: ", row)
                            print("Col: ", col)
                            if (
                                row >= 0
                                and row < image_2.shape[0]
                                and col >= 0
                                and col < image_2.shape[1]
                            ):
                                color_vector.append(image_2[row, col])
                            else:
                                color_vector.append([0, 0, 0])
                            print(color_vector[0])

                        color_vector = np.array(color_vector)

                        # color_vector = np.array([image_2[l[1], l[0]] for l in points_left.T])

                        # print("Color",color_vector.shape)
                        total_colors = np.vstack((total_colors, color_vector))
                    else:
                        total_points = np.vstack((total_points, points_3d[:, 0, :]))
                        # print("hola")
                        print("Total points", total_points.shape)

                        points_left = np.array(cm_mask_1, dtype=np.int32)

                        color_vector = np.array(
                            [image_2[l[1], l[0]] for l in points_left.T]
                        )
                        # print("Color")

                        # print("Fin")
                        total_colors = np.vstack((total_colors, color_vector))
                        print("Total colors", total_colors.shape)

                    transform_matrix_0 = np.copy(transform_matrix_1)
                    pose_0 = np.copy(pose_1)
                    # image_0 = np.copy(image_1)
                    image_1 = np.copy(image_2)
                    feature_0 = np.copy(features_cur)
                    feature_1 = np.copy(features_2)
                    pose_1 = np.copy(pose_2)
                    file_1 = file_0
                    self.vect[i] = [pose_0, pose_1, feature_0, feature_1]
                    output_array = (
                        total_points.reshape(-1, 3) #* 200
                    )  # Serveix per fer la imatge mes gran

                    # si la length de la copia_points_3d es mes petita que la de output array que s'afegeixin 0s
                    nrows.value, ncols.value = output_array.shape
                    final_array = output_array.reshape(-1)
                    print(
                        "Output point_cloud"
                        + str(index.value)
                        + ": "
                        + str(final_array.shape)
                    )
                    copia_points_3d[:] = final_array
                    index.value += 1

                    start_time = time.time()
                    print(i)

                    # print("Pose_2",pose_array_final,len(pose_array_final))

                    print("Final", final.value)
                    '''stop_event.set()
                    self.dron.do_stop_event(stop_event)
                    thread.join()

                    n_points = (max_points_quality-len(total_points))/len(points_3d)

                    seq = self.dron.sequence_point(int(n_points),0.6,0.6,point[3])
                    print("Aprox points left:", n_points)'''
                
                i += 1
            # sleep 3 seconds

            imgs = sorted(
            [file for file in os.listdir(dir) if file.lower().endswith(".jpg")]
            )
            ac_time = time.time()

            if (ac_time - start_time) > 10:
                final.value = True
                with open("pose_array.pickle", "wb") as file:
                    pickle.dump(pose_array_final, file)

                with open("final_point_3d.txt","w") as file:
                    for item in output_array:
                        file.write(str(item) + '\n')

                with open("transform_matrix_1_array.pickle", "wb") as file:
                    pickle.dump(transform_matrix_1_array, file)

                with open("tran_matrix_final.pickle", "wb") as file:
                    pickle.dump(tran_matrix_final, file)
                print("Acaba")
                break

        # while
        '''self.dron.do_land()'''
        #p.join()
        print("Comença el mesh")
        self.to_ply(self.img_obj.path, total_points, total_colors)
        np.savetxt(
            self.img_obj.path + "/res/" + self.img_dir + "_pose_array.csv",
            pose_array,
            delimiter="\n",
        )
        # self.mesh_representation(self.img_dir)
        print("Acaba el mesh")


if __name__ == "__main__":
    sfm = Sfm("GustavIIAdolf")
    sfm()
    print("Final")
