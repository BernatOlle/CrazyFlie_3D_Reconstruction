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

# from take_photo import Take
matplotlib.use("TkAgg")
# import pymeshlab


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
        with open("tran_matrix_final.pickle", "rb") as file:
            self.tran_matrix_final = pickle.load(file)

        self.img_obj = Image_loader(img_dir, downscale_factor)
        self.img_dir = img_dir
        print("DIR" + self.img_dir)
        # self.camera = Network()
        # self.cam = Take(img_dir)

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

    def to_ply(self, path, point_cloud, colors) -> None:
        """
        Generates the .ply which can be used to open the point cloud
        """
        out_points = (
            point_cloud.reshape(-1, 3) * 200
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

    def __call__(self, enable_bundle_adjustment: boolean = False):
        dir = "images/" + self.img_dir
        transform_matrix_1 = np.empty((3, 4))
        pose_array = self.img_obj.K.ravel()
        transform_matrix_1_array = {}
        pose_0 = np.empty((3, 4))
        pose_1 = np.empty((3, 4))
        total_points = np.zeros((1, 3))
        total_colors = np.zeros((1, 3))
        imgs = sorted(
            [file for file in os.listdir(dir) if file.lower().endswith(".jpg")]
        )
        print(imgs, len(imgs))
        i = 0
        self.vect = {}
        start_time = time.time()

        while True:
            print("---------------------------------")
            if len(imgs) > i:
                print("Inici", i)
                file_0 = imgs[i]
                print(file_0)
                if file_0.lower().endswith(".jpg"):

                    file_path = os.path.join(dir, file_0)
                    img0 = cv2.imread(file_path)
                    image_1 = self.remove_bg(img0)
                    image_1 = self.downscale_image(image_1)

                    if i == 0:
                        image_0 = np.copy(image_1)
                        file_1 = file_0
                        i += 1
                        continue

                    # print("Agafo totes les imatges")
                    features_cur, features_2 = self.find_features(
                        image_0, image_1, file_0, file_1
                    )

                    tran_matrix = self.tran_matrix_final[i][0]
                    rot_matrix = self.tran_matrix_final[i][1]

                    transform_matrix_1 = np.hstack(
                        (rot_matrix, tran_matrix)
                    )  # depen de K

                    transform_matrix_1_array[i] = transform_matrix_1  # depen de K
                    pose_1 = np.matmul(self.img_obj.K, transform_matrix_1)  # depen de K

                    if i > 1:
                        _, _, cm_mask_0, cm_mask_1 = self.common_points(
                            feature_1, features_cur, features_2
                        )

                        cm_mask_0, cm_mask_1, points_3d = self.triangulation(
                            pose_0, pose_1, cm_mask_0, cm_mask_1
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
                        pose_array = np.hstack((pose_array, pose_1.ravel()))
                        if len(points_3d) < 200:
                            cm_mask_1, points_3d = self.recal(imgs, image_1, file_0)

                        color_vector = []

                        total_points = np.vstack((total_points, points_3d[:, 0, :]))
                        # print("hola")
                        print("Total points", total_points.shape)

                        points_left = np.array(cm_mask_1, dtype=np.int32)

                        color_vector = np.array(
                            [image_1[l[1], l[0]] for l in points_left.T]
                        )

                        # print("Fin")
                        total_colors = np.vstack((total_colors, color_vector))
                        print("Total colors", total_colors.shape)

                        self.vect[i] = [pose_0, pose_1, feature_0, feature_1]
                        output_array = total_points.reshape(
                            -1, 3
                        )  # * 200  # Serveix per fer la imatge mes gran

                        # si la length de la copia_points_3d es mes petita que la de output array que s'afegeixin 0s
                        start_time = time.time()

                        # print("Pose_2",pose_array_final,len(pose_array_final))
                    file_1 = file_0
                    pose_0 = np.copy(pose_1)
                    image_0 = np.copy(image_1)
                    feature_0 = np.copy(features_cur)
                    feature_1 = np.copy(features_2)
                i += 1
            # sleep 3 seconds

            imgs = sorted(
                [file for file in os.listdir(dir) if file.lower().endswith(".jpg")]
            )
            ac_time = time.time()

            if (ac_time - start_time) > 10:
                with open("final_point_3d.txt", "w") as file:
                    for item in output_array:
                        file.write(str(item) + "\n")

                with open("transform_matrix_1_array.pickle", "wb") as file:
                    pickle.dump(transform_matrix_1_array, file)

                print("Acaba")
                break

        # while
        # p.join()
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
    sfm = Sfm("Test_1")
    sfm()
    print("Final")
