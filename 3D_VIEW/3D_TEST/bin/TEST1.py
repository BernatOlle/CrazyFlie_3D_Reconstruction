import cv2 as cv
import os
import numpy as np
import glob
from scipy.optimize import least_squares
import plotly.graph_objects as go


def downscale():
    
    K[0, 0] /= factor
    K[1, 1] /= factor
    K[0, 2] /= factor
    K[1, 2] /= factor
    
    return K
    
def downscale_image(image):
    for _ in range(1,int(factor / 2) + 1):
        image = cv.pyrDown(image)
    return image


def reprojection_loss_function(opt_variables, points_2d, num_pts):
    P = opt_variables[0:12].reshape(3,4)
    point_3d = opt_variables[12:].reshape((num_pts,4))
    rep_error = []
    
    for idx, pt_3d in enumerate(point_3d):
        pt_2d=np.array([points_2d[0][idx], points_2d[1][idx]])
        
        reprojected_pt = np.matmul(P,pt_3d)
        reprojected_pt /= reprojected_pt[2]
        rep_error.append(pt_2d-reprojected_pt[0:2])
        
        return np.array(rep_error).ravel()

def bundle_adjustment(points_3d, points_2d, img, projection_matrix):
    
    opt_variables = np.hstack((projection_matrix.ravel(), points_3d.ravel(order="F")))
    num_points = len(points_2d[0])
    
    corrected_values = least_squares(reprojection_loss_function, opt_variables, args=(points_2d,num_points))
    
    #print("Corrected values:" + str(corrected_values))
    P=corrected_values.x[0:12].reshape((3, 4))
    points_3d = corrected_values.x[12:].reshape((num_points,4))
    
    return P, points_3d

factor=2.0
image_list= glob.glob("*.JPG")
print (image_list)
image1 = cv.imread(image_list[0])
img1 = downscale_image(image1)
img2 = downscale_image(cv.imread(image_list[1]))

K= np.array([[573.45294189,   0,        281.68469945],
 [  0,         574.84844971, 351.73494837],
 [  0,           0,           1        ]])

K= np.array(
    [[2393.952166119461,0,932.3821770809047],
     [0,2398.118540286656,628.2649953288065],
     [0,0,1]])
    



#Variables
prev_img = None
prev_kp = None
prev_desc = None

#Matrx
downscale()
print("K matrix:"+str(K)+ "\n")

transform_matrix_0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])
transform_matrix_1 = np.empty((3,4))
pose_0 = np.matmul(K,transform_matrix_0)
pose_1 = np.empty((3,4))

X=np.array([])
Y=np.array([])
Z=np.array([])

    
for iter, filename in enumerate(image_list):
    print(filename+"\n")
    img = downscale_image(cv.imread(filename,0))
    

    sift     = cv.SIFT_create()
    kp, desc = sift.detectAndCompute(img,None)
    
    if iter == 0:
        prev_img = img
        prev_kp = kp
        prev_desc = desc
        #print("1\n")
    else:
        #print("2\n")
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 100)
        flann = cv.FlannBasedMatcher(index_params,search_params)
        matches = flann.knnMatch(prev_desc,desc,k=2)
        good = []
        pts1 = []
        pts2 = []
        
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.70 * n.distance:
                good.append(m)
                pts1.append(prev_kp[m.queryIdx].pt)
                pts2.append(kp[m.trainIdx].pt)
        
        pts1 = np.array(pts1)
        pts2 = np.array(pts2)
        
        F, mask = cv.findFundamentalMat(pts1, pts2 ,cv.FM_RANSAC)
        print("The fundamental matrix:\n" + str(F) + "\n")
        
        #*We select only inlier points
        #pts1 = pts1[mask.ravel()==1]
        #pts2 = pts2[mask.ravel()==1]
        
        #Calculate the essential matrix
        essential_matrix, em_mask = cv.findEssentialMat(pts1, pts2, K, method=cv.RANSAC, prob=0.999, threshold=0.4, mask=None)
        pts1 = pts1[em_mask.ravel() == 1]
        pts2 = pts2[em_mask.ravel() == 1]
        
        E=K.T.dot(F.dot(K))
        print("The new essential matrix:\n" + str(essential_matrix))
        
        retval, R, t, em_mask = cv.recoverPose(essential_matrix, pts1, pts2, K)
        pts1 = pts1[em_mask.ravel() > 0]
        pts2 = pts2[em_mask.ravel() > 0]
        
        transform_matrix_1[:3,:3] = np.matmul(R, transform_matrix_0[:3,:3])
        transform_matrix_1[:3,3] = transform_matrix_0[:3, 3] + np.matmul(transform_matrix_0[:3,:3], t.ravel())
        
        #print("The R_t_0 \n" + str(R_t_0) +"\n")
        #print("The R_t_1 \n" + str(R_t_1) +"\n")
        
        pose_1 = np.matmul(K, transform_matrix_1)
        
        #print("The projection matrix 1: \n" + str(P1) +"\n")
        #print("The projection matrix 2: \n" + str(P2) +"\n")
        
        pts1 = np.transpose(pts1)
        pts2 = np.transpose(pts2)
        
       # print("Shape pts1:\n" +str(pts1.shape))
        
        
        points_3d = cv.triangulatePoints(pose_0, pose_1, pts1, pts2)
        points_3d /= points_3d[3]
        
        pose_1, points_3d = bundle_adjustment(points_3d, pts2, img, pose_1)
        #print(points_3d)
        opt_variables = np.hstack((pose_1.ravel(), points_3d.ravel(order="F")))
        num_points = len(pts2[0])
        
        X = np.concatenate((X, points_3d[0]))
        Y= np.concatenate((Y, points_3d[1]))
        Z = np.concatenate((Z, points_3d[2]))
        
        #update references
        
        transform_matrix_0 = np.copy(transform_matrix_1)
        pose_0 = np.copy(pose_1)
        prev_img = img
        prev_kp = kp
        prev_desc = desc
        
indexs = np.abs(Z)>6.5
#print(Z)
Z2 = np.delete(Z, indexs)
X2 = np.delete(X, indexs)
Y2 = np.delete(Y, indexs)

fig = go.Figure(data=[go.Mesh3d(x=X2, y=Y2, z=Z2, color='red', opacity= 0.8)])
fig.update_layout(autosize=False, width=900, height = 900)
fig.show()

        