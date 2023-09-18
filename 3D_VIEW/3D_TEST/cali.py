import numpy as np
import cv2 as cv
import glob
import os
import time

name = 'images/Test1/iPhone'
      

    
# termination criteria
CHECKERBOARD = (5,5)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((1,CHECKERBOARD[0]*CHECKERBOARD[1],3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
print(name+"/*.JPG")
images = glob.glob(name+"/*.JPG")
print(images)
i = 0
for fname in images:
    img = cv.imread(fname)
    img = cv.resize(img, dsize=(1296, 1936), interpolation=cv.INTER_CUBIC)
    cv.imshow('img', img)
    cv.waitKey(500)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, CHECKERBOARD, None)
    print(ret)
    # If found, add object points, image points (after refining them)
    if ret == True:
        i += 1
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)
        print(i)

#print(objpoints)
#print(imgpoints)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
h,  w = img.shape[:2]
newcameramtx, roi=cv.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

file = open(name+"/K.txt", "w")

for fila in newcameramtx:
    for element in fila:
        file.write(str(element)+" ")
    file.write("\n")
    
file.close()    
cv.destroyAllWindows()