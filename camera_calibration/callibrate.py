'''
From OpenCV official tutorial: https://docs.opencv.org/4.5.1/dc/dbb/tutorial_py_calibration.html
'''
import numpy as np
import cv2 as cv
import glob
import json

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)

grid_w = 9  # TODO: change
grid_h = 6  # TODO: change

img_dir = "../camera_calibration/cam_new/"  # TODO: change
params_file = "../params/cam_new.json"  # TODO: change

objp = np.zeros((grid_h * grid_w, 3), np.float32)
objp[:, :2] = np.mgrid[0:grid_w, 0:grid_h].T.reshape(-1, 2)
# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob(img_dir + '*.png')
for fname in images:
    print("Processing " + fname + " ......")
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (grid_w, grid_h), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (grid_w, grid_h), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(2000)
cv.destroyAllWindows()

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
data = dict(mtx=mtx.tolist(), dist=dist.tolist())
with open(params_file, 'w') as f:
    str_data = json.dumps(data, indent=2)
    f.write(str_data)
print("The parameters of camera has been saved in " + params_file)
