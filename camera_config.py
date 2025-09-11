import numpy as np
import cv2 as cv
import glob

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Set the chessboard size correctly: number of inner corners per a chessboard row and column
chessboard_size = (8, 5)  # width, height (number of inner corners)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane

images = glob.glob('/home/pi5/Turcja2025-dron/camera calibration/captures/*.jpg')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        print('Found corners in', fname)

        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        objpoints.append(objp.copy())  # Use copy to avoid reference issues

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboard_size, corners2, ret)
        # Optional: display image
        # cv.imshow('Corners', img)
        # cv.waitKey(500)
    print('Processed', fname)

# Calibrate camera
if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Camera matrix:\n", mtx)
    print("Distortion coefficients:\n", dist)
else:
    print("No corners found in any image.")

cv.destroyAllWindows()
