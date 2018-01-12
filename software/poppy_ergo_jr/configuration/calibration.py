import numpy as np
import cv2
import glob
import json

class CameraCalibration(object):

    def __init__(self, abs_path, chess_shape, cell_size):
        self.shape = chess_shape
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, cell_size, 0.001)
        self.images = glob.glob(abs_path)
        self.objp = np.zeros((chess_shape[0]*chess_shape[1],3), np.float32)
        self.objp[:,:2] = np.mgrid[0:chess_shape[0],0:chess_shape[1]].T.reshape(-1,2)

    def calibrate(self, path_output_file='result.json'):
        objpoints = []
        imgpoints = []
        gray = None
        for fname in self.images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (self.shape[0], self.shape[1]), None)

            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
                imgpoints.append(corners2)
        ret, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        results = open(path_output_file,'w')
        file.write('ret: '+ str(ret) + ',mtx: '+str(mtx),+',dist: '+str(dist)+', rvecs: '+str(rvecs)+',tvecs: '+str(tvecs)) 
