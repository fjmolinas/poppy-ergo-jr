import cv2
import json
import numpy as np
import os


from pypot.primitive import LoopPrimitive

class TagFollower(LoopPrimitive):
    def __init__(self, robot):
        LoopPrimitive.__init__(self, robot, 1.)

        def camera_parameters():
            base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir))
            data_path = os.path.join(os.path.join(base_path, 'configuration'), 'camera_calibration.json')
            data = json.load(open(data_path))
            return np.float64(data['mtx']), np.float64(data['dist'])

        self.camera_parameters = camera_parameters
    def setup(self):
        """
            First part is motor setup and then camera & cv2.aruco setup:
                * dictionary define the aruco dictionary that you use.
                * camera_matrix and distrib define the intrinsec and distribution camera calibration.
                * angle define the angle of camera in ergo_jr base
                * marker_lenght define the marker size in meters
        """
        for m in self.robot.motors:
            m.compliant = False

        init_pos = dict([(m.name, 0.0) for m in self.robot.motors])
        self.robot.goto_position(init_pos, 3., wait=True)

        for m in self.robot.motors:
            m.moving_speed = 70.

        for m in self.robot.motors:
            m.led = 'green'

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.camera_matrix,self.distrib = self.camera_parameters()
        self.angle = np.pi/3
        self.marker_lenght = 0.027
        self.M = np.eye(4)
        self.kwargs = {}


    def update(self):
        """
            Search in robot camera all the aruco markers and take the first to follow.
        """
        frame = self.robot.camera.frame
        marker = cv2.aruco.detectMarkers(frame,self.dictionary)
        if marker[0]:
            rvecs,tvecs,objects = cv2.aruco.estimatePoseSingleMarkers(marker[0],self.marker_lenght,self.camera_matrix,self.distrib)
            position = tvecs[0][0]
            self.M[:3,3] = (position[0],-1.*position[2]*np.sin(self.angle)-position[1]*np.sin(np.pi/2 - self.angle),position[2]*np.cos(self.angle))
            inverse = np.round(self.robot.chain.inverse_kinematics(self.M, initial_position=self.robot.chain.convert_to_ik_angles(self.robot.chain.joints_position), **self.kwargs),3)
            inverse_ik = self.robot.chain.convert_from_ik_angles(inverse)
            for i in range(len(self.robot.motors)):
                self.robot.motors[i].goto_position(inverse_ik[i],1)
    def teardown(self):

        for m in self.robot.motors:
            m.led = 'off'
            m.compliant = True
