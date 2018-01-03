from cv2 import aruco
import numpy as np
import json
import os


from pypot.primitive import LoopPrimitive

class TagFollower(LoopPrimitive):
    def __init__(self, robot):
        LoopPrimitive.__init__(self, robot, 1.)

        def camera_parameters():
            base_path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir))
            data_path = os.path.join(os.path.join(base_path, 'configuration'), 'camera_calibration.json')
            data = json.load(open(data_path))
            matrix = np.array([np.array(xi) for xi in data['mtx']])
            return matrix, np.array(data['dist'])

        self.camera_parameters = camera_parameters
    def setup(self):
        """Motor setup"""
        for m in self.robot.motors:
            m.compliant = False

        init_pos = dict([(m.name, 0.0) for m in self.robot.motors])
        self.robot.goto_position(init_pos, 3., wait=True)

        for m in self.robot.motors:
            m.moving_speed = 70.

        for m in self.robot.motors:
            m.led = 'green'

        """Camera & aruco setup"""

        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
        self.matrix,self.distribution = self.camera_parameters()
        self.M = np.eye(4)
        self.kwargs = {}


    def update(self):
        frame = self.robot.camera.frame
        marker = aruco.detectMarkers(frame,self.dictionary)
        if marker[0]:
            estimated_pose = aruco.estimatePoseSingleMarkers(marker[0],0.027,self.matrix,self.distribution)
            position = estimated_pose[1][0][0]
            self.M[:3,3] = position
            inverse = np.round(self.robot.chain.inverse_kinematics(self.M, initial_position=self.robot.chain.convert_to_ik_angles(self.robot.chain.joints_position),**self.kwargs),3)
            inverse_ik = self.robot.chain.convert_from_ik_angles(inverse)
            for i in range(len(self.robot.motors)):
                self.robot.motors[i].goto_position(inverse_ik[i],1)


    def teardown(self):

        for m in self.robot.motors:
            m.led = 'off'
	    m.compliant = True
