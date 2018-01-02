from cv2 import aruco
import numpy as np
import json

from pypot.primitive import LoopPrimitive

class TagFollower(LoopPrimitive):
    def __init__(self, robot):
        LoopPrimitive.__init__(self, robot, 1.)

        def camera_parameters():
            data = json.load(open('../configuration/camera_calibration.json'))
            return data['mtx'], data['dist']

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
        frame = poppy.camera.frame
        marker = aruco.detectMarkers(frame,dictionary)
        if markers[0]>0:
            estimated_pose = aruco.estimatePoseSingleMarkers(marker[0],0.027,self.matrix,self.distribution)
            position = estimated_pose[1][0][0]
            M[:3,3] = position
            inverse = np.round(popy.chain.inverse_kinematics(M, initial_position=poppy.chain.convert_to_ik_angles(poppy.chain.joints_position),**kwargs),3)
            inverse_ik = poppy.chain.convert_from_ik_angles(inverse)
            for i in range(len(robot.motors)-1):
                robot.motors[i].goto_position(inverse_ik[i],1)


    def teardown(self):
        [s.stop() for s in self.sinus]

        for m in self.robot.motors:
            m.led = 'off'
