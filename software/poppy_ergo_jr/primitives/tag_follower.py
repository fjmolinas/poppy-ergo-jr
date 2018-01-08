import cv2
import json
import numpy as np
import os
import logger
import time


from pypot.primitive import LoopPrimitive
from .postures import IdlePosture

logger = logging.getLogger(__name__)


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

        idle_prim = IdlePosture(self.robot, 3.0)
        idle_prim.start()
        idle_prim.wait_to_stop()

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
        init_time = time.clock()
        frame = self.robot.camera.frame
        logger.info("time spent in get camera frame: {}".format((time.clock()-init_time)/1000.))
        marker = cv2.aruco.detectMarkers(frame,self.dictionary)
        logger.info("time spent in detect marker: {}".format((time.clock()-init_time)/1000.))
        if marker[0]:
            rvecs,tvecs,objects = cv2.aruco.estimatePoseSingleMarkers(marker[0],self.marker_lenght,self.camera_matrix,self.distrib)
            logger.info("time spent in estimate pose: {}".format((time.clock()-init_time)/1000.))
            position = tvecs[0][0]
            self.M[:3,3] = (position[0],-1.*position[2]*np.sin(self.angle)-position[1]*np.sin(np.pi/2 - self.angle),position[2]*np.cos(self.angle))
            inverse = np.round(self.robot.chain.inverse_kinematics(self.M, initial_position=self.robot.chain.convert_to_ik_angles(self.robot.chain.joints_position), **self.kwargs),3)
            logger.info("time spent in get ik: {}".format((time.clock()-init_time)/1000.))
            inverse_ik = self.robot.chain.convert_from_ik_angles(inverse)
            for i in range(len(self.robot.motors)):
                self.robot.motors[i].goto_position(inverse_ik[i],1)
            logger.info("time spent in move to position: {}".format((time.clock()-init_time)/1000.))

    def teardown(self):
        idle_prim = IdlePosture(self.robot, 3.0)
        idle_prim.start()
        idle_prim.wait_to_stop()
        for m in self.robot.motors:
            m.led = 'off'
            m.compliant = True
