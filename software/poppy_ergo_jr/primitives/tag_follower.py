import cv2
import json
import numpy as np
import os
import logging
import time


from pypot.primitive import LoopPrimitive
from .postures import SafePowerUp

logger = logging.getLogger(__name__)


class TagFollower(LoopPrimitive):
    def __init__(self, robot, detector, marker_id):
        LoopPrimitive.__init__(self, robot, 1.)

        def get_marker_position():
            marker = [m.position for m in getattr(detector, 'markers') if m.id == marker_id]

            if len(marker)>0:
                return marker[0]
            return None
        self.get_marker_position = get_marker_position
    
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

        safe_prim = SafePowerUp(self.robot)
        safe_prim.start()
        safe_prim.wait_to_stop()

        for m in self.robot.motors:
            m.moving_speed = 70.

        for m in self.robot.motors:
            m.led = 'red'
        self.angle = np.pi/3
        self.M = np.eye(4)
        self.kwargs ={}

    def update(self):
        """
            Search in robot camera all the aruco markers and take the first to follow.
        """
        init_time = time.clock()
        marker = self.get_marker_position()

        logger.info("time spent in detect marker: {}".format((time.clock()-init_time)/1000.))
        if marker is not None:
            rvecs,tvecs,objects = marker[0:3]
            position = tvecs[0][0]
            self.M[:3,3] = (position[0],-1.*position[2]*np.sin(self.angle)-position[1]*np.sin(np.pi/2 - self.angle),position[2]*np.cos(self.angle))
            inverse = np.round(self.robot.chain.inverse_kinematics(self.M, initial_position=self.robot.chain.convert_to_ik_angles(self.robot.chain.joints_position), **self.kwargs),3)
            logger.info("time spent in get ik: {}".format((time.clock()-init_time)/1000.))
            inverse_ik = self.robot.chain.convert_from_ik_angles(inverse)
            for i in range(len(self.robot.motors)):
                self.robot.motors[i].led = 'blue'
                self.robot.motors[i].goto_position(inverse_ik[i],1)
            logger.info("time spent in move to position: {}".format((time.clock()-init_time)/1000.))
        else:
            for m in self.robot.motors:
                m.led = 'red'

    def teardown(self):
        safe_prim = SafePowerUp(self.robot)
        safe_prim.start()
        safe_prim.wait_to_stop()
        for m in self.robot.motors:
            m.led = 'off'
            m.compliant = True
