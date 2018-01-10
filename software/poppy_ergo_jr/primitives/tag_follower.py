import cv2
import json
import numpy as np
import os
import logging
import time


from pypot.primitive import LoopPrimitive
from .postures import SafePowerUp
from .move import MoveToPosition

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

    def update(self):
        """
            Search in robot camera all the aruco markers and take the first to follow.
        """
        init_time = time.clock()
        marker = self.get_marker_position()

        if marker is not None:
            rvecs,tvecs,objects = marker[0:3]
            position = tvecs[0][0]
            position = (position[0],-1.*position[2]*np.sin(self.angle)-position[1]*np.sin(np.pi/2 - self.angle),position[2]*np.cos(self.angle))
            move = MoveToPosition(self.robot,position,1)
            move.start()
           

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
