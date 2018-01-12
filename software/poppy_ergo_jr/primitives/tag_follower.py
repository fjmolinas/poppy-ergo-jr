import numpy as np

from pypot.primitive import LoopPrimitive
from .postures import SafePowerUp,BasePostureGripper
from .move import MoveToPosition

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
            Use BasePostureGripper to get a better range with the camera.
        """
        for m in self.robot.motors:
            m.compliant = False

        base_prim = BasePostureGripper(self.robot,3)
        base_prim.start()
        base_prim.wait_to_stop()

        for m in self.robot.motors:
            m.moving_speed = 70.

        for m in self.robot.motors:
            m.led = 'red'
        self.angle = np.pi/3

    def update(self):
        """
            Search in robot camera the aruco marker if exist, it will be followed.
        """
        marker = self.get_marker_position()

        if marker is not None:
            rvecs,tvecs,objects = marker[0:3]
            position = tvecs[0][0]
            position = (position[0],-1.*(position[2]*np.sin(self.angle)+position[1]*np.sin(np.pi/2 - self.angle)),position[2]*np.cos(self.angle)-position[1]*np.cos(np.pi/2-self.angle)-0.03)
            move = MoveToPosition(self.robot,position)
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
