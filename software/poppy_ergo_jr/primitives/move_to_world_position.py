import time

from pypot.primitive import Primitive

from numpy import eye


class MoveToPosition(Primitive):

    def __init__(self,robot,position,time):
        Primitive.__init__(self,robot)
        self.position = position
        self.time = time

    def setup(self):
        self.M = eye(4)
        self.M[:3,3] = position
        inverse = np.round(self.robot.chain.inverse_kinematics(self.M, initial_position=self.robot.chain.convert_to_ik_angles(self.robot.chain.joints_position), **{}),3)
        self.inverse_ik = self.robot.chain.convert_from_ik_angles(inverse)
        for m in self.robot.motors:
            m.compliant = False

    def run(self):
        for i in range(len(self.robot.motors)):
            self.robot.motors[i].goto_position(self.inverse_ik[i],self.time)

    def teardown(self):
        pass
