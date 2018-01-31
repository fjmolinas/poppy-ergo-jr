import numpy as np
import time

from pypot.primitive import Primitive
from .postures import BasePostureGripper, SafePowerUp
from .move import MoveToPosition

class TagHold(Primitive):
    
    def __init__(self, robot, detector, marker_id, marker_position_id,time = 2):
        Primitive.__init__(self, robot)
        
        self.target_position = marker_position_id
        
        def get_marker_position(id=marker_id):
            marker = [m.position for m in getattr(detector, 'markers') if m.id == id]

            if len(marker)>0:
                return marker[0]
            return None
        
        def move_to_marker(marker):
            rvecs,tvecs,objects = marker[0:3]
            position = tvecs[0][0]
            position = (position[0],-1.*(position[2]*np.sin(self.angle)+position[1]*np.sin(np.pi/2 - self.angle)),position[2]*np.cos(self.angle)-position[1]*np.cos(np.pi/2-self.angle)-0.03)
            move = MoveToPosition(self.robot,position,3)
            move.start()
            move.wait_to_stop()
         
        self.move_to_marker = move_to_marker
        self.get_marker_position = get_marker_position

    def setup(self):
        """
            TODO: Write documentation
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
        
    

    def run(self):
        """
            Search in robot camera all the aruco markers and take the first to follow.
        """
        while self.get_marker_position() is None:
            continue
            
        self.move_to_marker(self.get_marker_position())
        
        for m in self.robot.motors:
            m.led = 'green'
        
        gripper = self.robot.m6
        gripper.goal_position = 50
        time.sleep(1)
        while gripper.present_load < 15 and gripper.present_position > -20:
            gripper.goal_position =gripper.present_position - 2
            time.sleep(0.1)
            print(gripper.goal_position, gripper.present_load)
            
        base_prim = BasePostureGripper(self.robot,3)
        base_prim.start()
        base_prim.wait_to_stop()
        
        while self.get_marker_position(self.target_position) is None:
            continue
        
        self.move_to_marker(self.get_marker_position(self.target_position))
       
        while gripper.present_position < 20:
            gripper.goal_position = gripper.present_position + 2
            time.sleep(0.1)
        

    def teardown(self):
        safe_prim = SafePowerUp(self.robot)
        safe_prim.start()
        safe_prim.wait_to_stop()
        for m in self.robot.motors:
            m.led = 'off'
            m.compliant = True
