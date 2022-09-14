from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

class MoveIt:
    def __init__(self): 
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        self.info = self.robot.arm.group_info
        #print(self.info)
        self.mode = 'h'
    def interactive(self): 
        while self.mode != 'q': 
            self.mode = input("[h]ome, [s]leep, [q]uit, [o]pen, [c]lose, [t]wist ")
            if self.mode == "h":
                self.robot.arm.go_to_home_pose()
            elif self.mode == "s":
                self.robot.arm.go_to_sleep_pose()
            elif self.mode == "o":
                # open grippers
                self.robot.gripper.release()
            elif self.mode == "c": 
                #close grippers
                self.robot.gripper.grasp()
            elif self.mode == "t": 
                # twist 
                theta = input("Enter final degree position: ")
                theta_rad = (np.pi/180.)* float(theta)
                self.robot.arm.set_single_joint_position("waist", theta_rad)
    def go(): 
        

m = MoveIt()
m.interactive()