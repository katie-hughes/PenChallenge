from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import modern_robotics as mr
import time

class MoveIt:
    def __init__(self): 
        self.robot = InterbotixManipulatorXS("px100", "arm", "gripper")
        self.info = self.robot.arm.group_info
        #print(self.info)
        self.mode = 'h'

        self.dist_x = 0
        self.dist_y = 0
    def interactive(self): 
        while self.mode != 'q': 
            self.mode = input("[h]ome, [s]leep, [q]uit, [o]pen, [c]lose, [t]wist, [e]nd effector, [p]os: ")
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
            elif self.mode == 'e': 
                matrix = self.robot.arm.get_ee_pose_command()
                print(matrix)
            elif self.mode == 'p': 
                print(self.get_current_pos())
    def get_current_pos(self): 
        joints = self.robot.arm.get_joint_commands()
        T = mr.FKinSpace(self.robot.arm.robot_des.M, self.robot.arm.robot_des.Slist, joints)
        [R, p] = mr.TransToRp(T) # get the rotation matrix and the displacement
        return p
    def calibrate(self, pos): 
        print(f"my pos: {pos}")
        robot_pos = self.get_current_pos()
        print(f"robot pos (m): {robot_pos}")
        xc = pos[0]
        xr = robot_pos[0]
        #this is the distance from the camera to the center of the robot
        if xc < 0: 
            if xr > 0: 
                self.dist_x = xc*-1. + xr
            else: 
                self.dist_x = xc*-1.
        else: 
            if xr > 0: 
                self.dist_x = np.abs(xr - xc)
            else: 
                self.dist_x = xr*-1. + xc
        print(f"X distance bewteen camera and robot:{self.dist_x}")
        cy = pos[2]  # this one will always be positive. 
        ry = robot_pos[1]
        if ry < 0: 
            self.dist_y = cy + ry*-1.
        else: 
            self.dist_y = cy - ry 
        print(f"Y distance bewteen camera and robot:{self.dist_y}")





m = MoveIt()
#m.interactive()

centroid_pos = [63.33274459838867, -6.052464008331299, 250.0]
depth_scale = 0.001
centroid_pos = [i*depth_scale for i in centroid_pos]
m.calibrate(centroid_pos)
#time.sleep(5)

print("\n\n\n\n")