from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import modern_robotics as mr
import time
import math

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
            self.mode = input("[h]ome, [s]leep, [q]uit, [o]pen, [c]lose, [t]wist, [e]nd effector, [p]os, sho[u]lder, [j]oints, [w]rist: ")
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
            elif self.mode == 'u':  
                theta = input("Enter final degree position: ")
                theta_rad = (np.pi/180.)* float(theta)
                self.robot.arm.set_single_joint_position("shoulder", theta_rad)
            elif self.mode == 'w':  
                theta = input("Enter final degree position: ")
                theta_rad = (np.pi/180.)* float(theta)
                self.robot.arm.set_single_joint_position("wrist_angle", theta_rad)
            elif self.mode == 'j': 
                joints = self.robot.arm.get_joint_commands()
                print(joints)
    def get_current_pos(self): 
        joints = self.robot.arm.get_joint_commands()
        T = mr.FKinSpace(self.robot.arm.robot_des.M, self.robot.arm.robot_des.Slist, joints)
        [R, p] = mr.TransToRp(T) # get the rotation matrix and the displacement
        return p
    def calibrate(self, pos): 
        print(f"my pos: {pos}")
        robot_pos = self.get_current_pos()
        print(f"robot pos (m): {robot_pos}")
        cx = pos[0]
        rx = robot_pos[0]
        cz = pos[2]  # this one will always be positive. 
        ry = robot_pos[1]
        # distances betewen center of camera to center of robot
        # THis assumes robot is to the right of the camera
        self.dist_x = rx - cx
        self.dist_y = cz - ry 
        print(f"DIST X: {self.dist_x}")
        print(f"DIST Y: {self.dist_y}")
        ## now when I get a distance in the camera frame
        ## how to convert it to robot frame? 
        ## basically I want something to add to the x and y coordinates
        ## to convert it.
    def convert(self, pos): 
        cx = pos[0]
        cz = pos[2]
        rx = self.dist_x + cx
        ry = cz - self.dist_y
        #print("rx, ry", rx, ry)
        theta = np.arctan(ry / rx)
        rad = math.sqrt(rx**2 + ry**2)
        print("theta", theta)
        return rx, ry, theta, rad
    def twist(self, theta): 
        self.robot.arm.set_single_joint_position("waist", theta)
    def zzz(self):
        self.robot.arm.go_to_sleep_pose()
    def home(self):
        self.robot.arm.go_to_home_pose()
    def setpose(self, pos): 
        current = self.get_current_pos()
        current_r = math.sqrt((current[0]**2)+(current[1]**2))
        print(pos, current_r, pos-current_r)
        self.robot.arm.set_ee_cartesian_trajectory(x=(pos - current_r + 0.02))
    def close(self): 
        self.robot.gripper.grasp()
    def open(self): 
        self.robot.gripper.release()
    def spin_pos(self): 
        self.robot.arm.set_joint_positions([0.0, -0.5*math.pi, 0.5*math.pi, 0.0])








#m = MoveIt()
#m.interactive()


"""
centroid_pos = [63.33274459838867, -6.052464008331299, 250.0]
depth_scale = 0.001
centroid_pos = [i*depth_scale for i in centroid_pos]
m.calibrate(centroid_pos)
#time.sleep(5)

print("CONVERT")
m.convert(centroid_pos)

"""
print("\n\n\n\n")
