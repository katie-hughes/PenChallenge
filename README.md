# PEN ASSISTANT
Katie Hughes

This code uses OpenCV and data from an intel Realsense camera to control a PincherX 100 robot. 
When a purple pen is detected by the camera, the robot will turn towards the pen and grab it.

First, ensure that the Realsense and the PincherX 100 are plugged in to your computer and powered on.
To set up the PincherX 100, run 
`ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100`.

To calibrate the system, run
`python3 Alignment.py -c`.
During the calibration run, the grippers will open, and you should place
the purple pen in between them before they close. Because this is a known
gripper position, and we can calculate the pen position in the camera frame based
on color masking, we can now convert objects between the camera frame and the gripper frame.

After a calibration, run
`python3 Alignment.py`.
This will load the most recent calibration which is stored in cal.txt.
After this, the gripper should turn towards the pen, and waits to see if the pen moves. 
When the pen consistently doesn't move from this position, 
the gripper will reach out and grasp the pen.