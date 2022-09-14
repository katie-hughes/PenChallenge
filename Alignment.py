from __future__ import print_function
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import cv2
import pyrealsense2 as rs
import argparse



# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 1 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)


purple_rgb = [13.7, 9.8, 25.9]
print('PURPLE RGB:', purple_rgb)
my_purple_hsv = cv2.cvtColor(np.uint8([[purple_rgb]]), cv2.COLOR_RGB2HSV)
print("PURPLE HSV: ", my_purple_hsv)
purple_hue = my_purple_hsv[0][0][0]
purple_sat = my_purple_hsv[0][0][1]
purple_val = my_purple_hsv[0][0][2]
print("PURPLE HUE", purple_hue)
hue_buff = 50
sat_buff = 50
val_buff = 50
purple_low = np.array([purple_hue-hue_buff, purple_sat-sat_buff, purple_val-val_buff])
purple_high = np.array([purple_hue+hue_buff, purple_sat+sat_buff, purple_val+val_buff])

def hue_trackbar(val): 
    print("hello")

cv2.createTrackbar('Hue', 'HueTitle' , 0, 180, hue_trackbar)


# Streaming loop
try:
    ct = 0
    while True: #ct < 5:
        ct += 1
        # Get frameset of color and depth
        frames = pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        w = bg_removed.shape[0]
        h = bg_removed.shape[1]

        # Convert the RGB colors to HSV colors
        # Hue ranges 0 to 180
        hsv_bg = cv2.cvtColor(bg_removed, cv2.COLOR_RGB2HSV)
        hsv_bg = np.array(hsv_bg)



        purple_mask = cv2.inRange(hsv_bg, purple_low, purple_high)
        print('Is there purple:', np.any(purple_mask))
        masked_background = cv2.bitwise_and(hsv_bg, hsv_bg, mask=purple_mask)

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap, masked_background))

        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()