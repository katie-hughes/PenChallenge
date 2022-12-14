from __future__ import print_function
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
import cv2
import pyrealsense2 as rs
import argparse
from move import MoveIt
import time



class processing: 
    def __init__(self, cal=False): 
        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        self.config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
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

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        cfg = self.pipeline.start(self.config)

        profile = cfg.get_stream(rs.stream.color)
        self.intr = profile.as_video_stream_profile().get_intrinsics()
        print("INTRINSICS")
        print(self.intr)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = cfg.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 1 #1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.purple_lo = np.uint8([[[124, 0, 0]]])
        self.purple_hi = np.uint8([[[180, 255, 255]]])

        self.mask = None

        self.title = 'Align Example'

        cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)

        cv2.createTrackbar('HueLo', self.title , self.purple_lo[0][0][0], 180, self.hue_lo_trackbar)
        cv2.createTrackbar('HueHi', self.title , self.purple_hi[0][0][0], 180, self.hue_hi_trackbar)
        cv2.createTrackbar('SatLo', self.title , self.purple_lo[0][0][1], 255, self.sat_lo_trackbar)
        cv2.createTrackbar('SatHi', self.title , self.purple_hi[0][0][1], 255, self.sat_hi_trackbar)
        cv2.createTrackbar('ValLo', self.title , self.purple_lo[0][0][2], 255, self.val_lo_trackbar)
        cv2.createTrackbar('ValHi', self.title , self.purple_hi[0][0][2], 255, self.val_hi_trackbar)

        # set up mover class
        print("Importing the mover class")
        self.mover = MoveIt()
        self.mover.home()
        self.mover.open()
        self.calibration_run = False
        self.calibration_pts = 500
        if cal: 
            self.calibration_run = True
        if self.calibration_run: 
            print("Calibration run!")
            print("Prepare for grippers closing")
            time.sleep(3)
            self.mover.close()
        else:    
            # calibrate where the arm is (this is deprojected coords of pen held in home)
            # calculated for you if you choose to do a calibration run (below)!
            f = open('cal.txt', 'r')
            start = []
            for l in f.readlines(): 
                start.append(float(l))
            print(f"Pen Location from Calibration Run: {start}")
            time.sleep(1)
            self.mover.calibrate(start)
            time.sleep(1)
            self.mover.spin_pos()
        # factor for my proportional control which I am not using atm
        self.alpha = 0.5
        self.n_consecutive = 25

    def hue_lo_trackbar(self,val): 
        self.purple_lo[0][0][0] = val

    def hue_hi_trackbar(self,val): 
        self.purple_hi[0][0][0] = val

    def sat_lo_trackbar(self,val): 
        self.purple_lo[0][0][1] = val

    def sat_hi_trackbar(self,val): 
        self.purple_hi[0][0][1] = val

    def val_lo_trackbar(self,val): 
        self.purple_lo[0][0][2] = val

    def val_hi_trackbar(self,val): 
        self.purple_hi[0][0][2] = val
    
    def go(self): 
        # Streaming loop
        try:
            last_theta = 0
            errors = []
            zs = []
            ct = 0
            coords = []
            leftovers = 0
            while True: ##ct < 3:
                ct += 1
                # Get frameset of color and depth
                frames = self.pipeline.wait_for_frames()
                # frames.get_depth_frame() is a 640x360 depth image

                # Align the depth frame to color frame
                aligned_frames = self.align.process(frames)

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
                bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
                w = bg_removed.shape[0]
                h = bg_removed.shape[1]

                # Convert the RGB colors to HSV colors
                # Hue ranges 0 to 180
                hsv_bg = cv2.cvtColor(bg_removed, cv2.COLOR_RGB2HSV)
                hsv_bg = np.array(hsv_bg)



                purple_mask = cv2.inRange(hsv_bg, self.purple_lo, self.purple_hi)
                #print('Is there purple:', np.any(purple_mask))
                masked_background = cv2.bitwise_and(hsv_bg, hsv_bg, mask=purple_mask)

                contours, hierarchy = cv2.findContours(purple_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                drawn_contours = cv2.drawContours(color_image, contours, -1, (0,255,0), 3)
                centroids = []
                areas = []
                for c in contours: 
                    M = cv2.moments(c)
                    area = cv2.contourArea(c)
                    try: 
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        centroid = (cx,cy)
                        centroids.append(centroid)
                        areas.append(area)
                    except: 
                        pass
                if len(areas) != 0: 
                    print()
                    largest_index = np.argmax(areas)
                    max_centroid = centroids[largest_index]
                    print(f"estimated center: {max_centroid}")
                    centroid_depth = depth_image[max_centroid[1]][max_centroid[0]]
                    print(f"depth: {centroid_depth}")
                    if centroid_depth == 0: 
                        continue
                    point = rs.rs2_deproject_pixel_to_point(self.intr, [max_centroid[0], max_centroid[1]], self.depth_scale*centroid_depth)
                    print(f"deprojected: {point}")
                    rx, ry, rz, theta, rad = self.mover.convert(point)
                    print(f"THETA: {theta}")
                    print(f"RAD: {rad}") 
                    if self.calibration_run: 
                        coords.append(point)
                        if ct > self.calibration_pts: 
                            break
                    else: 
                        error = last_theta - theta
                        last_theta = theta 
                        if np.absolute(error) < 0.05: 
                            leftovers += error
                            # Orig i just did pass here
                            # but i think this is an issue sometimes. 
                        elif np.absolute(error) > 1.0: 
                            ## sometimes it swings back and forth and I dont want it to
                            pass
                        else: 
                            print("Twisting")
                            self.mover.twist(theta)
                        errors = [error] + errors
                        zs = [rz] + zs
                        if len(errors) > self.n_consecutive: 
                            errors = errors[:self.n_consecutive]
                            zs = zs[:self.n_consecutive]
                            print(f"ERROR: {error}")
                            if np.all(np.array(errors) < 0.01): 
                                print("Grabbing the pen!")
                                print(f"LEFTOVERS: {leftovers}")
                                if np.absolute(leftovers) > 0.1: 
                                    print("TWISTING AGAIN")
                                    self.mover.twist(leftovers)
                                leftovers = 0
                                avg_z = np.mean(zs)
                                success = self.mover.setpose(rad,avg_z)
                                if success: 
                                    self.mover.close()
                                    time.sleep(1)
                                    self.mover.zzz()
                                    self.mover.open()
                                    self.mover.spin_pos()
                                errors = []
                                zs = []
                                last_theta = 0
                    drawn_contours = cv2.circle(drawn_contours, max_centroid, 5, [0,0,255], 5)
                

                # Render images:
                #   depth align to color on left
                #   depth on right
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                #images = np.hstack((bg_removed, depth_colormap, masked_background))

                cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
                cv2.imshow(self.title, drawn_contours)
                #cv2.imshow(self.title, purple_mask)
                key = cv2.waitKey(1)
                # Press esc or 'q' to close the image window
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
            if self.calibration_run: 
                coords = np.array(coords)
                mean = np.mean(coords, axis=0)
                print(f"TO USE FOR CALIBRATION: {list(mean)}\n\n")
                f = open('cal.txt', "w")
                for c in mean: 
                    f.write(str(c)+'\n')
                f.close()
                self.mover.open()
        finally:
            self.pipeline.stop()



parser = argparse.ArgumentParser(description='Grab the pen')
parser.add_argument('-c', '--calibration', action='store_true', help='Do a calibration run')


args = parser.parse_args()

if args.calibration: 
    a = processing(cal=True)
else: 
    a = processing()

a.go()