#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from camera_matching.msg import OpticalFlowMsg
from cv_bridge import CvBridgeError, CvBridge
import cv2
import sys
import numpy as np


################################################################################
# Extreme left and extreme right initialization
x_init_el = 0
y_init_el = 0
y_end_el = 0

y_init_er = 0
x_end_er = 0
y_end_er = 0

# Left and right
x_end_l = 0
y_end_l = 0
y_init_l = 0
y_end_r = 0

x_init_r = 0
y_init_r = 0

# Definition of the limit for the three sub-images
def set_limit(img_width, img_height):

    # Extreme left and extreme right
    global x_init_el
    global y_init_el
    global y_end_el
    x_init_el = 0
    y_init_el = 0
    y_end_el = int(7.5 * img_height / 12)

    global x_end_er
    global y_end_er
    global y_init_er
    x_end_er = int(img_width)
    y_end_er = int(7.5 * img_height / 12)
    y_init_er = 0

    # Left and right
    global x_end_l
    global y_end_l
    global y_init_l
    x_end_l = int(4 * img_width / 12)
    y_end_l = int(7 * img_height / 12)
    y_init_l = int(1 * img_height / 12)

    global x_init_r
    global y_init_r
    global y_end_r
    x_init_r = int(8 * img_width / 12)
    y_init_r = int(1 * img_height / 12)
    y_end_r = int(7 * img_height / 12)


################################################################################

def draw_optical_flow_field(gray_image, points, flow, dt):
    color_img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
    color_red = [0, 255, 0]  # bgr colorspace
    linewidth = 3

    print(len(points))

    for i in range(len(points)):
        x = points[i, 0]
        y = points[i, 1]
        vx = flow[i, 0] / dt
        vy = flow[i, 1] / dt

        cv2.line(color_img, (x, y), (int(x + vx), int(y + vy)), color_red, linewidth)

    cv2.namedWindow('Optical Flow', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Optical Flow', (600, 600))
    cv2.imshow('Optical Flow', color_img)
    cv2.waitKey(10)


################################################################################


class OFCalculator:

    def __init__(self, param):

	########## IMPORTANT PARAMETERS: ##########
	self.image_sub_name = "front/image_raw"
	self.num_ext_features = 250 
	self.num_cen_features = 100
	self.min_feat_threshold = 0.9
	###########################################

	 # Initialize Image acquisition
        self.bridge = CvBridge()
        # Verbose
        self.show = int(param)
        # Previous Image
        self.prev_image = None
        # Previous key points
        self.prev_kps = None
        # Previous time instant
        self.prev_time = 0
	#Re-usage of features
	self.detect_again = True
        # Masks for the three sub-images
        self.roi_el = np.array([])
        self.roi_er = np.array([])
        self.roi_c = np.array([])

        # Lucas Kanade Optic Flow parameters
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=3,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        # ORB Detector/Descriptor initialization
        self.orb_extreme = cv2.ORB_create(self.num_ext_features)
        self.orb_center = cv2.ORB_create(self.num_cen_features)

        # Raw Image Subscriber Jackal PointGrey
        self.image_sub = rospy.Subscriber(self.image_sub_name, Image, self.callback)

        # Optical flow message Publisher
        self.optic_flow_pub = rospy.Publisher("optical_flow", OpticalFlowMsg, queue_size=10)

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "ok")
        try:
            curr_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        except CvBridgeError as e:
            print(e)
            return

        # Get time stamp
        secs = data.header.stamp.secs
        nsecs = data.header.stamp.nsecs
        curr_time = float(secs) + float(nsecs) * 1e-9
        frequency = 1.0 / (curr_time - self.prev_time)
        print("Frequency: " + str(frequency))

        # First time when no previous image are present
        if self.prev_image is None:
            self.prev_image = curr_image
            self.prev_time = curr_time

            set_limit(data.width, data.height)
            # creating three sub-images from the original image
            self.roi_el = curr_image[y_init_el:y_end_el, x_init_el:x_end_l]
            self.roi_er = curr_image[y_init_er:y_end_er, x_init_r:x_end_er]
            self.roi_c = curr_image[y_init_l:y_end_r, x_end_l:x_init_r]

            keypoints_el = np.array([])
            # ORB to find keypoints
            keypoints_el = np.append(keypoints_el, self.orb_extreme.detect(self.roi_el))
            if (x_init_el != 0) or (y_init_el != 0):
                for i in range(np.size(keypoints_el)):
                    tmp = list(keypoints_el[i].pt)
                    tmp[0] += x_init_el
                    tmp[1] += y_init_el
                    keypoints_el[i].pt = tuple(tmp)

            # ORB to find keypoints
            keypoints_er = np.array([])
            keypoints_er = np.append(keypoints_er, self.orb_extreme.detect(self.roi_er))
            if (x_init_r != 0) or (y_init_er != 0):
                for i in range(np.size(keypoints_er)):
                    tmp = list(keypoints_er[i].pt)
                    tmp[0] += x_init_r
                    tmp[1] += y_init_er
                    keypoints_er[i].pt = tuple(tmp)

            # ORB to find keypoints
            keypoints_c = np.array([])
            keypoints_c = np.append(keypoints_c, self.orb_center.detect(self.roi_c))
            if (x_end_l != 0) or (y_init_l != 0):
                for i in range(np.size(keypoints_c)):
                    tmp = list(keypoints_c[i].pt)
                    tmp[0] += x_end_l
                    tmp[1] += y_init_l
                    keypoints_c[i].pt = tuple(tmp)

            # Store all the keypoints in a single array
            keypoints = np.array([])
            keypoints = np.append(keypoints, keypoints_el)
            keypoints = np.append(keypoints, keypoints_er)
            keypoints = np.append(keypoints, keypoints_c)

            if np.size(keypoints) > 0:
                p0 = cv2.KeyPoint_convert(keypoints)
                self.prev_kps = p0.reshape(-1, 1, 2)
            else:
                self.prev_kps = None
                print("Features detected: 0")
            return

        # When a previous frame is already present
	if self.detect_again == True:

            # creating three sub-images
      	    self.roi_el = curr_image[y_init_el:y_end_el, x_init_el:x_end_l]
            self.roi_er = curr_image[y_init_er:y_end_er, x_init_r:x_end_er]
            self.roi_c = curr_image[y_init_l:y_end_r, x_end_l:x_init_r]

            keypoints_el = np.array([])
            keypoints_el = np.append(keypoints_el, self.orb_extreme.detect(self.roi_el))
            if (x_init_el != 0) or (y_init_el != 0):
                for i in range(np.size(keypoints_el)):
                    tmp = list(keypoints_el[i].pt)
                    tmp[0] += x_init_el
                    tmp[1] += y_init_el
                    keypoints_el[i].pt = tuple(tmp)

            keypoints_er = np.array([])
            keypoints_er = np.append(keypoints_er, self.orb_extreme.detect(self.roi_er))
            if (x_init_r != 0) or (y_init_er != 0):
                for i in range(np.size(keypoints_er)):
                    tmp = list(keypoints_er[i].pt)
                    tmp[0] += x_init_r
                    tmp[1] += y_init_er
                    keypoints_er[i].pt = tuple(tmp)

            keypoints_c = np.array([])
            keypoints_c = np.append(keypoints_c, self.orb_center.detect(self.roi_c))
            if (x_end_l != 0) or (y_init_l != 0):
                for i in range(np.size(keypoints_c)):
                    tmp = list(keypoints_c[i].pt)
                    tmp[0] += x_end_l
                    tmp[1] += y_init_l
                    keypoints_c[i].pt = tuple(tmp)

            keypoints = np.array([])
            keypoints = np.append(keypoints, keypoints_el)
            keypoints = np.append(keypoints, keypoints_er)
            keypoints = np.append(keypoints, keypoints_c)

            if np.size(keypoints) > 0:
                p0 = cv2.KeyPoint_convert(keypoints)
                curr_kps = p0.reshape(-1, 1, 2)
            else:
                curr_kps = None
                print("Features detected: 0")
	else:
	    curr_kps = self.prev_kps

        # Get time between images
        dt = curr_time - self.prev_time
        # print("Time: " + str(dt))

        # Calculate optic flow with lucas kanade
        if self.prev_kps is None:
            self.prev_image = curr_image
            self.prev_kps = curr_kps
            self.prev_time = curr_time
            return

        tracked_features, status, error = cv2.calcOpticalFlowPyrLK(self.prev_image, curr_image,
                                                                   self.prev_kps, None,
                                                                   **self.lk_params)

        # Select good points (the keypoints with a correspondance between two frames)
        good_kps_new = tracked_features[status == 1]
        good_kps_old = self.prev_kps[status == 1]

	if np.size(good_kps_new) <= self.min_feat_threshold*np.size(self.prev_kps):
		self.detect_again = True
	else:
		self.detect_again = False
		curr_kps = good_kps_new

        # Calculate flow field
        flow = good_kps_new - good_kps_old

        # Draw the flow field
        if self.show == 1:
            draw_optical_flow_field(curr_image, good_kps_old, flow, dt)

        # Publish Optical Flow data to rostopic
        # Creation of OpticalFlow.msg
        msg = OpticalFlowMsg()
        msg.header.stamp.secs = secs
        msg.header.stamp.nsecs = nsecs

        msg.height = data.height
        msg.width = data.width

        msg.dt = dt  # in msec
        # displacement coordinates
        msg.x = good_kps_old[:, 0]
        msg.y = good_kps_old[:, 1]
        # velocity coordinates
        msg.vx = flow[:, 0] / dt
        msg.vy = flow[:, 1] / dt
        self.optic_flow_pub.publish(msg)

        # Update frame
        self.prev_image = curr_image
        self.prev_kps = curr_kps
        self.prev_time = curr_time
		

def optical_flow(param):
    rospy.init_node("optical_flow", anonymous=False)
    OFCalculator(param)
    rospy.spin()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        parameter = str(0)
        print("Parameter = 1, verbose mode")
    else:
        parameter = sys.argv[1]
    optical_flow(parameter)
