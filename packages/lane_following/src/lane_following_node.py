#!/usr/bin/env python3
import os
from pathlib import Path
import rospy
import cv2
import numpy as np

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelEncoderStamped, Twist2DStamped
import yaml


class LaneFollowingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneFollowingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_param("~veh")
        self.map_file = rospy.get_param("~map_file")

        twist = f'/{self.veh_name}/car_cmd_switch_node/cmd'

        #self.calibration_file = f'/data/config/calibrations/camera_intrinsic/{self.veh}.yaml'
        self.calibration_file = f'/data/config/calibrations/camera_intrinsic/default.yaml'
        
        self.map = self.readYamlFile(self.map_file)
        self.calibration = self.readYamlFile(self.calibration_file)

        self.img_width = self.calibration['image_width']
        self.img_height = self.calibration['image_height']
        self.cam_matrix = np.array(self.calibration['camera_matrix']['data']).reshape((self.calibration['camera_matrix']['rows'], self.calibration['camera_matrix']['cols']))
        self.distort_coeff = np.array(self.calibration['distortion_coefficients']['data']).reshape((self.calibration['distortion_coefficients']['rows'], self.calibration['distortion_coefficients']['cols']))

        self.new_cam_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.distort_coeff, (self.img_width, self.img_height), 1, (self.img_width, self.img_height))

        self.undistorted = None
    
        # construct publisher
        self.sub_img = rospy.Subscriber(f'/{self.veh}/camera_node/image/compressed', CompressedImage, self.get_img)
        self.pub_img = rospy.Publisher(f'/{self.veh}/{node_name}/{Path(self.map_file).stem}/image/compressed', CompressedImage, queue_size=1)
        self.pub_twist = rospy.Publisher(twist, Twist2DStamped, queue_size=1)
        

    def get_img(self, msg):
        img = np.frombuffer(msg.data, np.uint8)
        img2 = cv2.imdecode(img, 0)     

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        undistorted = cv2.undistort(img2, self.cam_matrix, self.distort_coeff, None, self.new_cam_matrix)
        x, y, w, h = self.roi
        self.undistorted = undistorted[y:y+h, x:x+w]


    def move(self, v, omega):
        # move using twist by sending velocity and omega
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        self.pub_twist.publish(twist)


    def region_of_interest(img, vertices):
        # Define a blank matrix that matches the image height/width.
        mask = np.zeros(img.shape[:2], dtype="uint8")
        
        # Fill inside the polygon
        cv2.fillPoly(mask, vertices, 255)
        
        # Returning the image only where mask pixels match
        masked_image = cv2.bitwise_and(img, img, mask=mask)

        return masked_image


    def HSV_filter(img, hsv_values):

        # Set minimum and max HSV values to display
        lower = np.array([hsv_values['hMin'], hsv_values['sMin'], hsv_values['vMin']])
        upper = np.array([hsv_values['hMax'], hsv_values['sMax'], hsv_values['vMax']])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        output = cv2.bitwise_and(img, img, mask= mask)
        
        return cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)


    def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
        # If there are no lines to draw, exit.
        if lines is None:
            return
        
        # Make a copy of the original image.
        img = np.copy(img)

        # Loop over all lines and draw them on the blank image.
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)


        # Return the modified image.
        return img


    def find_max_line_lenght(lines):
        max_line = lines[0,0]
        max_len = 0

        if len(lines) == 0:
            return 
        
        for line in lines:
            lenght = np.sqrt((line[0,0]-line[0,2])**2 + (line[0,1]-line[0,3])**2)
            print(lenght)
            if lenght > max_len:
                max_len = lenght
                max_line = line

        return [max_line]


    def find_lanes(self):
        img = cv2.pyrDown(cv2.pyrDown(cv2.pyrDown(self.undistorted)))

        (height, width, _) = img.shape

        region_of_interest_vertices = [
            (0, height),
            (0, height-50),
            (width // 2 -25, 50),
            (width // 2 +25, 50),
            (width, height-50),
            (width, height),
        ]


        cropped_image = self.region_of_interest(
            img,
            np.array([region_of_interest_vertices], np.int32),
        )


        yellow_HSV_filter =  {'hMin':21, 'sMin':68, 'vMin':0, 'hMax':111, 'sMax':201, 'vMax':255}
        white_HSV_filter = {'hMin':92, 'sMin':21, 'vMin':124, 'hMax':121, 'sMax':67, 'vMax':255}


        yellow_img = self.HSV_filter(img, yellow_HSV_filter)
        white_img = self.HSV_filter(img, white_HSV_filter)


        # Call Canny Edge Detection here.
        yellow_edges = cv2.Canny(yellow_img, 100, 300)
        white_edges = cv2.Canny(white_img, 100, 300)


        yellow_line_img = self.region_of_interest(
            yellow_edges,
            np.array(
                [region_of_interest_vertices],
                np.int32
            ),
        )

        white_line_img = self.region_of_interest(
            white_edges,
            np.array(
                [region_of_interest_vertices],
                np.int32
            ),
        )

        white_lines = cv2.HoughLinesP(
            white_line_img,
            rho=6,
            theta=np.pi / 180,
            threshold=100,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=25
        )

        yellow_lines = cv2.HoughLinesP(
            yellow_line_img,
            rho=6,
            theta=np.pi / 180,
            threshold=100,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=25
        )


        white_line = self.find_max_line_lenght(white_lines)
        yellow_line = self.find_max_line_lenght(yellow_lines)

        lines = np.concatenate((white_line, yellow_line), axis=0)

        slope_right = (lines[0,0,3] - lines[0,0,1]) / (lines[0,0,2] - lines[0,0,0])
        slope_left = (lines[1,0,3] - lines[1,0,1]) / (lines[1,0,2] - lines[1,0,0])


        max_y = height
        min_y = int(height // 2)

        left_x_start = int((max_y - lines[1,0,1]) / slope_left + lines[1,0,0])
        left_x_end = int((min_y - lines[1,0,1]) / slope_left + lines[1,0,0])

        right_x_start = int((max_y - lines[0,0,1]) / slope_right + lines[0,0,0])
        right_x_end = int((min_y - lines[0,0,1]) / slope_right + lines[0,0,0])


        line_image = self.draw_lines(
            img,
            [[
                [left_x_start, max_y, left_x_end, min_y],
                [right_x_start, max_y, right_x_end, min_y],
            ]],
            thickness=5,
        )

        cv2.imshow(line_image)



    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.undistorted is not None and not self.is_processed:
                self.find_lanes()

    

if __name__ == '__main__':
    # create the node
    node = LaneFollowingNode(node_name='lane_following_node')
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass

    # run node
    #node.run()
    # keep spinning
    #rospy.spin()