#!/usr/bin/env python3
import os
from pathlib import Path
import rospy
import cv2
import numpy as np

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from sensor_msgs.msg import CompressedImage


class LaneTrackingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LaneTrackingNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self.veh = rospy.get_param("~veh")
        self.draw = True
        self.english = False
        self.linear_speed = 3
        self.yellow_line_offset = 5
        self.min_contour_lenght_white = 20
        self.horozontal_road_line_offset = 10
        self.road_marker_offset = 10
        
        # -- euclidian image --
        self.img_euclid = None
    
        # -- subscribers -- 
        self.img_sub = rospy.Subscriber(f'/{self.veh}/augmented_reality_node/image_color/compressed', CompressedImage, self.get_img)

        # -- publisher -- 
        self.pub_img_testing = rospy.Publisher(f'/{self.veh}/{node_name}/image/compressed', CompressedImage, queue_size=1)
        self.pub_twist = rospy.Publisher(f'/{self.veh}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)



    def get_img(self, msg):
        img = np.frombuffer(msg.data, np.uint8)
        self.img_euclid  = cv2.imdecode(img, 1)     


    def region_of_interest(self, img, vertices):
        # Define a blank matrix that matches the image height/width.
        mask = np.zeros(img.shape[:2], dtype="uint8")
        
        # Fill inside the polygon
        cv2.fillPoly(mask, vertices, 255)
        
        # Returning the image only where mask pixels match
        masked_image = cv2.bitwise_and(img, img, mask=mask)

        return masked_image


    def HSV_filter(self, img, hsv_values):
        # Set minimum and max HSV values to display
        lower = np.array([hsv_values['hMin'], hsv_values['sMin'], hsv_values['vMin']])
        upper = np.array([hsv_values['hMax'], hsv_values['sMax'], hsv_values['vMax']])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)

        output = cv2.bitwise_and(img, img, mask= mask)
        
        return cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)


    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=3):
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

    def get_centroids(self, img, num, min_lenght=-1):
        # -- find contours on yellow line -- 
        contours, hierarchies = cv2.findContours(
            img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )

        centroids = []

        if len(contours) < num:
            return

        for i in range(num):
            M = cv2.moments(contours[i])
            if M['m00'] != 0:
                # central moment of contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                if self.draw:
                    cv2.drawContours(img, [contours[i]], -1, (0, 255, 0), 1)
                    cv2.circle(img, (cx, cy), 2, (0, 0, 255), -1)

                if cv2.arcLength(contours[i], True) > min_lenght:
                    centroids.append([cx, cy])

        return centroids, img


    def find_lanes(self):
        img = cv2.pyrDown(self.img_euclid)
        (h, w, _) = img.shape

        yellow_HSV_filter =  {'hMin':21, 'sMin':68, 'vMin':0, 'hMax':111, 'sMax':201, 'vMax':255}
        white_HSV_filter = {'hMin':103, 'sMin':0, 'vMin':141, 'hMax':132, 'sMax':44, 'vMax':255}

        yellow_img = self.HSV_filter(img, yellow_HSV_filter)

        centroids_yellow, img = self.get_centroids(yellow_img, 2)

        if centroids_yellow is None:
            pass

        # == lines to draw ==
        # -- line connecting centroids -- 
        centroid_line = [centroids_yellow[0][0], centroids_yellow[0][1], centroids_yellow[1][0], centroids_yellow[1][1]]
        slope = (centroid_line[3] - centroid_line[1]) / (centroid_line[2] - centroid_line[0])

        # -- inner side of yellow line -- 
        x1_yellow = centroids_yellow[0][0] + self.english*(-1) * self.yellow_line_offset
        y1_yellow = int(-1/slope * x1_yellow + (centroids_yellow[0][1] + 1/slope * centroids_yellow[0][0]))
        x2_yellow = centroids_yellow[1][0] + self.english*(-1) * self.yellow_line_offset
        y2_yellow = int(-1/slope * x2_yellow + (centroids_yellow[1][1] + 1/slope * centroids_yellow[1][0]))

        yellow_line = [x1_yellow, y1_yellow, x2_yellow, y2_yellow]

        # -- inner side of yellow line - long -- 
        x1_yellow_long = int(x1_yellow - y1_yellow / slope)

        yellow_line_long = [x1_yellow_long, 0, x1_yellow, y1_yellow]

        # == get centroid on white line == 
        region_of_interest_vertices = [
            (50, centroids_yellow[0][1]),
            (w, centroids_yellow[0][1]),
            (w, centroids_yellow[0][3]),
            (50, centroids_yellow[0][3]),
        ]

        cropped_image = self.region_of_interest(
            img,
            np.array([region_of_interest_vertices], np.int32),
        )

        white_img = self.HSV_filter(cropped_image, white_HSV_filter)

        centroids_white, img = self.get_centroids(white_img, 1, min_lenght=self.min_contour_lenght_white)

        # -- horozontal road line -- 
        x1_road = (x1_yellow + x2_yellow) // 2
        y1_road = (y1_yellow + y2_yellow) // 2
        x2_road = centroids_white[0] - self.horozontal_road_line_offset
        y2_road = centroids_white[1]
        
        road_line = [x1_road, y1_road, x2_road, y2_road]

        # -- middle of road marker -- 
        x1_road_mid = (x1_road+x2_road) // 2
        y1_road_mid = (y1_road+y2_road) // 2

        road_mid_line = [x1_road_mid, y1_road_mid - self.road_marker_offset, x1_road_mid, 
                         y1_road_mid + self.road_marker_offset]

        # -- middle of image marker -- 
        x1_img_mid = w//2
        y1_img_mid = (y1_road+y2_road) // 2
        
        img_mid_line = [x1_img_mid,  y1_img_mid - self.road_marker_offset, x1_img_mid, 
                        y1_img_mid + self.road_marker_offset]
                        
        if self.draw:
            cv2.circle(img, (w//2, y1_img_mid), 2, (0, 255, 0), -1)
            cv2.circle(self.img, (x1_road_mid, y1_road_mid), 2, (255, 255, 0), -1)

            cv2.line(img, (centroid_line[0], centroid_line[1]), (centroid_line[2], centroid_line[3]), color=[0, 0, 255], thickness=1)

            cv2.line(img, (yellow_line[0], yellow_line[1]), (yellow_line[2], yellow_line[3]), color=[255, 0, 0], thickness=2)
            cv2.line(img, (road_line[0], road_line[1]), (road_line[2], road_line[3]), color=[255, 0, 0], thickness=2)

            cv2.line(img, (yellow_line_long[0], yellow_line_long[1]), (yellow_line_long[2], yellow_line_long[3]), color=[255, 0, 0], thickness=1)

            cv2.line(img, (road_mid_line[0], road_mid_line[1]), (road_mid_line[2], road_mid_line[3]), color=[255, 255, 0], thickness=1)
            cv2.line(img, (img_mid_line[0], img_mid_line[1]), (img_mid_line[2], img_mid_line[3]), color=[0, 0, 255], thickness=1)


        self.display_img(img)


        return x1_road_mid - x1_img_mid


    def PID_Control(self):
        pass


    def display_img(self, img):
        new_img = CompressedImage()
        new_img.data = cv2.imencode('.jpg', img)[1].tobytes()

        self.pub_img_testing.publish(new_img)


    def move(self, v, omega):
        twist = Twist2DStamped()
        twist.v = v
        twist.omega = omega
        self.pub_twist.publish(twist)


    def run(self):
        rate = rospy.Rate(1)

        #self.move(self.linear_speed, 0)

        while not rospy.is_shutdown():
            if self.img_euclid is not None:

                pid_signal = self.find_lanes()

                rate.sleep()

    

if __name__ == '__main__':
    # create the node
    node = LaneTrackingNode(node_name='lane_tracking_node')

    try:
        node.run()

    except rospy.ROSInterruptException:
        pass


