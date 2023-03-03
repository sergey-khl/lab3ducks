#!/usr/bin/env python3
import os
from pathlib import Path
import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Quaternion, Pose, Point
import yaml
import apriltag as at

class AugmentedRealityNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AugmentedRealityNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_param("~veh")
        self.map_file = rospy.get_param("~map_file")
        #self.calibration_file = f'/data/config/calibrations/camera_intrinsic/{self.veh}.yaml'
        self.calibration_file = f'/data/config/calibrations/camera_intrinsic/default.yaml'
        

        self.map = self.readYamlFile(self.map_file)
        self.calibration = self.readYamlFile(self.calibration_file)

        self.img_width = self.calibration['image_width']
        self.img_height = self.calibration['image_height']
        self.cam_matrix = np.array(self.calibration['camera_matrix']['data']).reshape((self.calibration['camera_matrix']['rows'], self.calibration['camera_matrix']['cols']))
        self.distort_coeff = np.array(self.calibration['distortion_coefficients']['data']).reshape((self.calibration['distortion_coefficients']['rows'], self.calibration['distortion_coefficients']['cols']))

        self.new_cam_matrix, self.roi = cv2.getOptimalNewCameraMatrix(self.cam_matrix, self.distort_coeff, (self.img_width, self.img_height), 1, (self.img_width, self.img_height))

        # setup april tag detector
        options = at.DetectorOptions(families="tag36h11")
        self.detector = at.Detector(options)

        self.undistorted = None
    
        # construct publisher
        self.sub_img = rospy.Subscriber(f'/{self.veh}/camera_node/image/compressed', CompressedImage, self.get_img)
        self.pub_img = rospy.Publisher(f'/{self.veh}/{node_name}/{Path(self.map_file).stem}/image/compressed', CompressedImage, queue_size=1)
        self.pub_loc = rospy.Publisher(f'/{self.veh}/teleport', Pose, queue_size=1) 

        

    def get_img(self, msg):
        img = np.frombuffer(msg.data, np.uint8)
        img2 = cv2.imdecode(img, 0)     

        # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
        undistorted = cv2.undistort(img2, self.cam_matrix, self.distort_coeff, None, self.new_cam_matrix)
        x, y, w, h = self.roi
        self.undistorted = undistorted[y:y+h, x:x+w]
        


    def detect_april(self):
        # https://pyimagesearch.com/2020/11/02/apriltag-with-python/
        # april tag detection
        
        results = self.detector.detect(self.undistorted)
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(self.undistorted, ptA, ptB, (0, 255, 0), 2)
            cv2.line(self.undistorted, ptB, ptC, (0, 255, 0), 2)
            cv2.line(self.undistorted, ptC, ptD, (0, 255, 0), 2)
            cv2.line(self.undistorted, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(self.undistorted, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagID = str(r.tag_id)
            cv2.putText(self.undistorted, tagID, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            #print("[INFO] tag id: {}".format(tagID))
            
            # 94, 93, 62, 162, 201, 153
        

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            
            if self.undistorted is not None:
                
                self.detect_april()
                new_img = CompressedImage()
                new_img.data = cv2.imencode('.jpg', self.undistorted)[1].tobytes()
                self.pub_img.publish(new_img)
                
            else:

                # init location
                pose = Pose(Point(0.32, 0.3, 0), Quaternion(0, 0, 0, 1))
                self.pub_loc.publish(pose)
        rate.sleep()

    def readYamlFile(self,fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                        %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0 , 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, (pt_x[0], pt_y[0]), (pt_x[1], pt_y[1]), (b * 255, g * 255, r * 255), 5)
        return image

if __name__ == '__main__':
    # create the node
    node = AugmentedRealityNode(node_name='augmented_reality_node')
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    # run node
    #node.run()
    # keep spinning
    #rospy.spin()