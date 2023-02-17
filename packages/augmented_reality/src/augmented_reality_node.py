#!/usr/bin/env python3
import os
from pathlib import Path
import rospy
import cv2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
import yaml

class AugmentedRealityNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(AugmentedRealityNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.veh = rospy.get_param("~veh")
        self.map_file = rospy.get_param("~map_file")
        self.calibration_file = f'/data/config/calibrations/camera_extrinsic/{self.veh}.yaml'

        self.map = self.readYamlFile(self.map_file)
        self.calibration = self.readYamlFile(self.calibration_file)

        print(self.calibration)
        # construct publisher
        #self.sub_img = rospy.Subscriber(f'/{self.veh}/camera_node/image/compressed', CompressedImage, self.get_img)
        self.pub_img = rospy.Publisher(f'/{self.veh}/{node_name}/{Path(self.map_file).stem}/image/compressed', CompressedImage, queue_size=10)

    def get_img(self, msg):
        print(msg)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            message = "Hello World!"
            rospy.loginfo("Publishing message: '%s'" % message)
            self.pub.publish(message)
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
    # run node
    #node.run()
    # keep spinning
    rospy.spin()