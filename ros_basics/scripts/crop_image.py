#!/usr/bin/python3

import rospy
import cv_bridge

from sensor_msgs.msg import Image


class CropImageNode:

    def __init__(self):
        rospy.init_node('crop_image')

        self.x_top_left = rospy.get_param('~x_top_left', 0)
        self.y_top_left = rospy.get_param('~y_top_left', 0)
        self.x_bottom_right = rospy.get_param('~x_bottom_right', -1)
        self.y_bottom_right = rospy.get_param('~y_bottom_right', -1)

        rospy.loginfo('x_top_left = %i', self.x_top_left)
        rospy.loginfo('y_top_left = %i', self.y_top_left)
        rospy.loginfo('x_bottom_right = %i', self.x_bottom_right)
        rospy.loginfo('y_bottom_right = %i', self.y_bottom_right)

        self.sub = rospy.Subscriber('image', Image, self.on_image, queue_size=10)
        self.pub = rospy.Publisher('image_crop', Image, queue_size=10)

        self.br = cv_bridge.CvBridge()
    

    def on_image(self, image_msg : Image):
        image = self.br.imgmsg_to_cv2(image_msg)
        image = image[self.y_top_left:self.y_bottom_right, self.x_top_left:self.x_bottom_right]
        image_msg = self.br.cv2_to_imgmsg(image)
        self.pub.publish(image_msg)


    def spin(self):
        rospy.spin()


def main():
    node = CropImageNode()
    node.spin()


if __name__ == '__main__':
    main()
