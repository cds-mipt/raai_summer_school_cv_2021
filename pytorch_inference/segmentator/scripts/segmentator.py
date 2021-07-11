#!/usr/bin/env python3

import rospy
import torch
import cv_bridge
import numpy as np

from torchvision.models.segmentation import fcn_resnet50

from sensor_msgs.msg import Image


class SegmentatorNode:

    def __init__(self):
        rospy.init_node('segmentator')

        rospy.loginfo('Loading model...')
        self.model = fcn_resnet50(pretrained=True)
        if torch.cuda.is_available():
            self.model = self.model.cuda()
        rospy.loginfo('Model loaded')

        self.sub = rospy.Subscriber('image', Image, self.on_image, queue_size=10)
        self.pub = rospy.Publisher('segmentation', Image, queue_size=10)

        self.br = cv_bridge.CvBridge()


    def on_image(self, image_msg : Image):
        image = self.br.imgmsg_to_cv2(image_msg)
        batch = SegmentatorNode.preproc_cv2(image)
        if torch.cuda.is_available():
            batch = batch.cuda()
        logits = self.model(batch)

        probs = torch.softmax(logits['aux'][0], 0)
        segm = probs.argmax(dim=0) * (probs.max(dim=0).values > 0.5)

        segm_msg = self.br.cv2_to_imgmsg(segm.cpu().numpy().astype(np.uint8), 'mono8')
        segm_msg.header = image_msg.header
        self.pub.publish(segm_msg)


    @staticmethod
    def preproc_cv2(image):
        image_tensor = torch.Tensor(image.copy()).float() / 255
        mean = torch.Tensor([0.485, 0.456, 0.406])
        std = torch.Tensor([0.229, 0.224, 0.225])

        image_tensor = (image_tensor - mean) / std
        image_tensor = image_tensor.permute(2, 0, 1)

        batch = image_tensor.unsqueeze(0)

        return batch


    def spin(self):
        rospy.spin()


def main():
    node = SegmentatorNode()
    node.spin()


if __name__ == '__main__':
    main()
