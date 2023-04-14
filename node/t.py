#!/usr/bin/env python3
import rospy

from realsense_detection.object_detection import ObjectDetection

if __name__ == "__main__":
    rospy.init_node("test")
    _ = ObjectDetection()