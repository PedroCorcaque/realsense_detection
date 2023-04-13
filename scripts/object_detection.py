#!/usr/bin/env python3
import rospy
import ros_numpy

from jetson_inference import detectNet
from jetson_utils import cudaFromNumpy

from butia_vision_bridge import VisionSynchronizer

class ObjectDetection():
    """ The ObjectDetection class is used to load the network detection and 
    get the position of the object """

    @staticmethod
    def load_network():
        """ Load the detection network """
        network = "ssd-mobilenet-v2"
        return detectNet(network, threshold=0.5)

    def __init__(self, point_cloud_topic: str, image_topic: str) -> None:
        self.point_cloud_topic = point_cloud_topic
        self.image_topic = image_topic

        self.image  = None
        self.points = None

        self.network = ObjectDetection.load_network()

        source_topic_dict = {
            "image_rgb": self.image_topic,
            "points": self.point_cloud_topic
        }

        VisionSynchronizer.syncSubscribers(source_topic_dict, \
                                           self.vision_synchronizer_cb)

    def vision_synchronizer_cb(self, *args) -> None:
        """ The callback to image and point cloud synchronized """
        if len(args) > 0:
            self.image = args[0]
            self.points = args[1]

        np_image    = ros_numpy.numpify(self.image)
        np_points   = ros_numpy.numpify(self.points)

        cuda_image = cudaFromNumpy(np_image)

        # Detection
        detections = self.network.Detect(cuda_image, overlay="box,labels,conf")
        print(f"detected {len(detections)} objects in image")

        for detection in detections:
            print(detection)

        print(f"{self.network.GetNetworkFPS()} FPS".format())

def main() -> None:
    """ A main function to choose the option """
    pass

if __name__ == "__main__":
    rospy.init_node("object_detection_node")
    main()
