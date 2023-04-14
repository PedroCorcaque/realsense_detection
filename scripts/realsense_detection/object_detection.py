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

    def __init__(self, image_topic: str) -> None:
        self.image_topic = image_topic

        self.image_rgb = None

        self.network = ObjectDetection.load_network()

        source_topic_dict = {
            "image_rgb": self.image_topic
        }

        VisionSynchronizer.syncSubscribers(source_topic_dict, \
                                           self.vision_synchronizer_cb)

    def vision_synchronizer_cb(self, *args) -> None:
        """ The callback to image and point cloud synchronized """
        if len(args) > 0:
            self.image_rgb = args[0]

        np_image = ros_numpy.numpify(self.image_rgb)

        cuda_image = cudaFromNumpy(np_image)

        # Detection
        detections = self.network.Detect(cuda_image, overlay="box,labels,conf")
        print(f"detected {len(detections)} objects in image")

        for detection in detections:
            print(detection)

def main(camera="realsense") -> None:
    """ A main function to choose the option """
    if camera == "realsense":
        image_color_topic = "/camera/color/image_raw"
        _ = ObjectDetection(image_color_topic)
    elif camera == "zed":
        image_color_topic = "/zed/stereo/image_rect_color"
        _ = ObjectDetection(image_color_topic)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("object_detection_node")
    main()
