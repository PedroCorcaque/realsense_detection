from realsense_detection.object_detection import ObjectDetection

def test_create_object() -> bool:
    """ Test if ObjectDetection class can be created """
    try:
        _ = ObjectDetection("/topic/test", "/topic/test")
        return True
    except:
        return False
    