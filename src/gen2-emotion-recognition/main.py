from depthai_sdk import OakCamera
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time
import depthai as dai

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.bridge = CvBridge()

    def publish_image(self, cv_image):
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    nn = pipeline.create(dai.node.MobileNetDetectionNetwork)

    camRgb.setPreviewSize(300, 300)
    camRgb.setInterleaved(False)
    camRgb.setFps(40)
    nn.setConfidenceThreshold(0.5)
    nn.setNumInferenceThreads(2)
    nn.input.setBlocking(False)

    camRgb.preview.link(nn.input)

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)

        frame = None
        detections = []

        while True:
            inRgb = qRgb.tryGet()
            inDet = qDet.tryGet()

            if inRgb is not None:
                frame = inRgb.getCvFrame()
                image_publisher.publish_image(frame)

            if inDet is not None:
                detections = inDet.detections

            if frame is not None:
                color = (255, 0, 0)
                for detection in detections:
                    bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                    cv2.putText(frame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                    cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                cv2.imshow("rgb", frame)

            if cv2.waitKey(1) == ord('q'):
                break

        rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
