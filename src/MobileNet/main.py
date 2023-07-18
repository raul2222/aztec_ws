#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
import time
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

nnPathDefault = str((Path(__file__).parent / Path('../models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute())
parser = argparse.ArgumentParser()
parser.add_argument('nnPath', nargs='?', help="Path to mobilenet detection network blob", default=nnPathDefault)
parser.add_argument('-s', '--sync', action="store_true", help="Sync RGB output with NN output", default=False)
args = parser.parse_args()

if not Path(nnPathDefault).exists():
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')

# MobilenetSSD label texts
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'topic', 10)
        self.bridge = CvBridge()

        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.nn = self.pipeline.create(dai.node.MobileNetDetectionNetwork)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.nnOut = self.pipeline.create(dai.node.XLinkOut)
        self.nnNetworkOut = self.pipeline.create(dai.node.XLinkOut)

        self.xoutRgb.setStreamName("rgb")
        self.nnOut.setStreamName("nn")
        self.nnNetworkOut.setStreamName("nnNetwork");

        # Properties
        self.camRgb.setPreviewSize(300, 300)
        self.camRgb.setInterleaved(False)
        self.camRgb.setFps(40)
        # Define a neural network that will make predictions based on the source frames
        self.nn.setConfidenceThreshold(0.5)
        self.nn.setBlobPath(args.nnPath)
        self.nn.setNumInferenceThreads(2)
        self.nn.input.setBlocking(False)

        # Linking
        if args.sync:
            self.nn.passthrough.link(self.xoutRgb.input)
        else:
            self.camRgb.preview.link(self.xoutRgb.input)

        self.camRgb.preview.link(self.nn.input)
        self.nn.out.link(self.nnOut.input)
        self.nn.outNetwork.link(self.nnNetworkOut.input);

        self.frame = None
        self.detections = []

    def frameNorm(self, frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    def displayFrame(self, frame, detections):
        color = (0, 255, 255)  # Amarillo (BGR)
        for detection in detections:
            bbox = self.frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            cv2.putText(frame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
        return frame

    def publish_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(msg)

    def run(self):
        # Connect to device and start pipeline
        with dai.Device(self.pipeline) as device:

            # Output queues will be used to get the rgb frames and nn data from the outputs defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            qNN = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)

            startTime = time.monotonic()
            counter = 0
            color2 = (255, 255, 255)

            printOutputLayersOnce = True

            while True:
                if args.sync:
                    # Use blocking get() call to catch frame and inference result synced
                    inRgb = qRgb.get()
                    inDet = qDet.get()
                    inNN = qNN.get()
                else:
                    # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
                    inRgb = qRgb.tryGet()
                    inDet = qDet.tryGet()
                    inNN = qNN.tryGet()

                if inRgb is not None:
                    self.frame = inRgb.getCvFrame()
                    cv2.putText(self.frame, "NN fps: {:.2f}".format(counter / (time.monotonic() - startTime)),
                                (2, self.frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color2)

                if inDet is not None:
                    self.detections = inDet.detections
                    counter += 1

                if printOutputLayersOnce and inNN is not None:
                    toPrint = 'Output layer names:'
                    for ten in inNN.getAllLayerNames():
                        toPrint = f'{toPrint} {ten},'
                    print(toPrint)
                    printOutputLayersOnce = False

                # If the frame is available, draw bounding boxes on it and show the frame
                if self.frame is not None:
                    self.frame = self.displayFrame(self.frame, self.detections)
                    self.publish_image(self.frame)

                if cv2.waitKey(1) == ord('q'):
                    break

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    image_publisher.run()
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
