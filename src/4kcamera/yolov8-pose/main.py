# coding=utf-8
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from utils import non_max_suppression, FPSHandler, plot_skeleton_kpts

ROOT = Path(__file__).parent

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.image_pub = self.create_publisher(Image, "/camera/image", 1)
        self.bridge = CvBridge()

    def publish_image(self, frame):
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(image_message)

def create_pipeline():
    blob = ROOT.joinpath("yolov8n-pose.blob")
    model = dai.OpenVINO.Blob(blob)
    dim = next(iter(model.networkInputs.values())).dims
    nnWidth, nnHeight = dim[:2]
    print(f"{nnWidth, nnHeight = }")
    print(f"{blob = }")

    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.ColorCamera)
    detectionNetwork = pipeline.create(dai.node.NeuralNetwork)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("image")
    xoutNN.setStreamName("detections")

    camRgb.setPreviewSize(nnWidth, nnHeight)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(60)
    camRgb.setPreviewKeepAspectRatio(False)

    detectionNetwork.setBlob(model)

    camRgb.preview.link(detectionNetwork.input)
    camRgb.preview.link(xoutRgb.input)
    detectionNetwork.out.link(xoutNN.input)

    return pipeline

def run():
    with dai.Device(create_pipeline()) as device:
        imageQueue = device.getOutputQueue(name="image", maxSize=4, blocking=False)
        detectQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

        frame = None
        detections = []

        fpsHandler = FPSHandler()

        def drawText(frame, text, org, color=(255, 255, 255)):
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        def displayFrame(name, frame):
            color = (10, 255, 255)

            for detection in detections:
                *bbox, conf, cls = detection[:6]
                bbox = np.array(bbox).astype(int)
                kpts = detection[6:]

                drawText(frame, f"{conf:.2%}", (bbox[0] + 10, bbox[1] + 35),)
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                frame = plot_skeleton_kpts(frame, kpts=kpts, steps=3)

           # cv2.imshow(name, frame)
            return frame

        def toTensorResult(packet):
            data = {}
            for tensor in packet.getRaw().tensors:
                if tensor.dataType == dai.TensorInfo.DataType.INT:
                    data[tensor.name] = np.array(packet.getLayerInt32(tensor.name)).reshape(tensor.dims)
                elif tensor.dataType == dai.TensorInfo.DataType.FP16:
                    data[tensor.name] = np.array(packet.getLayerFp16(tensor.name)).reshape(tensor.dims)
                elif tensor.dataType == dai.TensorInfo.DataType.I8:
                    data[tensor.name] = np.array(packet.getLayerUInt8(tensor.name)).reshape(tensor.dims)
                else:
                    print("Unsupported tensor layer type: {}".format(tensor.dataType))
            return data

        while not device.isClosed():
            batch_bboxes, batch_poses, batch_scores = [], [], []
            imageQueueData = imageQueue.tryGet()
            detectQueueData = detectQueue.tryGet()

            if imageQueueData is not None:
                frame = imageQueueData.getCvFrame()
                fpsHandler.tick("color")

            if detectQueueData is not None:
                pred = toTensorResult(detectQueueData)["output0"]
                fpsHandler.tick("nn")
                detections = non_max_suppression(pred, 0.5, nc=80)[0]

            if frame is not None:
                fpsHandler.drawFps(frame, "color")

                frame = displayFrame("image", frame)
                image_publisher.publish_image(frame)  # Publish the frame
                frame = None

            if cv2.waitKey(1) == ord("q"):
                rclpy.shutdown()  # Shutdown ROS2
                break

if __name__ == "__main__":
    from loguru import logger

    rclpy.init()
    image_publisher = ImagePublisher()

    with logger.catch():
        run()
