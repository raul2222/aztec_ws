# coding=utf-8
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
from utils import non_max_suppression, FPSHandler, plot_skeleton_kpts
ROOT = Path(__file__).parent

def create_pipeline():
    blob = ROOT.joinpath("yolov8n-pose.blob")

    # blob = "/home/mulong/codes/models/yolov8/yolov8n-pose.blob"
    model = dai.OpenVINO.Blob(blob)
    dim = next(iter(model.networkInputs.values())).dims
    nnWidth, nnHeight = dim[:2]
    print(f"{nnWidth, nnHeight = }")
    print(f"{blob = }")

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    camRgb = pipeline.create(dai.node.ColorCamera)
    detectionNetwork = pipeline.create(dai.node.NeuralNetwork)
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutNN = pipeline.create(dai.node.XLinkOut)

    xoutRgb.setStreamName("image")
    xoutNN.setStreamName("detections")

    # Properties
    camRgb.setPreviewSize(nnWidth, nnHeight)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(60)
    camRgb.setPreviewKeepAspectRatio(False)

    # Network specific settings
    detectionNetwork.setBlob(model)

    # Linking
    camRgb.preview.link(detectionNetwork.input)
    camRgb.preview.link(xoutRgb.input)
    # detectionNetwork.passthrough.link(xoutRgb.input)
    detectionNetwork.out.link(xoutNN.input)

    return pipeline


def run():
    # Connect to device and start pipeline
    with dai.Device(create_pipeline()) as device:
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        imageQueue = device.getOutputQueue(name="image", maxSize=4, blocking=False)
        detectQueue = device.getOutputQueue(
            name="detections", maxSize=4, blocking=False
        )

        frame = None
        detections = []

        fpsHandler = FPSHandler()

        def drawText(frame, text, org, color=(255, 255, 255)):
            cv2.putText(
                frame,
                text,
                org,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA
            )

        def displayFrame(name, frame):
            color = (255, 0, 0)

            for detection in detections:
                *bbox, conf, cls = detection[:6]
                bbox = np.array(bbox).astype(int)
                kpts = detection[6:]

                # if int(cls) == 0:
                drawText(
                    frame,
                    f"{conf:.2%}",
                    (bbox[0] + 10, bbox[1] + 35),
                )
                cv2.rectangle(
                    frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2
                )
                frame = plot_skeleton_kpts(frame, kpts=kpts, steps=3)

            # Show the frame
            cv2.imshow(name, frame)
            return frame

        def toTensorResult(packet):
            """
            Converts NN packet to dict, with each key being output tensor name and each value being correctly reshaped and converted results array
            Useful as a first step of processing NN results for custom neural networks
            Args:
                packet (depthai.NNData.NNData): Packet returned from NN node
            Returns:
                dict: Dict containing prepared output tensors
            """
            data = {}
            for tensor in packet.getRaw().tensors:
                if tensor.dataType == dai.TensorInfo.DataType.INT:
                    data[tensor.name] = np.array(
                        packet.getLayerInt32(tensor.name)
                    ).reshape(tensor.dims)
                elif tensor.dataType == dai.TensorInfo.DataType.FP16:
                    data[tensor.name] = np.array(
                        packet.getLayerFp16(tensor.name)
                    ).reshape(tensor.dims)
                elif tensor.dataType == dai.TensorInfo.DataType.I8:
                    data[tensor.name] = np.array(
                        packet.getLayerUInt8(tensor.name)
                    ).reshape(tensor.dims)
                else:
                    print("Unsupported tensor layer type: {}".format(tensor.dataType))
            return data

        vid_writer = cv2.VideoWriter(
            
            ROOT.joinpath(f"result_{datetime.now().strftime( '%Y%m%d_%H%M%S' )}.mp4").as_posix(),
            cv2.VideoWriter_fourcc(*"mp4v"),
            20,
            (320, 320),
        )
        bboxColors = np.random.randint(255, size=3)
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
               # vid_writer.write(frame)
                frame = None

            if cv2.waitKey(1) == ord("q"):
                vid_writer.release()
                break


if __name__ == "__main__":
    from loguru import logger

    with logger.catch():
        run()
