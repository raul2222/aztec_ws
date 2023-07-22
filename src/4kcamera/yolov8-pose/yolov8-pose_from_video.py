# coding=utf-8
from pathlib import Path
from time import monotonic
from datetime import datetime

import cv2
import depthai as dai
import numpy as np
from utils import non_max_suppression, plot_skeleton_kpts

ROOT = Path(__file__).parent

videoPath = "/home/mulong/codes/models/kapao/res/squash_inference_kapao_s_coco.gif"

nnWidth, nnHeight = 320, 320


def create_pipeline():
    global nnWidth, nnHeight

    blob = ROOT.joinpath("yolov8n-pose-sh8.blob")

    # blob = "/home/mulong/codes/models/yolov8/yolov8n-pose-sh8.blob"
    model = dai.OpenVINO.Blob(blob)
    dim = next(iter(model.networkInputs.values())).dims
    nnWidth, nnHeight = dim[:2]

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define sources and outputs
    detectionNetwork = pipeline.create(dai.node.NeuralNetwork)
    xinFrame = pipeline.create(dai.node.XLinkIn)
    xoutNN = pipeline.create(dai.node.XLinkOut)

    xinFrame.setStreamName("inFrame")
    xoutNN.setStreamName("detections")

    # Network specific settings
    detectionNetwork.setBlob(model)

    # Linking
    xinFrame.out.link(detectionNetwork.input)
    detectionNetwork.out.link(xoutNN.input)

    return pipeline


def run():
    # Connect to device and start pipeline
    with dai.Device(create_pipeline()) as device:
        # Output queues will be used to get the rgb frames and nn data from the outputs defined above
        inFrameQueue = device.getInputQueue(name="inFrame")
        detectQueue = device.getOutputQueue(name="detections")

        frame = None
        detections = []

        cap = cv2.VideoCapture(videoPath)

        def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
            return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

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

            height, width = frame.shape[:2]

            for detection in detections:
                *bbox, conf, cls = detection[:6]
                bbox = (
                        np.array(bbox)
                        / [nnWidth, nnHeight, nnWidth, nnHeight]
                        * [width, height, width, height]
                )
                bbox = bbox.astype(int)
                kpts = (
                        np.array(detection[6:]).reshape(-1, 3)
                        / [nnWidth, nnHeight, 1]
                        * [width, height, 1]
                ).ravel()

                if cls == 0:
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
            ROOT.joinpath(f"result_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4").as_posix(),
            cv2.VideoWriter_fourcc(*"mp4v"),
            cap.get(cv2.CAP_PROP_FPS),
            (
                int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            ),
        )
        while not device.isClosed() and cap.isOpened():
            read_correctly, frame = cap.read()
            if not read_correctly:
                break

            img = dai.ImgFrame()
            img.setData(to_planar(frame, (nnWidth, nnHeight)))
            img.setTimestamp(monotonic())
            img.setWidth(nnWidth)
            img.setHeight(nnHeight)
            inFrameQueue.send(img)

            detectQueueData = detectQueue.get()

            if detectQueueData is not None:
                pred = toTensorResult(detectQueueData)["output0"]
                detections = non_max_suppression(pred, 0.5, nc=80)[0]

            if frame is not None:
                frame = displayFrame("image", frame)
                vid_writer.write(frame)

            if cv2.waitKey(1) == ord("q"):
                vid_writer.release()
                break


if __name__ == "__main__":
    from loguru import logger

    with logger.catch():
        run()
