import argparse
import threading
from pathlib import Path

from depthai_sdk.managers import PipelineManager, NNetManager, BlobManager, PreviewManager
from depthai_sdk import Previews
from depthai_sdk.fps import FPSHandler
from depthai_sdk.utils import toTensorResult, getDeviceInfo
import blobconverter

from pose import getKeypoints, getValidPairs, getPersonwiseKeypoints
import cv2
import depthai as dai
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation_node')
        self.publisher_ = self.create_publisher(Image, 'pose_estimation', 10)
        self.bridge = CvBridge()

    def publish_pose(self, frame):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()

    parser = argparse.ArgumentParser()
    parser.add_argument('-nd', '--no-debug', action="store_true", help="Prevent debug output")
    parser.add_argument('-cam', '--camera', action="store_true", help="Use DepthAI 4K RGB camera for inference (conflicts with -vid)")
    parser.add_argument('-vid', '--video', type=str, help="Path to video file to be used for inference (conflicts with -cam)")
    args = parser.parse_args()

    if not args.camera and not args.video:
        raise RuntimeError("No source selected. Please use either \"-cam\" to use RGB camera as a source or \"-vid <path>\" to run on video")

    debug = not args.no_debug
    device_info = getDeviceInfo()

    if args.camera:
        shaves = 6
    else:
        shaves = 8
        if str(args.video).startswith('https'):
           
            print("Youtube video downloaded.")
        if not Path(args.video).exists():
            raise ValueError("Path {} does not exists!".format(args.video))

    blob_path = blobconverter.from_zoo(name="human-pose-estimation-0001", shaves=shaves)

    colors = [[0, 100, 255], [0, 100, 255], [0, 255, 255], [0, 100, 255], [0, 255, 255], [0, 100, 255], [0, 255, 0],
            [255, 200, 100], [255, 0, 255], [0, 255, 0], [255, 200, 100], [255, 0, 255], [0, 0, 255], [255, 0, 0],
            [200, 200, 0], [255, 0, 0], [200, 200, 0], [0, 0, 0]]
    POSE_PAIRS = [[1, 2], [1, 5], [2, 3], [3, 4], [5, 6], [6, 7], [1, 8], [8, 9], [9, 10], [1, 11], [11, 12], [12, 13],
                [1, 0], [0, 14], [14, 16], [0, 15], [15, 17], [2, 17], [5, 16]]

    running = True
    pose = None
    keypoints_list = None
    detected_keypoints = None
    personwiseKeypoints = None

    nm = NNetManager(inputSize=(456, 256))
    pm = PipelineManager()
    pm.setNnManager(nm)

    if args.camera:
        fps = FPSHandler()
        pm.createColorCam(previewSize=(456, 256), xout=True)
    else:
        cap = cv2.VideoCapture(str(Path(args.video).resolve().absolute()))
        fps = FPSHandler(cap)

    nn = nm.createNN(pm.pipeline, pm.nodes, source=Previews.color.name if args.camera else "host", blobPath=Path(blob_path), fullFov=True)
    pm.addNn(nn=nn)

    with dai.Device(pm.pipeline, device_info) as device:
        if args.camera:
            pv = PreviewManager(display=[Previews.color.name], nnSource=Previews.color.name, fpsHandler=fps)
            pv.createQueues(device)
        nm.createQueues(device)
        seq_num = 1

        def should_run():
            return cap.isOpened() if args.video else True

        try:
            while should_run():
                fps.nextIter()
                if args.camera:
                    pv.prepareFrames()
                    frame = pv.get(Previews.color.name)
                    if debug:
                        node.publish_pose(frame)
                if not args.camera:
                    read_correctly, frame = cap.read()

                    if not read_correctly:
                        break

                    nm.sendInputFrame(frame)
                    fps.tick('host')

                    if debug:
                        node.publish_pose(frame)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    break

        except KeyboardInterrupt:
            pass

        running = False

    fps.printStatus()
    if not args.camera:
        cap.release()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
