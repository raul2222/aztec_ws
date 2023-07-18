from depthai_sdk import OakCamera
import cv2

def main():
    with OakCamera() as oak:
        color = oak.create_camera('color')

        oak.start(blocking=False)

        while True:
            packet = oak.get_packet()
            if packet is not None and packet.stream_name == 'color':
                cv2.imwrite('test_image.jpg', packet.getCvFrame())
                break

if __name__ == '__main__':
    main()
