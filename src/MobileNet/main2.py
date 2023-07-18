import depthai as dai
import numpy as np
import cv2

# Labels for the emotions that the network can recognize
emotions = ['neutral', 'happy', 'sad', 'surprise', 'anger']

# Start defining a pipeline
pipeline = dai.Pipeline()

# Define a source - color camera
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)
cam_rgb.setInterleaved(False)

# Create face detection neural network
nn_face_detection = pipeline.createMobileNetDetectionNetwork()
nn_face_detection.setBlobPath("face-detection-retail-0004.blob")
nn_face_detection.setConfidenceThreshold(0.5)
cam_rgb.preview.link(nn_face_detection.input)

# Create emotions recognition neural network, linked to the output of the face detection network
nn_emotions = pipeline.createNeuralNetwork()
nn_emotions.setBlobPath("emotions-recognition-retail-0003.blob")
nn_face_detection.out.link(nn_emotions.input)

# Create outputs
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

xout_nn_face = pipeline.createXLinkOut()
xout_nn_face.setStreamName("nn_face")
nn_face_detection.out.link(xout_nn_face.input)

xout_nn_emotions = pipeline.createXLinkOut()
xout_nn_emotions.setStreamName("nn_emotions")
nn_emotions.out.link(xout_nn_emotions.input)

# Pipeline defined, now the device is connected to
with dai.Device(pipeline) as device:
    # Start pipeline
    device.startPipeline()

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    q_nn_face = device.getOutputQueue(name="nn_face", maxSize=4, blocking=False)
    q_nn_emotions = device.getOutputQueue(name="nn_emotions", maxSize=4, blocking=False)

    frame = None
    faces = []
    emotions = []

    while True:
        # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        in_rgb = q_rgb.tryGet()
        in_nn_face = q_nn_face.tryGet()
        in_nn_emotions = q_nn_emotions.tryGet()

        # If a new frame is available, convert it to an OpenCV frame
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

        # If new neural network data is available from the face detection network, save detections
        if in_nn_face is not None:
            faces = in_nn_face.detections

        # If new neural network data is available from the emotions recognition network, save detections
        if in_nn_emotions is not None:
            emotions = np.array(in_nn_emotions.getFirstLayerFp16())

        # If the frame is available, draw bounding boxes and emotion names on it and show the frame
        if frame is not None:
            for face, emotion_results in zip(faces, emotions):
                bbox = face.getBoundingBoxCoords()
                emotion_name = emotions[np.argmax(emotion_results)]
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
                cv2.putText(frame, emotion_name, (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow("rgb", frame)

        # Break the loop if 'q' key was pressed
        if cv2.waitKey(1) == ord('q'):
            break
