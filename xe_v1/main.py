from ultralytics import YOLO
import cv2
from motor_control import set_motor_speed, set_servo_angle
from PID_controller import PID

model = YOLO('yolov8n.pt')

pid = PID(Kp = 0.1, Ki = 0.1, Kd = 0.1)

cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)
''' | Property ID | Description                                                                       |
    | ----------- | --------------------------------------------------------------------------------- |
    | `0`         | `CV_CAP_PROP_POS_MSEC` : Current position of the video file in milliseconds       |
    | `1`         | `CV_CAP_PROP_POS_FRAMES` : Index of the next frame to be captured                 |
    | `2`         | `CV_CAP_PROP_POS_AVI_RATIO` : Relative position of the video file: 0=start, 1=end |
    | `3`         | `CV_CAP_PROP_FRAME_WIDTH` : Width of the frames in the video stream               |
    | `4`         | `CV_CAP_PROP_FRAME_HEIGHT` : Height of the frames in the video stream             |
'''
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_center = frame_width // 2
base_speed = 20

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)[0]

person_indices = [i for i, c in enumerate(results.boxes.cls) if model.names[int(c)] == "person"]

if person_indices:
    largest = max(person_indices, key=lambda i: results.boxes.xyxy[i][2] - results.boxes.xyxy[i][0])
    x1, y1, x2, y2 = results.boxes.xyxy[largest]
    object_center = int((x1 + x2) / 2)

    error = frame_center - object_center
    correction = pid.update(error)

    angle = 90 + correction
    set_servo_angle(angle)
    set_motor_speed(10)  

else:
    set_motor_speed(0)