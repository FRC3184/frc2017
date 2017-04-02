import math
from grip import GripPipeline
import cv2
import time
from networktables import NetworkTables


# This file runs on the Beaglebone, also running mjpg-streamer


CAMERA_WIDTH = 320
CAMERA_ANGULAR_WIDTH = 53
CAMERA_ANGULAR_HEIGHT = 34.3
CAMERA_HEIGHT = 240
FOCUS_LEN = 333.82


def millis():
    return int(round(time.time() * 1000))

_filter = None
_vision_table = None


class Contour:
    def __init__(self, contour):
        self.contour = contour
        self.area = cv2.contourArea(contour)
        self.x, self.y, self.width, self.height = cv2.boundingRect(contour)
        self.c_x = self.x + self.width / 2
        self.c_y = self.y + self.height / 2


def do_process(img):
    _filter.process(img)
    contours = _filter.convex_hulls_output
    contours = sorted([Contour(x) for x in contours], key=lambda x: x.area)
    if len(contours) > 0:
        main = contours[-1]  # Largest contour
        angle = ((main.c_x - CAMERA_WIDTH / 2) * CAMERA_ANGULAR_WIDTH / CAMERA_WIDTH) / 2
        _vision_table.putNumber("angle", angle)
        _vision_table.putBoolean("seeBoiler", True)
    else:
        _vision_table.putBoolean("seeBoiler", False)


def init_filter():
    global _filter
    _filter = GripPipeline()
    return do_process

if __name__ == '__main__':
    global _vision_table
    print("VISION STARTING")
    NetworkTables.initialize("roborio-3184-frc.local")
    # NetworkTables.initialize("192.168.7.1")
    _vision_table = NetworkTables.getTable("vision")
    cap = cv2.VideoCapture("http://localhost:1180/?action=stream&type=.mjpg")
    init_filter()
    last_millis = millis()
    val = True
    print("VISION READY")
    _vision_table.putBoolean("ready", True)
    while val:
        val, frame = cap.read()
        do_process(frame)
        time_now = millis()
        fps = 1/((time_now - last_millis)/1000)
        _vision_table.putNumber("fps", fps)
        last_millis = time_now

