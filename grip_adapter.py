from grip import GripPipeline
import cv2
import time
from networktables import NetworkTables


# This file runs on the Beaglebone, also running mjpg-streamer


CAMERA_WIDTH = 320
CAMERA_ANGULAR_WIDTH = 61
CAMERA_ANGLUAR_HEIGHT = 34.3
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


class PegTarget:
    def __init__(self, left, right):
        self.center_x_px = (right.x + left.x) / 2 - CAMERA_WIDTH / 2
        self.width_px = right.x + right.width - left.x
        self.height_px = max(left.height, right.height)

        self.dist = FOCUS_LEN * 5 / self.height_px
        self.angle = self.center_x_px * CAMERA_ANGULAR_WIDTH / CAMERA_WIDTH


def do_process(img):
    found_contours = _filter.process(img)
    if len(found_contours) in (2, 3):
        contours = list(sorted([Contour(n) for n in found_contours], key=lambda n:n.x))
        # TODO pair up contours more accurately
        peg = PegTarget(contours[0], contours[-1])
        _vision_table.putNumber("angle", peg.angle)
        _vision_table.putNumber("distance", peg.dist)
    return img


def init_filter():
    global _filter
    _filter = GripPipeline()
    return do_process

if __name__ == '__main__':
    global _vision_table
    print("VISION STARTING")
    NetworkTables.initialize("roborio-3184-frc.local")
    _vision_table = NetworkTables.getTable("vision")
    cap = cv2.VideoCapture("http://localhost:8080/?action=stream&type=.mjpg")
    init_filter()
    last_millis = millis()
    val = True
    print("VISION READY")
    while val:
        val, frame = cap.read()
        do_process(frame)
        time_now = millis()
        fps = 1/((time_now - last_millis)/1000)
        vision_table.putNumber("fps", fps)
        last_millis = time_now

