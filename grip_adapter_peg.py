import math
from grip import GripPipeline
import cv2
import time
from networktables import NetworkTables


# This file runs on the Beaglebone, also running mjpg-streamer


CAMERA_WIDTH = 320
CAMERA_ANGULAR_WIDTH = 53 * math.pi / 180
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
        global _vision_table
        pix_far_left = left.x
        pix_mid_right = right.x
        pix_far_right = right.x + right.width

        # Angular size
        a1 = CAMERA_ANGULAR_WIDTH * (1/2 - pix_far_left / CAMERA_WIDTH)
        a3 = CAMERA_ANGULAR_WIDTH * (1/2 - pix_mid_right / CAMERA_WIDTH)
        a4 = CAMERA_ANGULAR_WIDTH * (1/2 - pix_far_right / CAMERA_WIDTH)

        # Angle from horizontal to wall
        c1 = 10.25 / math.sin(a1 - a4)
        c2 = 8.25 / math.sin(a1 - a3)
        c3 = (c1**2 + c2**2 - 2*c1*c2 * math.cos(a4 - a3))**0.5
        aw = 2 * math.atan((c1 * math.cos(a4) - c2 * math.cos(a3) - c3) / (c1 * math.sin(a4) - c2 * math.sin(a3)))

        # Side distances
        d1 = 10.25 * math.sin(aw + a4) / math.sin(a1 - a4)
        d4 = 10.25 * math.sin(math.pi - aw - a1) / math.sin(a1 - a4)

        # Distance
        dp = ((d4**2 + d1**2)/2 + 5.125**2)**0.5

        _vision_table.putNumber("dp", dp)
        _vision_table.putNumber("aw", aw * 180 / math.pi)
        _vision_table.putNumber("d1", d1)
        _vision_table.putNumber("d4", d4)
        _vision_table.putNumber("a1", a1 * 180 / math.pi)
        _vision_table.putNumber("a3", a3 * 180 / math.pi)
        _vision_table.putNumber("a4", a4 * 180 / math.pi)

        ap = math.asin((d1 * math.sin(a1) - 5.125 * math.sin(aw)) / dp)
        apw = ap + aw

        _vision_table.putNumber("ap", ap * 180 / math.pi)
        _vision_table.putNumber("apw", apw * 180 / math.pi)

        self.init_turn = math.atan2(dp * math.cos(ap) - 24 * math.sin(aw), dp * math.sin(ap) - 24 * math.cos(aw))
        self.travel_dist = (dp**2 + 24**2 - 2 * 24 * dp * math.sin(apw))**0.5
        self.final_turn = math.pi / 2 - aw - self.init_turn


def do_process(img):
    found_contours = _filter.process(img)
    if len(found_contours) in (2, 3):
        contours = list(sorted([Contour(n) for n in found_contours], key=lambda n: n.x))
        # TODO pair up contours more accurately

        peg = PegTarget(contours[0], contours[-1])
        _vision_table.putNumber("init_turn", peg.init_turn)
        _vision_table.putNumber("final_turn", peg.final_turn)
        _vision_table.putNumber("distance", peg.travel_dist)
    return img


def init_filter():
    global _filter
    _filter = GripPipeline()
    return do_process

if __name__ == '__main__':
    global _vision_table
    print("VISION STARTING")
    #NetworkTables.initialize("roborio-3184-frc.local")
    NetworkTables.initialize("192.168.7.1")
    _vision_table = NetworkTables.getTable("vision")
    cap = cv2.VideoCapture("http://localhost:8080/?action=stream&type=.mjpg")
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

