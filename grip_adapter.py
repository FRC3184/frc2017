from grip import GripPipeline
import cv2
import time
import math


CAMERA_WIDTH = 320
CAMERA_ANGULAR_WIDTH = 61
CAMERA_ANGLUAR_HEIGHT = 34.3
CAMERA_HEIGHT = 240
FOCUS_LEN = 333.82


def millis():
    return int(round(time.time() * 1000))

_filter = None

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
    #print("Found {} contours".format(len(found_contours)))
    if len(found_contours) in (2, 3):
        contours = list(sorted([Contour(n) for n in found_contours], key=lambda n:n.x))
        tallest = list(sorted(contours, key=lambda n: n.height))
        peg = PegTarget(tallest[0], tallest[-1])
        print("Angle: {}*".format(peg.angle))
        print("Distance: {} in".format(peg.dist))
    return img

def init_filter():
    global _filter
    _filter = GripPipeline()
    print(_filter)
    return do_process

if __name__ == '__main__':
    cap = cv2.VideoCapture("http://localhost:8080/?action=stream&type=.mjpg")
    init_filter()
    print(_filter)
    last_millis = millis()
    count = 0
    fps_sum = 0
    val = True
    while val:
        val, frame = cap.read()
        do_process(frame)
        time_now = millis()
        fps = 1/((time_now - last_millis)/1000)
        fps_sum += fps
        #print("{} FPS".format(fps))
        last_millis = time_now
        count += 1

