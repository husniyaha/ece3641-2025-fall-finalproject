#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

keyboard_rect = None     # (x, y, w, h)
keyboard_locked = False  # lock keyboard once found
click_point = None       # clicked pixel

# -----------------------------------------------------------
# Mouse callback – store click
# -----------------------------------------------------------
def mouse_callback(event, x, y, flags, param):
    global click_point, keyboard_rect

    if event == cv2.EVENT_LBUTTONDOWN and keyboard_rect is not None:
        (kx, ky, kw, kh) = keyboard_rect

        if kx <= x <= kx + kw and ky <= y <= ky + kh:
            click_point = (x, y)
            print("Clicked pixel:", click_point)

cv2.namedWindow("camera")
cv2.setMouseCallback("camera", mouse_callback)

# -----------------------------------------------------------
# Keyboard detection – detect once, then lock
# -----------------------------------------------------------
def detect_keyboard(frame):

    global keyboard_rect, keyboard_locked

    if keyboard_locked and keyboard_rect is not None:
        (x, y, w, h) = keyboard_rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return frame

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    cnts = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = cnts[1] if len(cnts) == 2 else cnts[1]

    if not contours:
        return frame

    largest = max(contours, key=cv2.contourArea)

    x, y, w, h = cv2.boundingRect(largest)

    if w > 200 and h > 80:
        keyboard_rect = (x, y, w, h)
        keyboard_locked = True
        print("Keyboard locked:", keyboard_rect)

    # draw box
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return frame



# -----------------------------------------------------------
# ROS callback – receive camera frame
# -----------------------------------------------------------
def callback(msg):

    global click_point

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    frame = detect_keyboard(frame)

    if click_point is not None:
        cx, cy = click_point
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

    cv2.imshow("camera", frame)
    cv2.waitKey(1)

def main():

    rospy.init_node("keyboard_click_demo")

    rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    rospy.spin()

if __name__ == "__main__":
    main()
