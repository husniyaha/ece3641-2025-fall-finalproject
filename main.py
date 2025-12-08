#import sys
#import time
#import rospy
#from kinematics import ik_transform
#from armpi_pro import bus_servo_control

#!/usr/bin/python3
# coding=utf8
"""
Robot typing using RoboArm Pi Pro camera + IK + bus_servo_control.

Features:
 - Automatic 4-point calibration (robot moves to known XY points).
 - User clicks pen tip in camera frames for each calibration pose.
 - Computes pixel -> robot (X,Y) mapping.
 - Computes pixel -> robot (X,Y) mapping
 - Detects keyboard image and lets user click keys for letters h,e,l,o.
 - Types user-entered phrase using those mapped coordinates.


"""

import cv2
import json
import numpy as np
import rospy
import time
import sys
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print("Please run with python3")
    sys.exit(1)

# =========================
# User-editable parameters
# =========================
CAM_IDX = 0  # camera index for cv2.VideoCapture
CALIB_ROBOT_POINTS = [
    # Four points (x, y, z) in robot workspace (meters).
    # These should form a rectangle over the keyboard area.
    # Adjust if necessary for your robot/keyboard placement.
    (0.08, 0.18, 0.04),
    (0.18, 0.18, 0.04),
    (0.08, 0.28, 0.04),
    (0.18, 0.28, 0.04),
]
CALIB_ABOVE_OFFSET = 0.06  # move above target by this when showing pen in frame (m)
TAP_DEPTH = 0.015  # how deep (m) to press down from key surface
TAP_DURATION = 0.6  # seconds for each servo move
SERVO_MOVE_DURATION_MS = 1200  # duration for set_servos (ms)

ALLOWED_CHARS = set(["h", "e", "l", "o"])
COORDS_SAVE_FILE = "keyboard_coords.json"

# =========================
# Robot helpers (IK + servos)
# =========================
ik = ik_transform.ArmIK()

def init_ros_node():
    rospy.init_node('robot_typing_vision', anonymous=True, log_level=rospy.INFO)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur',
                                 MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1.0)
    return joints_pub

def move_to_xyz(joints_pub, x, y, z, pitch=-90, roll=-180, yaw=-90, duration_ms=SERVO_MOVE_DURATION_MS):
    """
    Move arm to absolute XYZ using ik.setPitchRanges and bus_servo_control.set_servos.
    Returns True if IK succeeded.
    """
    target = ik.setPitchRanges((x, y, z), pitch, roll, yaw)
    if not target:
        rospy.logwarn("IK failed for (%.3f, %.3f, %.3f)" % (x, y, z))
        return False
    servo_data = target[1]
    bus_servo_control.set_servos(joints_pub, duration_ms,
                                 ((1, 500), (2, 500),
                                  (3, servo_data['servo3']),
                                  (4, servo_data['servo4']),
                                  (5, servo_data['servos']),
                                  (6, servo_data['servo6'])))
    time.sleep(duration_ms/1000.0 + 0.1)
    return True

def tap_at_xy(joints_pub, x, y, z_surface):
    """Move above key, go to surface, press down TAP_DEPTH, then raise back."""
    # Move above
    if not move_to_xyz(joints_pub, x, y, z_surface + 0.06):
        return
    # Move to surface
    if not move_to_xyz(joints_pub, x, y, z_surface, duration_ms=800):
        return
    # Tap down
    down_z = z_surface - TAP_DEPTH
    if not move_to_xyz(joints_pub, x, y, down_z, duration_ms=600):
        return
    time.sleep(0.08)
    # raise back to surface
    move_to_xyz(joints_pub, x, y, z_surface, duration_ms=600)
    time.sleep(0.12)
    # move back up to safe height
    move_to_xyz(joints_pub, x, y, z_surface + 0.06, duration_ms=800)

# =========================
# Vision / calibration
# =========================
def capture_frame(cap):
    for _ in range(5):
        ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Camera capture failed")
    return frame

# global state for mouse callbacks
mouse_point = None
clicked_points = []

def mouse_click_event(event, x, y, flags, param):
    global mouse_point, clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_point = (x, y)
        clicked_points.append((x, y))
        print("[VIS] Click registered at pixel:", mouse_point)

def calibrate_pixel_to_robot(joints_pub, cap):
    """
    For each robot calibration point:
      - Move robot to the point (above and down)
      - Show camera frame and request user to click the pen tip in image
    After 4 clicks, compute a perspective transform from pixel -> robot XY plane.
    Returns a 3x3 matrix H such that [x,y,1]^T ~ H * [u, v, 1]^T
    """
    global clicked_points
    clicked_points = []

    robot_points = []
    pixel_points = []

    cv2.namedWindow("calib")
    cv2.setMouseCallback("calib", mouse_click_event)

    print("\n=== Calibration routine ===")
    print("The script will move the robot to 4 known XY positions (over the keyboard).")
    print("For each pose: watch the arm/pen in the camera window and LEFT-CLICK on the pen tip.")
    print("Make sure the pen tip is visible. If not, adjust camera or robot placement.\n")
    time.sleep(2.0)

    for idx, robot_pose in enumerate(CALIB_ROBOT_POINTS):
        x, y, z = robot_pose
        print(f"[CALIB] Moving to calibration pose {idx+1}: robot (x={x}, y={y}, z={z})")
        # move above first for camera visibility
        move_to_xyz(joints_pub, x, y, z + CALIB_ABOVE_OFFSET)
        time.sleep(0.6)
        move_to_xyz(joints_pub, x, y, z)
        time.sleep(0.6)

        # show frame and wait for click
        while True:
            frame = capture_frame(cap)
            # draw instruction text
            disp = frame.copy()
            cv2.putText(disp, f"Click PEN TIP for cal point {idx+1}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
            cv2.imshow("calib", disp)
            key = cv2.waitKey(30) & 0xFF
            if mouse_point is not None:
                px = mouse_point
                # record and clear mouse_point for next
                pixel_points.append(px)
                robot_points.append((x, y))   # store XY only
                print(f"[CALIB] Recorded pixel {px} -> robot {(x,y)}")
                # reset for next
                mouse_point = None
                break
            if key == ord('q'):
                raise KeyboardInterrupt("Calibration aborted by user")

    cv2.destroyWindow("calib")

    if len(pixel_points) < 4:
        raise RuntimeError("Calibration incomplete (need 4 points)")

    # compute perspective transform (pixel -> robot XY)
    src = np.array(pixel_points, dtype=np.float32)  # pixels (u,v)
    dst = np.array(robot_points, dtype=np.float32)  # robot XY

    # Add zeros for Z mapping via perspective; we'll compute full 3x3 homography mapping
    H = cv2.getPerspectiveTransform(src, dst)
    # H maps [u,v,1] -> [X,Y,?] in homogeneous coords (we will normalize)
    print("[CALIB] Computed perspective transform matrix H:")
    print(H)
    return H

def pixel_to_robot_xy(H, px):
    """
    Map pixel (u,v) to robot (x,y) using H (3x3 homography).
    px: (u,v)
    """
    uv1 = np.array([ [ [px[0], px[1]] ] ], dtype=np.float32)  # shape (1,1,2)
    mapped = cv2.perspectiveTransform(uv1, H)  # shape (1,1,2)
    x, y = mapped[0,0]
    return float(x), float(y)

# =========================
# Key detection & selection
# =========================
def detect_key_boxes(frame):
    """
    Simple contour-based key box detection.
    Returns list of bounding boxes (x,y,w,h).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    key_boxes = []
    for cnt in contours:
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.03 * peri, True)
        if len(approx) == 4:
            x,y,w,h = cv2.boundingRect(approx)
            if 20 < w < 200 and 20 < h < 200:
                key_boxes.append((x,y,w,h))
    # remove duplicates and sort left-to-right, top-to-bottom
    key_boxes = sorted(key_boxes, key=lambda b: (b[1], b[0]))
    return key_boxes

def select_keys_via_click(frame, required_keys=("h","e","l","o")):
    """
    Show frame and let user click on each desired key in order of required_keys.
    Returns a dict mapping char -> pixel (u,v).
    """
    sel = {}
    cv2.namedWindow("select_keys")
    global mouse_point, clicked_points
    clicked_points = []
    mouse_point = None
    cv2.setMouseCallback("select_keys", mouse_click_event)

    print("\n=== Key selection ===")
    print("For each letter requested, LEFT-CLICK on the center of that key in the image window.")
    print("Order:", required_keys)
    time.sleep(1.0)

    for ch in required_keys:
        print(f"Click key for letter '{ch}' now...")
        while True:
            disp = frame.copy()
            # draw previously selected
            for k, p in sel.items():
                cv2.circle(disp, (int(p[0]), int(p[1])), 6, (0,255,0), -1)
                cv2.putText(disp, k, (int(p[0])+6, int(p[1])+6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0),2)
            cv2.imshow("select_keys", disp)
            key = cv2.waitKey(30) & 0xFF
            if mouse_point is not None:
                px = mouse_point
                sel[ch] = px
                print(f"[SELECT] {ch} -> pixel {px}")
                mouse_point = None
                break
            if key == ord('q'):
                raise KeyboardInterrupt("Selection aborted by user")
    cv2.destroyWindow("select_keys")
    return sel

# =========================
# Main flow
# =========================
def main():
    joints_pub = init_ros_node()
    cap = cv2.VideoCapture(CAM_IDX)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera index %d" % CAM_IDX)

    try:
        # 1) Calibration: move robot and click pen tip
        H = calibrate_pixel_to_robot(joints_pub, cap)

        # 2) Capture a clear image of keyboard
        print("[STEP] Capturing keyboard image for key detection...")
        frame = capture_frame(cap)
        vis = frame.copy()

        # optionally run key detection and draw boxes for user
        boxes = detect_key_boxes(frame)
        for (x,y,w,h) in boxes:
            cv2.rectangle(vis, (x,y), (x+w, y+h), (0,255,0), 2)
            cx, cy = x+w//2, y+h//2
            cv2.circle(vis, (cx,cy), 3, (0,0,255), -1)

        cv2.imwrite("keyboard_detect_preview.jpg", vis)
        print("Saved keyboard_detect_preview.jpg. A window will open for selecting keys now.")
        time.sleep(0.5)

        # 3) Let user click the keys for h,e,l,o
        selected_pixels = select_keys_via_click(frame, required_keys=("h","e","l","o"))

        # 4) Convert selected pixels to robot XY coordinates
        keyboard_robot_coords = {}
        z_surface_guess = CALIB_ROBOT_POINTS[0][2]  # assume same Z for all keys
        for ch, px in selected_pixels.items():
            x_robot, y_robot = pixel_to_robot_xy(H, px)
            keyboard_robot_coords[ch] = (x_robot, y_robot, z_surface_guess)
            print(f"[MAPPED] '{ch}' pixel {px} -> robot (x={x_robot:.3f}, y={y_robot:.3f}, z={z_surface_guess:.3f})")

        # Save to file
        with open(COORDS_SAVE_FILE, "w") as f:
            json.dump(keyboard_robot_coords, f, indent=2)
        print(f"[INFO] Saved keyboard coords to {COORDS_SAVE_FILE}")

        # 5) Demo typing loop
        print("\n=== Typing demo ===")
        print("You may now type a phrase using letters only from:", ALLOWED_CHARS)
        text = input("Enter phrase: ").lower().strip()
        for ch in text:
            if ch not in keyboard_robot_coords:
                print(f"[WARN] Character '{ch}' not mapped/supported; skipping.")
                continue
            x,y,z = keyboard_robot_coords[ch]
            print(f"[TYPE] Pressing '{ch}' at robot (x={x:.3f}, y={y:.3f})")
            tap_at_xy(joints_pub, x, y, z)
            time.sleep(0.25)

        print("[DONE] Typing sequence complete. Moving arm to safe position...")
        # return to a safe/rest position
        move_to_xyz(joints_pub, 0.10, 0.15, 0.08)

    except KeyboardInterrupt:
        print("Aborted by user.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__=='__main__':
    main()


