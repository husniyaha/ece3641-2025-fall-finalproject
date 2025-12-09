#!/usr/bin/env python3
# encoding:utf-8

import sys
import time
import rospy

from armpi_pro_kinematics.kinematics import ik_transform
import armpi_fpv.bus_servo_control as bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = ik_transform.ArmIK()


def stop():
    """
    Default safe position of the robot arm (similar to your stop()).
    """
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -90, -180, -90)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500,
            ((1, 200), (2, 500),
             (3, servo_data['servo3']),
             (4, servo_data['servo4']),
             (5, servo_data['servo5']),
             (6, servo_data['servo6'])))


def main():
    global joints_pub

    rospy.init_node('pen_pickup', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur',
                                 MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1.0)

    # ---------------------------
    # 1) Move arm to a neutral pose in front of the base
    # ---------------------------
    # Similar to your first move in the water pouring script, but closer in.
    rospy.loginfo("Moving to neutral pose before pen pickup...")
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500,
            ((1, 200), (2, 500),
             (3, servo_data['servo3']),
             (4, servo_data['servo4']),
             (5, servo_data['servo5']),
             (6, servo_data['servo6'])))
    time.sleep(1.0)

    # ---------------------------
    # 2) Open claws wide to accept pen
    # ---------------------------
    # Reuse your “open claws for bottle” idea:
    rospy.loginfo("Opening gripper wide for pen insertion...")
    target = ik.setPitchRanges((0.0, 0.0, 0.40), -90, 270, -270)
    if target:
        servo_data = target[1]
        # (1, 0) = rotate wrist; (2, 500) = gripper neutral; adjust if needed
        bus_servo_control.set_servos(joints_pub, 1500,
            ((1, 0), (2, 500),
             (3, servo_data['servo3']),
             (4, servo_data['servo4']),
             (5, servo_data['servo5']),
             (6, servo_data['servo6'])))
    time.sleep(1.0)

    # Same position but with different pitch range to ensure a valid IK solution
    target = ik.setPitchRanges((0.0, 0.0, 0.40), -90, 0, -270)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500,
            ((1, 0), (2, 500),
             (3, servo_data['servo3']),
             (4, servo_data['servo4']),
             (5, servo_data['servo5']),
             (6, servo_data['servo6'])))
    time.sleep(2.0)

    rospy.loginfo("Insert the pen between the claws now, then press Enter...")
    input()

    # ---------------------------
    # 3) Close claws on the pen
    # ---------------------------
    # Use your pattern: (1, 1000) (2, 0) closes more aggressively.
    rospy.loginfo("Closing gripper to hold pen...")
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500,
            ((1, 1000), (2, 0),
             (3, servo_data['servo3']),
             (4, servo_data['servo4']),
             (5, servo_data['servo5']),
             (6, servo_data['servo6'])))
    time.sleep(1.5)

    # ---------------------------
    # 4) Move to keyboard hover pose with pen held
    # ---------------------------
    # Adjust these XYZs to be above the center of your keyboard.
    rospy.loginfo("Moving to keyboard hover pose with pen held...")

    keyboard_hover_xyz = (0.0, 0.20, 0.15)  # tune to your keyboard position
    hover_alpha = -90

    target = ik.setPitchRanges(keyboard_hover_xyz, hover_alpha, -180, 0)
    if not target:
        rospy.logerr("IK failed for keyboard hover pose at {}".format(keyboard_hover_xyz))
        return

    servo_data = target[1]
    bus_servo_control.set_servos(joints_pub, 1500,
        ((1, 200), (2, 0),
         (3, servo_data['servo3']),
         (4, servo_data['servo4']),
         (5, servo_data['servo5']),
         (6, servo_data['servo6'])))
    time.sleep(2.0)

    rospy.loginfo("Pen pickup + hover complete. Arm is hovering over keyboard with pen held.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass