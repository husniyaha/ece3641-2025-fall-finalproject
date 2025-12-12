import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
ik = ik_transform.ArmIK()

## 3. Pre-calibrated Servo Angles
# Row 1-15: Different positions for gripping pen and tapping keys 1-6
servo_positions = {
    1: {'servo1': 800, 'servo2': 472, 'servo3': 109, 'servo4': 198, 'servo5': 223, 'servo6': 548},   # Grip pen
    2: {'servo1': 800, 'servo2': 472, 'servo3': 119, 'servo4': 199, 'servo5': 125, 'servo6': 555},   # Move to key 1
    3: {'servo1': 800, 'servo2': 472, 'servo3': 337, 'servo4': 419, 'servo5': 127, 'servo6': 571},   # Press key 1 (DOWN)
    4: {'servo1': 800, 'servo2': 472, 'servo3': 333, 'servo4': 336, 'servo5': 223, 'servo6': 566},   # Lift from key 1
    5: {'servo1': 800, 'servo2': 472, 'servo3': 263, 'servo4': 372, 'servo5': 124, 'servo6': 500},   # Move to key 2
    6: {'servo1': 800, 'servo2': 472, 'servo3': 263, 'servo4': 419, 'servo5': 124, 'servo6': 510},   # Press key 2 (DOWN)
    7: {'servo1': 800, 'servo2': 472, 'servo3': 216, 'servo4': 249, 'servo5': 223, 'servo6': 510},   # Lift from key 2
    8: {'servo1': 800, 'servo2': 472, 'servo3': 244, 'servo4': 375, 'servo5': 123, 'servo6': 470},   # Move to key 3
    9: {'servo1': 800, 'servo2': 472, 'servo3': 243, 'servo4': 419, 'servo5': 125, 'servo6': 470},   # Press key 3 (DOWN)
    10: {'servo1': 800, 'servo2': 472, 'servo3': 244, 'servo4': 382, 'servo5': 223, 'servo6': 470},  # Lift from key 3
    11: {'servo1': 800, 'servo2': 472, 'servo3': 243, 'servo4': 252, 'servo5': 125, 'servo6': 450},  # Move to key 4
    12: {'servo1': 800, 'servo2': 472, 'servo3': 245, 'servo4': 419, 'servo5': 123, 'servo6': 450},  # Press key 4 (DOWN)
    13: {'servo1': 800, 'servo2': 472, 'servo3': 243, 'servo4': 254, 'servo5': 223, 'servo6': 450},  # Lift from key 4
    14: {'servo1': 800, 'servo2': 472, 'servo3': 244, 'servo4': 381, 'servo5': 123, 'servo6': 430},  # Move to key 5
    15: {'servo1': 800, 'servo2': 472, 'servo3': 240, 'servo4': 419, 'servo5': 125, 'servo6': 430},  # Press key 5 (DOWN)
}

## 4. Stop Function

def stop():
    # Return arm to safe neutral position
    target = servo_positions[1]  # Use grip position as safe state
    bus_servo_control.set_servos(joints_pub, 1500, ((1, target['servo1']), (2, target['servo2']), 
                    (3, target['servo3']), (4, target['servo4']), (5, target['servo5']), (6, target['servo6'])))

## 5. Move to Position Function

def move_to_position(position_number, duration=500):
    """
    Move arm to a pre-calibrated position
    
    Args:
        position_number: 1-15, corresponds to row in servo_positions
        duration: movement duration in milliseconds
    """
    if position_number not in servo_positions:
        print(f"Position {position_number} not found!")
        return False
    
    servo_data = servo_positions[position_number]
    bus_servo_control.set_servos(joints_pub, duration, ((1, servo_data['servo1']), (2, servo_data['servo2']), 
                    (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
    return True

## 6. Main Function

if __name__=='__main__':
    rospy.init_node('Robot_Typing', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(1.0)

    # TEST MODE - Uncomment to test servo ranges
    TEST_MODE = False
    
    if TEST_MODE:
        print("Testing servo3 LEFT/RIGHT range...")
        test_servo3_values = [100, 150, 200, 250, 300, 350, 400, 450]
        
        for val in test_servo3_values:
            print(f"Testing servo3 = {val}")
            test_pos = {'servo1': 800, 'servo2': 472, 'servo3': val, 'servo4': 300, 'servo5': 125, 'servo6': 555}
            bus_servo_control.set_servos(joints_pub, 1000, ((1, test_pos['servo1']), (2, test_pos['servo2']), 
                            (3, test_pos['servo3']), (4, test_pos['servo4']), (5, test_pos['servo5']), (6, test_pos['servo6'])))
            time.sleep(2.0)
        
        print("\nTesting servo4 UP/DOWN range...")
        test_servo4_values = [100, 150, 200, 250, 300, 350, 400, 450]
        
        for val in test_servo4_values:
            print(f"Testing servo4 = {val}")
            test_pos = {'servo1': 800, 'servo2': 472, 'servo3': 300, 'servo4': val, 'servo5': 125, 'servo6': 555}
            bus_servo_control.set_servos(joints_pub, 1000, ((1, test_pos['servo1']), (2, test_pos['servo2']), 
                            (3, test_pos['servo3']), (4, test_pos['servo4']), (5, test_pos['servo5']), (6, test_pos['servo6'])))
            time.sleep(2.0)
    
    else:
        # STAGE 1: Move to pen position (gripper open)
        print("Moving to pen position...")
        move_to_position(1, duration=1000)
        time.sleep(1.0)
        
        # STAGE 2: Close gripper to grip pen
        print("Gripping pen...")
        move_to_position(2, duration=1000)
        time.sleep(1.0)

        # STAGE 3: Press Key 1
        print("Pressing Key 1...")
        move_to_position(3, duration=1000)  # Move to key 1
        time.sleep(0.5)
        move_to_position(4, duration=1000)  # Press down
        time.sleep(0.3)
        move_to_position(5, duration=1000)  # Lift up
        time.sleep(0.5)

        # STAGE 4: Press Key 2
        print("Pressing Key 2...")
        move_to_position(6, duration=1000)  # Move to key 2
        time.sleep(0.5)
        move_to_position(7, duration=1000)  # Press down
        time.sleep(0.3)
        move_to_position(8, duration=1000)  # Lift up
        time.sleep(0.5)

        # STAGE 5: Press Key 3
        print("Pressing Key 3...")
        move_to_position(8, duration=1000)  # Move to key 3
        time.sleep(0.5)
        move_to_position(9, duration=1000)  # Press down
        time.sleep(0.3)
        move_to_position(10, duration=1000)  # Lift up
        time.sleep(0.5)

        # STAGE 6: Press Key 4
        print("Pressing Key 4...")
        move_to_position(12, duration=1000)  # Move to key 4
        time.sleep(0.5)
        move_to_position(13, duration=1000)  # Press down
        time.sleep(0.3)
        move_to_position(14, duration=1000)  # Lift up
        time.sleep(0.5)

        # STAGE 7: Press Key 5
        print("Pressing Key 5...")
        move_to_position(15, duration=1000)  # Move to key 5
        time.sleep(0.5)
        time.sleep(1.0)
	


        # STAGE 8: Release pen and return to neutral
        print("Releasing pen...")
        move_to_position(2, duration=1000)  # Return to grip position (neutral)
        time.sleep(2.0)

        print("Typing task complete!")
