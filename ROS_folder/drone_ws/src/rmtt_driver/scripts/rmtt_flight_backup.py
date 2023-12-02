#!/usr/bin/env python3

# Import the library that allows us to read our keyboard
from pynput import keyboard as kb

# Import ROS related libraries
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


def initialize_ros():
    # Initialize the ROS node.
    rospy.init_node('rmtt_flight')

    # Define the publishers
    land_pub = rospy.Publisher('/land', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
    move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Create the message format for the publishers
    empty_msg = Empty()
    twist_msg = Twist()

    return land_pub, takeoff_pub, move_pub, empty_msg, twist_msg

# Function called to indicate the drone's movement with the keyboard
def key_pressed(key, twist_msg, move_pub, takeoff_pub, land_pub, empty_msg):
    #print('Key pressed: ' + str(key))

    if key == kb.Key.space:  # Takeoff
        takeoff_pub.publish(empty_msg)
        #print("takeoff")
    elif key == kb.KeyCode.from_char('0'):  # Landing
        land_pub.publish(empty_msg)
        #print("land")
    elif key == kb.Key.ctrl:  # Finish program
        # Ensure that the drone lands before the program exits.
        land_pub.publish(empty_msg)
        #print("land")
        return False
    else:
        # Drone's directional movements
        handle_movement_keys(key, twist_msg, move_pub)

# Function called to stop the drone's movements when a key is released
def key_released(key, twist_msg, move_pub):
    #print('Key released: ' + str(key))
    stop_single_movement(key, twist_msg, move_pub)

def handle_movement_keys(key, twist_msg, move_pub):
    # Instructions for the drone's directional movements
    if key == kb.Key.up:  # Up
        twist_msg.linear.z = 1
        move_pub.publish(twist_msg)
    elif key == kb.Key.down:  # Down
        twist_msg.linear.z = -1
        move_pub.publish(twist_msg)
    elif key == kb.Key.left:  # Turn left
        twist_msg.angular.z = 1
        move_pub.publish(twist_msg)
    elif key == kb.Key.right:  # Turn right
        twist_msg.angular.z = -1
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('w'):  # Front
        twist_msg.linear.x = 1
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('a'):  # Left
        twist_msg.linear.y = 1
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('d'):  # Right
        twist_msg.linear.y = -1
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('s'):  # Back
        twist_msg.linear.x = -1
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('q'):  # Emergency stop
        stop_all_movement(twist_msg, move_pub)

def stop_all_movement(twist_msg, move_pub):  # Emergency stop
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = 0
    move_pub.publish(twist_msg)
    print("stop")

def stop_single_movement(key, twist_msg, move_pub):
    # Stop individual linear and angular velocity leaving the rest unaltered
    if key == kb.Key.up:  # Up
        twist_msg.linear.z = 0
        move_pub.publish(twist_msg)
    elif key == kb.Key.down:  # Down
        twist_msg.linear.z = 0
        move_pub.publish(twist_msg)
    elif key == kb.Key.left:  # Turn left
        twist_msg.angular.z = 0
        move_pub.publish(twist_msg)
    elif key == kb.Key.right:  # Turn right
        twist_msg.angular.z = 0
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('w'):  # Front
        twist_msg.linear.x = 0
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('a'):  # Left
        twist_msg.linear.y = 0
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('d'):  # Right
        twist_msg.linear.y = 0
        move_pub.publish(twist_msg)
    elif key == kb.KeyCode.from_char('s'):  # Back
        twist_msg.linear.x = 0
        move_pub.publish(twist_msg)

def main():
    land_pub, takeoff_pub, move_pub, empty_msg, twist_msg = initialize_ros()
    print("Ready for instructions")
    listener = kb.Listener(
        on_press=lambda key: key_pressed(key, twist_msg, move_pub, takeoff_pub, land_pub, empty_msg),
        on_release=lambda key: key_released(key, twist_msg, move_pub)
    )
    listener.start()
    listener.join()

if __name__ == "__main__":
    main()