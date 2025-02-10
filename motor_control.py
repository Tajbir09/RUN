#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import Twist

# Initialize Serial Communication (Change port as needed)
SERIAL_PORT = "/dev/ttyUSB0"  # Check with `ls /dev/tty*` to find the correct port
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException:
    rospy.logerr("Could not connect to the Arduino. Check the port and connection.")

def cmd_vel_callback(msg):
    """Processes Twist messages and sends motor commands via serial."""
    linear_x = msg.linear.x  # Forward/Backward
    angular_z = msg.angular.z  # Left/Right Turn

    if linear_x > 0:
        command = "F\n"  # Forward
    elif linear_x < 0:
        command = "B\n"  # Backward
    elif angular_z > 0:
        command = "L\n"  # Left Turn
    elif angular_z < 0:
        command = "R\n"  # Right Turn
    else:
        command = "S\n"  # Stop

    if ser.is_open:
        ser.write(command.encode())  # Send command to Arduino

def motor_controller():
    """Initializes the ROS node and subscriber."""
    rospy.init_node("motor_controller", anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        motor_controller()
    except rospy.ROSInterruptException:
        pass
    finally:
        if ser.is_open:
            ser.write("S\n".encode())  # Ensure motors stop
            ser.close()
