
Overview
1. ROS Noetic Node (Python): Publishes velocity commands to an Arduino via serial communication.
2. Arduino Firmware (C++): Reads the commands and controls the motors accordingly.

---

## 1. ROS Noetic Node (Python)
This script runs on your Ubuntu machine, sending motor commands to the Arduino via serial.

📂 Save this as `motor_control.py` in `~/catkin_ws/src/two_motor_car/src/`:

```python
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
```

---

## 2. Arduino Firmware (C++)
This script runs on your Arduino, reading serial commands and controlling the motors.

📂 Save this as `motor_control.ino` and upload it to your Arduino:

```cpp
#define LEFT_MOTOR_FORWARD 9
#define LEFT_MOTOR_BACKWARD 10
#define RIGHT_MOTOR_FORWARD 5
#define RIGHT_MOTOR_BACKWARD 6

void setup() {
    Serial.begin(9600);
    pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
    pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
    pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
    stopMotors();
}

void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        switch (command) {
            case 'F': moveForward(); break;
            case 'B': moveBackward(); break;
            case 'L': turnLeft(); break;
            case 'R': turnRight(); break;
            case 'S': stopMotors(); break;
        }
    }
}

void moveForward() {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void moveBackward() {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void turnLeft() {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_FORWARD, HIGH);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}

void turnRight() {
    digitalWrite(LEFT_MOTOR_FORWARD, HIGH);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, HIGH);
}

void stopMotors() {
    digitalWrite(LEFT_MOTOR_FORWARD, LOW);
    digitalWrite(LEFT_MOTOR_BACKWARD, LOW);
    digitalWrite(RIGHT_MOTOR_FORWARD, LOW);
    digitalWrite(RIGHT_MOTOR_BACKWARD, LOW);
}
```

---

## 3. ROS Launch File
📂 Save this as `motor.launch` in `~/catkin_ws/src/two_motor_car/launch/`:

```xml
<launch>
    <node pkg="two_motor_car" type="motor_control.py" name="motor_controller" output="screen"/>
</launch>
```

---

## 4. Setup and Run**
### On Ubuntu (ROS)
1. Build the package:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

2. Check Serial Port** (Find the Arduino port):
   ```bash
   ls /dev/tty*
   ```
   Update `SERIAL_PORT` in `motor_control.py` accordingly.

3. Run the ROS node:
   ```bash
   roslaunch two_motor_car motor.launch
   ```

4. Send Commands via ROS:
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear:
     x: 0.5
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.0"
   ```

### Summary
✅ ROS Noetic sends movement commands to Arduino.  
✅ Arduino interprets the commands and drives the motors.  
✅ Uses Serial Communication between Ubuntu and Arduino.  


