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
