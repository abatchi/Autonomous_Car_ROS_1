#include <Servo.h>
#include <SoftwareSerial.h>

// Define pins
#define ESC_PIN 9      // ESC (BLDC motor)
#define SERVO_PIN 6    // Servo motor
#define BT_RX 10       // Bluetooth RX
#define BT_TX 11       // Bluetooth TX

Servo esc;    // ESC control
Servo steer;  // Servo control
SoftwareSerial BT(BT_RX, BT_TX);  // Bluetooth Serial communication

// ESC throttle range
const int minThrottle = 1000;  // Minimum ESC signal
const int maxThrottle = 2000;  // Maximum ESC signal
int throttle = 1500;           // Neutral throttle

// Servo steering range
const int minSteer = 45;   // Leftmost position
const int maxSteer = 135;  // Rightmost position
int steering = 90;         // Center position

void setup() {
    Serial.begin(9600);     // Debugging
    BT.begin(9600);         // Bluetooth communication
    esc.attach(ESC_PIN);
    steer.attach(SERVO_PIN);

    // Initialize ESC with neutral signal
    esc.writeMicroseconds(throttle);
    delay(2000);
}

void loop() {
    if (BT.available()) {
        char command = BT.read();
        Serial.println(command);  // Debugging

        // Acceleration Control (R2 Button Increases Throttle)
        if (command == 'R') {  
            throttle += 50;  // Increase speed
            if (throttle > maxThrottle) throttle = maxThrottle;
        }
        if (command == 'L') {  
            throttle -= 50;  // Decrease speed (braking)
            if (throttle < minThrottle) throttle = minThrottle;
        }

        // Steering Control (Joystick Left/Right)
        if (command == 'L') steering -= 5;  // Turn Left
        if (command == 'R') steering += 5;  // Turn Right
        if (steering > maxSteer) steering = maxSteer;
        if (steering < minSteer) steering = minSteer;

        // Apply control to motors
        esc.writeMicroseconds(throttle);
        steer.write(steering);
    }
    delay(20);
}
