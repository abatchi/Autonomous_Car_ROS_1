#include <Servo.h>

Servo myServo; // Create Servo object
Servo esc;  // Create ESC object
String command = "";

void setup() {
  Serial.begin(9600);
  myServo.attach(9);  // Attach to pin D9
  esc.attach(10);
  myServo.write(120); // Center position
  Serial.println("Arming ESC...");
    
  // **Step 1: Arm the ESC (Throttle = 1000µs)**
  esc.writeMicroseconds(1000);  // Minimum throttle (ESC Off)
  delay(3000);                  // Wait 3 seconds
  Serial.println("ESC Armed!");

}

void loop() {
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    if (command == "LEFT") {
      myServo.write(90);
      esc.writeMicroseconds(1000);
    } 
    else if (command == "RIGHT") {
      myServo.write(140);
      esc.writeMicroseconds(1000);
    } 
    else if (command == "CENTER") {
      myServo.write(120);
      esc.writeMicroseconds(1000);
    }

    else if (command == "LEFTMOVE"){
      myServo.write(90);
      esc.writeMicroseconds(1500);

    }

    else if (command == "RIGHTMOVE"){
      myServo.write(140);
      esc.writeMicroseconds(1500);

    }

    else if(command == "MOVE"){
      myServo.write(120);
      esc.writeMicroseconds(1500);
    }

    else{
      esc.writeMicroseconds(1000);
    }
  }
}
