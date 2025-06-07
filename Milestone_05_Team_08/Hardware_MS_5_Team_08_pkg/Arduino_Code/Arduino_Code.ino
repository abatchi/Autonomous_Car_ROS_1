#include <Wire.h>
#include <Servo.h>

// === Servo & ESC ===
Servo steeringServo;
Servo esc;
const int servoPin = 10;
const int escPin = 9;


// === Servo limits (customized zero point and limits) ===
const int servoCenter = 120;
const int servoLeftLimit = 90;
const int servoRightLimit = 140;

// === ESC PWM range ===
const int minPulse = 1000;    // ESC min pulse (µs)
const int maxPulse = 1050;    // ESC max pulse (µs)

// === MPU6050 Yaw ===
long gyro_z_cal = 0;
int16_t gyro_z;
double dt;
double gyroYaw = 0;
const float GYRO_SCALE = 65.5;

// === Encoder ===
const int encoderPin = 2;
volatile long encoderTicks = 0;
unsigned long lastEncoderSend = 0;
const int ticksPerRev = 85;
const unsigned long encoderSendInterval = 100; // ms

// === Timing ===
unsigned long prevTime;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // ESC setup
  esc.attach(escPin);
  for (int i = 0; i < 50; i++) {
    esc.writeMicroseconds(minPulse);
    delay(20);
  }

  // Servo setup
  steeringServo.attach(servoPin);
  steeringServo.write(servoCenter);

  // MPU6050 setup
  setup_mpu_6050_registers();
  calibrateGyro();
  prevTime = micros();

  // Encoder setup
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
}

void loop() {
  // === Serial input ===
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.indexOf(',') != -1) {
      int sep = input.indexOf(',');
      String angleStr = input.substring(0, sep);
      String throttleStr = input.substring(sep + 1);

      int angle = angleStr.toInt();
      float throttle = throttleStr.toFloat();

      // Clamp and write angle
      if (angle >= servoLeftLimit && angle <= servoRightLimit)
        steeringServo.write(angle);

      // Convert throttle [0.0 - 1.0] to pulse width
      throttle = constrain(throttle, 0.0, 1.0);
      int pulseWidth = minPulse + (int)(throttle * (maxPulse - minPulse));
      esc.writeMicroseconds(pulseWidth);
    }
  }

  // === Yaw computation ===
  dt = (micros() - prevTime) * 1e-6;
  prevTime = micros();

  read_gyro_z();
  gyro_z -= gyro_z_cal;
  double gyroRateZ = (double)gyro_z / GYRO_SCALE;
  gyroYaw += gyroRateZ * dt;

  // === Send yaw and encoder every 100ms ===
  unsigned long now = millis();
  if (now - lastEncoderSend >= encoderSendInterval) {
  lastEncoderSend = now;
    
  // if this doesn't work comment it and uncomment the one below it and try it
  noInterrupts();
    long ticks = encoderTicks;
    float rpm = (encoderTicks * 600.0) / 85.0;
    encoderTicks = 0;
  interrupts();

  // noInterrupts();
  // long ticks = encoderTicks;
  // encoderTicks = 0;
  // interrupts();

  // float rpm = (ticks * 600.0) / ticksPerRev;


  

  Serial.print(rpm, 2); // Send revolutions (2 decimal places)
  Serial.print(",");
  Serial.println(gyroYaw * -1);
}


  // Optional: maintain ~250 Hz loop
  while (micros() - prevTime < 4000);
}

// === MPU6050 Functions ===

void calibrateGyro() {
  for (int i = 0; i < 3000; i++) {
    read_gyro_z();
    gyro_z_cal += gyro_z;
    delay(3);
  }
  gyro_z_cal /= 3000;
}

void read_gyro_z() {
  Wire.beginTransmission(0x68);
  Wire.write(0x47); // Gyro Z register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 2);
  gyro_z = Wire.read() << 8 | Wire.read();
}

void setup_mpu_6050_registers() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08); // ±500°/s sensitivity
  Wire.endTransmission();
}

// === Encoder ISR ===
void encoderISR() {
  encoderTicks++;
}
