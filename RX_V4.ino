#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <DHT.h>

// === Radio Setup ===
RF24 radio(7, 8);
const byte address[6] = "00010";

// === DHT Sensor ===
DHT dhtSensor(45, DHT22);

// === Servoer ===
Servo movementServoLX;   // Base
Servo movementServoRY;   // Shoulder
Servo movementServoRX;   // Elbow
Servo gripperServo;      // Gripper
Servo steeringServo;     // Car steering

// === Pins ===
const int pinLX = 11;
const int pinRY = 12;
const int pinRX = 13;
const int pinGripper = 10;
const int pinSteering = 9;

// === Motorer ===
const int AIA = 5;
const int AIB = 6;
const int BIA = 2;
const int BIB = 3;

// === Vinkelstatus ===
int angleLX = 95, lastAngleLX = 90;
int angleRY = 12, lastAngleRY = 90;
int angleRX = 45, lastAngleRX = 90;
int gripperAngle = 90, lastGripperAngle = 90;
int steeringAngle = 75;

// === Grænser ===
const int angleMin = 0, angleMax = 180;
const int gripperMin = 30, gripperMax = 160;
const int deadZone = 30;

// === Data Structs ===
struct ControlData {
  int rxVal;
  int ryVal;
  int lxVal;
  int lyVal;
  bool mode;
  bool mode2;
};
ControlData data;

struct AckPayload {
  float temperature;
};
AckPayload ack;

// === Timing ===
unsigned long lastSignal = 0;
unsigned long lastUpdate = 0;
unsigned long lastTempSend = 0;
const unsigned long updateInterval = 20;
const unsigned long tempSendInterval = 10000; // 10 sekunder

// === Funktionsprototyper ===
int updateAngleWithDeadzone(int angle, int value, int direction, int minVal = angleMin, int maxVal = angleMax);
void updateServo(Servo& servo, int angle, int& lastAngle);
void driveWithJoystick(int throttle);
void steerWithJoystick(int input);
void STOP();

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  dhtSensor.begin();

  // Servo setup
  movementServoLX.attach(pinLX);
  movementServoRY.attach(pinRY);
  movementServoRX.attach(pinRX);
  gripperServo.attach(pinGripper);
  steeringServo.attach(pinSteering);

  // Motor pins
  pinMode(AIA, OUTPUT);
  pinMode(AIB, OUTPUT);
  pinMode(BIA, OUTPUT);
  pinMode(BIB, OUTPUT);

  // NRF
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();
  radio.startListening();

  Serial.println("RX klar – Crane + Car mode");
  radio.printDetails();
}

void loop() {
  if (millis() - lastUpdate >= updateInterval) {
    lastUpdate = millis();

    if (radio.available()) {
      radio.read(&data, sizeof(data));
      lastSignal = millis();

      float temp = dhtSensor.readTemperature();
      ack.temperature = temp;
      radio.writeAckPayload(1, &ack, sizeof(ack));

      handleInput();
    }

    if (millis() - lastSignal > 2000) {
      Serial.println("⚠️ Ingen signal – STOPPER");
      STOP();
    }
  }

  if (millis() - lastTempSend >= tempSendInterval) {
    lastTempSend = millis();
    Serial1.print("Temp: ");
    Serial1.print(ack.temperature, 1);
    Serial1.println(" C");
  }
}

// Modificer updateAngleWithDeadzone funktionen til at være mere simpel
int updateAngleWithDeadzone(int angle, int value, int direction, int minVal = angleMin, int maxVal = angleMax) {
  if (abs(value - 512) > deadZone) {
    int speed = map(abs(value - 512), deadZone, 512, 1, 10);
    angle += (value > 512) ? speed * direction : -speed * direction;
    angle = constrain(angle, minVal, maxVal);
  }
  return angle;
}

// Fjern lastAngle check i updateServo for at opdatere kontinuerligt
void updateServo(Servo& servo, int angle, int& lastAngle) {
  servo.write(angle);
  lastAngle = angle;
}

// I handleInput(), fjern STOP() i crane mode for at undgå unødvendige stop
void handleInput() {
  int rx = data.rxVal;
  int ry = data.ryVal;
  int lx = data.lxVal;
  int ly = data.lyVal;

  if (!data.mode) {
    // === CAR MODE ===
    driveWithJoystick(ly);       // Frem/bak
    steerWithJoystick(rx);       // Styring
  } else {
    // === CRANE MODE ===
    // Kranens led
    angleLX = updateAngleWithDeadzone(angleLX, lx, -1); // Base
    angleRX = updateAngleWithDeadzone(angleRX, rx, 1);  // Elbow
    angleRY = updateAngleWithDeadzone(angleRY, ry, 1);  // Shoulder
    gripperAngle = updateAngleWithDeadzone(gripperAngle, ly, 1, gripperMin, gripperMax); // Gripper

    // Opdater servoer kontinuerligt
    movementServoLX.write(angleLX);
    movementServoRY.write(angleRY);
    movementServoRX.write(angleRX);
    gripperServo.write(gripperAngle);
  }
  // Debug
  Serial.print("Mode: "); Serial.print(data.mode ? "CRANE" : "CAR");
  Serial.print(" | LX: "); Serial.print(angleLX);
  Serial.print(" | RY: "); Serial.print(angleRY);
  Serial.print(" | RX: "); Serial.print(angleRX);
  Serial.print(" | LY (Gripper): "); Serial.print(gripperAngle);
  Serial.print(" | Temp: "); Serial.println(ack.temperature, 1);
}

void driveWithJoystick(int throttle) {
  int center = 512;
  int dead = 30;

  if (throttle > center + dead) {
    int power = map(throttle, center + dead, 1023, 0, 255);
    analogWrite(AIA, power); analogWrite(AIB, 0);
    analogWrite(BIA, power); analogWrite(BIB, 0);
  } else if (throttle < center - dead) {
    int power = map(throttle, center - dead, 0, 0, 255);
    analogWrite(AIA, 0); analogWrite(AIB, power);
    analogWrite(BIA, 0); analogWrite(BIB, power);
  } else {
    STOP();
  }
}

void steerWithJoystick(int input) {
  int center = 512;
  int dead = 30;
  int target;

  
  if (abs(input - center) < dead) {
    target = 82; 
  } else {
    target = map(input, 0, 1023, 45, 120); 
  }


  if (steeringAngle < target) {
    steeringAngle += 8;
    if (steeringAngle > target) steeringAngle = target;
  } else if (steeringAngle > target) {
    steeringAngle -= 8;
    if (steeringAngle < target) steeringAngle = target;
  }
  steeringServo.write(steeringAngle);
}


void STOP() {
  analogWrite(AIA, 0);
  analogWrite(AIB, 0);
  analogWrite(BIA, 0);
  analogWrite(BIB, 0);
}
