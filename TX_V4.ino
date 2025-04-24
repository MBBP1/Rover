#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED setup
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Joystick pins
#define RX A0
#define RY A1
#define LX A2
#define LY A3

// Knapper
#define BUTTON1 2  // Skifter mellem Crane / Car
#define BUTTON2 5  // (valgfri fremtidig funktion)

RF24 radio(7, 8);
const byte address[6] = "00010";

// Data Structs
struct ControlData {
  int rxVal;
  int ryVal;
  int lxVal;
  int lyVal;
  bool mode;
  bool mode2;
};

struct AckPayload {
  float temperature;
};

ControlData data;
AckPayload ack;

bool lastButton1 = HIGH;
bool lastButton2 = HIGH;

unsigned long lastSend = 0;
const unsigned long sendInterval = 20;

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Ready");
  display.display();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.enableAckPayload();
  radio.stopListening();

  Serial.println("TX klar - Crane + Car");
  radio.printDetails();  // Debug
}

void loop() {
  if (millis() - lastSend >= sendInterval) {
    lastSend = millis();

    // Læs joysticks
    data.rxVal = analogRead(RX);
    data.ryVal = analogRead(RY);
    data.lxVal = analogRead(LX);
    data.lyVal = analogRead(LY);

    // MODE toggle (Crane/Car)
    bool currentButton1 = digitalRead(BUTTON1);
    if (currentButton1 == LOW && lastButton1 == HIGH) {
      data.mode = !data.mode;
      delay(300); // Debounce
    }
    lastButton1 = currentButton1;

    // MODE2 (NOT USED)
    bool currentButton2 = digitalRead(BUTTON2);
    if (currentButton2 == LOW && lastButton2 == HIGH) {
      data.mode2 = !data.mode2;
      delay(300);
    }
    lastButton2 = currentButton2;

    // Send data til RX
    radio.write(&data, sizeof(data));

    // Modtag temperatur
    if (radio.isAckPayloadAvailable()) {
      radio.read(&ack, sizeof(ack));
    }

    // === OLED Display ===
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print("Mode: ");
    display.setTextSize(2);
    display.setCursor(60, 0);
    if (data.mode) {
      display.println("CRANE");
    } else {
      display.println("CAR");
    }

    display.setTextSize(2);
    display.setCursor(0, 40);
    display.print("Temp:");
    display.print(ack.temperature, 1);
    display.print("C");
    display.display();

    // === Debug Serial ===
    Serial.print("MODE: "); Serial.print(data.mode ? "CRANE" : "CAR");
    Serial.print(" | RX: "); Serial.print(data.rxVal);
    Serial.print(" | RY: "); Serial.print(data.ryVal);
    Serial.print(" | LX: "); Serial.print(data.lxVal);
    Serial.print(" | LY: "); Serial.print(data.lyVal);
    Serial.print(" | TEMP: "); Serial.println(ack.temperature, 1);
  }
}