#include <SoftwareSerial.h>

// HC-12 connected to Nano pins 10 (RX) and 11 (TX)
SoftwareSerial hc12(10, 11); // hc12 RX, hc12 TX

// Stick input pins
const uint8_t THROTTLE_PIN = A0;
const uint8_t ROLL_PIN     = A1;
const uint8_t PITCH_PIN    = A2;
const uint8_t YAW_PIN      = A3;

// Output pulse range (microseconds)
const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;

// Send rate (milliseconds)
const unsigned long SEND_INTERVAL = 20;
unsigned long lastSend = 0;

void setup() {
  // Initialize HC-12 link
  hc12.begin(9600);
  
  // Configure analog inputs
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(ROLL_PIN,     INPUT);
  pinMode(PITCH_PIN,    INPUT);
  pinMode(YAW_PIN,      INPUT);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    // Read sticks (0–1023)
    int rawThr = analogRead(THROTTLE_PIN);
    int rawRol = analogRead(ROLL_PIN);
    int rawPit = analogRead(PITCH_PIN);
    int rawYaw = analogRead(YAW_PIN);

    // Map to 1000–2000
    int thr = map(rawThr, 0, 1023, PULSE_MIN, PULSE_MAX);
    int rol = map(rawRol, 0, 1023, PULSE_MIN, PULSE_MAX);
    int pit = map(rawPit, 0, 1023, PULSE_MIN, PULSE_MAX);
    int yaw = map(rawYaw, 0, 1023, PULSE_MIN, PULSE_MAX);

    // Compose packet: throttle,roll,pitch,yaw\n
    hc12.print(thr);
    hc12.print(',');
    hc12.print(rol);
    hc12.print(',');
    hc12.print(pit);
    hc12.print(',');
    hc12.print(yaw);
    hc12.print('\n');
  }
}
