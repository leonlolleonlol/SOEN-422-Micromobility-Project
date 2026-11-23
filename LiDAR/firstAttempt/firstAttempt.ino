// Viktor Dimitrov - SOEN422

// This code is about TFMini Single Point LiDAR with ESP32 TTGO and GPS Neo-7m

// The objective of this part of the project is to be able to detect potholes using the LiDAR
// and send the GPS coordinates to our server over hotspot through the ESP32-TTGO microcontroller

//#include <SoftwareSerial.h>   //header file of software serial port
//SoftwareSerial Serial1(2, 3); //define software serial port name as Serial1 and define pin2 as RX & pin3 as TX
 
// ------------------------------------------------------------ variables ------------------------------------------------------------
#define HISTORY_SIZE 21
int dist;
int strength;
int check;
int i;
int uart[9];
const int HEADER = 0x59;
const uint16_t BASE_DIST = 151;  // measure on flat floor with your mount
const uint16_t MIN_RISE  = 8;    // ~3 cm depth × 1.6
const uint16_t MAX_RISE  = 32;   // ~20 cm depth × 1.6
const uint8_t  N_CONSECUTIVE  = 3;
const int buzzerPin = 25;        // a free pin on your TTGO

// Circular Buffer definition
typedef struct{
  uint16_t buffer[HISTORY_SIZE];
  uint8_t head;
  uint8_t count;
} HistoryBuffer;

// Pothole Variables used
HistoryBuffer history;
bool isPothole = false;
unsigned long previousMillis = 0;
bool wasPothole = false;
bool buzzing = false;
unsigned long buzzerStart = 0;

// ------------------------------------------------------------ Buffer functions ------------------------------------------------------------
void history_init(HistoryBuffer *cb) {
    cb->head = 0;
    cb->count = 0;
}

void history_push(HistoryBuffer *hb, uint16_t value) {   // always succeeds
    hb->buffer[hb->head] = value;                        // overwrites oldest value when buffer is full
    hb->head = (hb->head + 1) % HISTORY_SIZE;
    if (hb->count < HISTORY_SIZE) hb->count++;
}

bool history_is_full(HistoryBuffer *hb) {
    return hb->count == HISTORY_SIZE;
}

bool history_get_recent (HistoryBuffer *hb, uint8_t k, uint16_t *out) {
  
  if (k >= hb->count) return false;
  uint8_t idx = (hb->head + HISTORY_SIZE - 1 - k) % HISTORY_SIZE;
  *out = hb->buffer[idx];
  return true;
}

bool detectPothole (HistoryBuffer *hb) {

  for (uint8_t k = 0; k < N_CONSECUTIVE; k++) {
    uint16_t d;
    if (!history_get_recent(hb, k, &d)) return false; // not enough data yet
    if (d < BASE_DIST + MIN_RISE || d > BASE_DIST + MAX_RISE) return false;
  }
  return true; // last N measurements are in the range to consider it a pothole!
} 

// ------------------------------------------------------------ initializations ------------------------------------------------------------
void setup() {
  Serial.begin(115200);                       // fix 2: USB serial monitor
  Serial2.begin(115200, SERIAL_8N1, 34, 17);  // TFMini-S: RX=34, TX=17
  history_init(&history);
}

// ------------------------------------------------------------ main code ------------------------------------------------------------
void loop() {
  if (Serial2.available() >= 9)               // fix 1 + 3: right port, full frame waiting
  {
    if (Serial2.read() == HEADER)
    {
      uart[0] = HEADER;
      if (Serial2.read() == HEADER)
      {
        uart[1] = HEADER;
        for (i = 2; i < 9; i++)
        {
          uart[i] = Serial2.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff))
        {
          dist = uart[2] + uart[3] * 256;
          strength = uart[4] + uart[5] * 256;
          Serial.print("distance = ");
          Serial.print(dist);
          Serial.print('\t');
          Serial.print("strength = ");
          Serial.print(strength);
          Serial.print('\n');

          if (strength >= 100 && strength != 65535 && dist >= 10) {
          history_push(&history, dist);
          }

          bool isPothole = detectPothole(&history);
          if (isPothole && !wasPothole) {             // only when a pothole starts
            Serial.println("Pothole detected");
            tone(buzzerPin, 1000);
            buzzing = true;
            buzzerStart = millis();
          }
          wasPothole = isPothole;
          // detectPothole (&history, isPothole);
          // if (isPothole) {

          //   Serial.println("Pothole Detected. Buzzer sound.");
          //   unsigned long currentMillis = millis(); // current time

          //   while (currentMillis - previousMillis > 1000) {
          //     tone(buzzerPin, 1000);
          //     previousMillis = currentMillis;
          //   }
          //   noTone(buzzerPin);
          // }
        } // end of if uart[8]
      }
    }
  }
  if (buzzing && millis() - buzzerStart > 300) {   // 300 ms beep, non-blocking
    noTone(buzzerPin);
    buzzing = false;
  }
} // end of void loop()


/*
IMU Example Code:

#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float GyroX, GyroY, GyroZ;

void setup() {
  Serial.begin(115200);
  
  // 1. Initialize I2C on your custom pins
  Wire.begin(32, 33); 
  
  // 2. Wake up the MPU6050 (it starts in sleep mode by default)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // Set to 0 to wake up
  Wire.endTransmission(true);
  
  delay(100);
}

void loop() {
  // Point to the first Gyroscope data register (0x43)
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  
  // Request 6 bytes (X_H, X_L, Y_H, Y_L, Z_H, Z_L)
  Wire.requestFrom(MPU, 6, true);
  
  // Cast to (int16_t) so negative rotation values work on 32-bit ESP32!
  GyroX = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 131.0;

  // Print rotational speed in degrees/second
  Serial.print("Gyro X: "); Serial.print(GyroX);
  Serial.print(" | Y: ");   Serial.print(GyroY);
  Serial.print(" | Z: ");   Serial.println(GyroZ);
  
  delay(100);
}

*/