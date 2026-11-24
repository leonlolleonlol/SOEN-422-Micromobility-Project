// Viktor Dimitrov

// This code is about TFMini Single Point LiDAR with ESP32 TTGO, MPU-6050 IMU and GPS Neo-7m

// The objective of this part of the project is to be able to detect potholes using the LiDAR
// and send the GPS coordinates to our server over hotspot through the ESP32-TTGO microcontroller.
// The IMU (gyroscope) rejects false positives caused by bumps: when the vehicle pitches nose-up,
// the LiDAR beam lands further away and the distance increases even though there is no pothole.
//
// Rule used when the distance rises into the pothole range:
//   pitch change  >  +BUMP_PITCH_DEG  -> bump (vehicle tilted nose-up), rejected
//   pitch change  <= +BUMP_PITCH_DEG  -> pothole (vehicle level or tilting down)
//   no pitch data                     -> skipped (never assumed to be a pothole)

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ------------------------------------------------------------ settings ------------------------------------------------------------
#define HISTORY_SIZE 21

const int HEADER = 0x59;                 // TFMini frame header byte
const uint8_t  BASE_SAMPLES = 51;        // valid readings used to measure the floor at startup (odd -> single median)
const uint16_t MIN_RISE  = 8;            // ~5 cm depth x 1.6
const uint16_t MAX_RISE  = 32;           // ~20 cm depth x 1.6
const uint8_t  N_CONSECUTIVE = 3;        // readings in a row needed to call it a pothole
const int buzzerPin = 25;                // free pin on the TTGO

// IMU settings
const int   PITCH_AXIS = 1;              // 0 = x, 1 = y, 2 = z  -> find with the axis test
const float PITCH_SIGN = -1.0;           // your test: lifting the stick gave a NEGATIVE pitch, so flip it
const float BUMP_PITCH_DEG = 1.0;        // pitch rise (deg) that means "going over a bump"
const unsigned long PITCH_SAMPLE_MS = 10;     // pitch is stored every 10 ms, independent of the LiDAR
const unsigned long PITCH_LOOKBACK_MS = 400;  // compare with the pitch 0.4 s before the event
const bool DEBUG_PITCH = true;           // true = print pitch with each reading (axis test)

// ------------------------------------------------------------ variables ------------------------------------------------------------
int dist;
int strength;
int check;
int i;
int uart[9];

// Circular Buffer definition
typedef struct {
  uint16_t buffer[HISTORY_SIZE];
  uint8_t head;
  uint8_t count;
} HistoryBuffer;

// Pothole variables
HistoryBuffer history;
uint16_t baseDist = 0;                   // floor distance, measured automatically at startup (cm)
bool armed = false;                      // true once the reading has been back near the floor
bool wasPothole = false;
bool buzzing = false;
unsigned long buzzerStart = 0;

// IMU variables
Adafruit_MPU6050 mpu;
bool imuOk = false;
float gyroBias = 0;                      // gyro offset measured at startup (rad/s)
float pitchDeg = 0;                      // relative pitch angle, integrated from the gyro
unsigned long lastImuMicros = 0;

// Pitch history: one sample every PITCH_SAMPLE_MS, NOT tied to valid LiDAR readings
#define PITCH_HIST 64                    // 64 x 10 ms = 0.64 s of pitch history
float pitchHist[PITCH_HIST];
uint8_t pitchHead = 0;
uint8_t pitchCount = 0;
unsigned long lastPitchSampleMs = 0;

// ------------------------------------------------------------ Buffer functions ------------------------------------------------------------
void history_init(HistoryBuffer *hb) {
  hb->head = 0;
  hb->count = 0;
}

void history_push(HistoryBuffer *hb, uint16_t value) {   // always succeeds
  hb->buffer[hb->head] = value;                          // overwrites oldest value when buffer is full
  hb->head = (hb->head + 1) % HISTORY_SIZE;
  if (hb->count < HISTORY_SIZE) hb->count++;
}

bool history_is_full(HistoryBuffer *hb) {
  return hb->count == HISTORY_SIZE;
}

// k = 0 -> newest, 1 -> the one before, ...
bool history_get_recent(HistoryBuffer *hb, uint8_t k, uint16_t *out) {
  if (k >= hb->count) return false;                      // not enough readings yet
  uint8_t idx = (hb->head + HISTORY_SIZE - 1 - k) % HISTORY_SIZE;
  *out = hb->buffer[idx];
  return true;
}

bool detectPothole(HistoryBuffer *hb) {
  for (uint8_t k = 0; k < N_CONSECUTIVE; k++) {
    uint16_t d;
    if (!history_get_recent(hb, k, &d)) return false;    // not enough data yet
    if (d < baseDist + MIN_RISE || d > baseDist + MAX_RISE) return false;
  }
  return true;                                           // last N readings are in pothole range
}

// ------------------------------------------------------------ TFMini functions ------------------------------------------------------------
// Reads one TFMini frame if a full one is waiting. Returns true with dist/strength filled in.
bool readFrame(int *d, int *s) {
  if (Serial2.available() < 9) return false;
  if (Serial2.read() != HEADER) return false;
  if (Serial2.read() != HEADER) return false;
  uart[0] = HEADER;
  uart[1] = HEADER;
  for (i = 2; i < 9; i++) uart[i] = Serial2.read();
  check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
  if (uart[8] != (check & 0xff)) return false;
  *d = uart[2] + uart[3] * 256;
  *s = uart[4] + uart[5] * 256;
  return true;
}

bool isValid(int d, int s) {
  return s >= 100 && s != 65535 && d >= 10;
}

// Measures the floor distance: median of BASE_SAMPLES valid readings (rig must be still, over flat floor)
uint16_t calibrateBaseline() {
  uint16_t samples[BASE_SAMPLES];
  uint8_t n = 0;
  int d, s;
  unsigned long start = millis();
  while (n < BASE_SAMPLES && millis() - start < 5000) {
    if (readFrame(&d, &s) && isValid(d, s)) samples[n++] = d;
  }
  if (n == 0) return 0;                                  // no valid readings: aim at a matte surface
  for (uint8_t a = 1; a < n; a++) {                      // insertion sort, then take the middle value
    uint16_t v = samples[a];
    int b = a - 1;
    while (b >= 0 && samples[b] > v) { samples[b + 1] = samples[b]; b--; }
    samples[b + 1] = v;
  }
  return samples[n / 2];
}

// ------------------------------------------------------------ IMU functions ------------------------------------------------------------
float readPitchRate() {                                  // rad/s on the chosen axis
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float rates[3] = {g.gyro.x, g.gyro.y, g.gyro.z};
  return rates[PITCH_AXIS];
}

void pitch_push(float p) {
  pitchHist[pitchHead] = p;
  pitchHead = (pitchHead + 1) % PITCH_HIST;
  if (pitchCount < PITCH_HIST) pitchCount++;
}

// pitch stored k samples ago (k = 0 -> newest); each sample is PITCH_SAMPLE_MS apart
bool getPitchAgo(uint8_t k, float *out) {
  if (k >= pitchCount) return false;
  uint8_t idx = (pitchHead + PITCH_HIST - 1 - k) % PITCH_HIST;
  *out = pitchHist[idx];
  return true;
}

// Called every loop pass: integrates the gyro (~400 Hz) and stores the pitch every 10 ms
void updatePitch() {
  if (!imuOk) return;

  unsigned long now = micros();
  if (now - lastImuMicros >= 2500) {                     // integrate at ~400 Hz
    float dt = (now - lastImuMicros) / 1e6;
    lastImuMicros = now;
    float rateDeg = (readPitchRate() - gyroBias) * PITCH_SIGN * 57.2958;   // rad/s -> deg/s
    pitchDeg += rateDeg * dt;                            // angle = sum of (rate x time)
  }

  if (millis() - lastPitchSampleMs >= PITCH_SAMPLE_MS) { // store a sample every 10 ms
    lastPitchSampleMs = millis();
    pitch_push(pitchDeg);
  }
}

// ------------------------------------------------------------ initializations ------------------------------------------------------------
void setup() {
  Serial.begin(115200);                                  // USB serial monitor
  Serial2.begin(115200, SERIAL_8N1, 34, 17);             // TFMini-S: RX=34, TX=17
  history_init(&history);

  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND - check wiring (SDA 21, SCL 22). Bump rejection disabled.");
  } else {
    imuOk = true;
    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Wire.setClock(400000);                               // faster I2C reads

    // Gyro calibration: keep the rig still for ~1 s at power-on
    Serial.println("Calibrating gyro - keep still...");
    float sum = 0;
    for (int n = 0; n < 200; n++) {
      sum += readPitchRate();
      delay(5);
    }
    gyroBias = sum / 200;
    lastImuMicros = micros();
    lastPitchSampleMs = millis();
    Serial.println("Gyro calibrated");
  }

  // Floor calibration: keep the rig still, aimed at flat floor
  Serial.println("Measuring floor distance - keep still...");
  baseDist = calibrateBaseline();
  if (baseDist == 0) {
    Serial.println("No valid LiDAR readings - check aim/surface, then press RST");
  } else {
    Serial.printf("Floor distance = %u cm -> pothole range %u to %u cm\n",
                  baseDist, baseDist + MIN_RISE, baseDist + MAX_RISE);
  }
}

// ------------------------------------------------------------ main code ------------------------------------------------------------
void loop() {
  updatePitch();                                         // keep the pitch angle current, every pass

  if (baseDist > 0 && readFrame(&dist, &strength))
  {
    Serial.print("distance = ");
    Serial.print(dist);
    Serial.print('\t');
    Serial.print("strength = ");
    Serial.print(strength);
    if (DEBUG_PITCH) {
      Serial.print('\t');
      Serial.print("pitch = ");
      Serial.print(pitchDeg, 2);
    }
    Serial.print('\n');

    if (isValid(dist, strength)) {
      history_push(&history, dist);

      // A pothole must start from the floor: the reading has to RISE into the range.
      // Coming back DOWN into the range (e.g. lowering the stick from a high reading) is ignored.
      if (dist < baseDist + MIN_RISE) armed = true;        // back near the floor -> ready
      else if (dist > baseDist + MAX_RISE) armed = false;  // too high -> wait until back at floor level
    }

    bool isPothole = detectPothole(&history);
    if (isPothole && !wasPothole && armed) {             // only when a rise from the floor starts
      float pitchBefore;
      uint8_t lookback = PITCH_LOOKBACK_MS / PITCH_SAMPLE_MS;
      bool havePitch = imuOk && getPitchAgo(lookback, &pitchBefore);

      if (!imuOk) {
        Serial.println("Skipped: IMU not running, cannot tell bump from pothole");
      } else if (!havePitch) {
        Serial.println("Skipped: not enough pitch history yet");
      } else {
        float pitchChange = pitchDeg - pitchBefore;    // + = nose up
        if (pitchChange > BUMP_PITCH_DEG) {
          // distance rose because the vehicle tilted nose-up: bump, not a pothole
          Serial.printf("Rejected: bump (pitch %+.1f deg)\n", pitchChange);
        } else {
          Serial.printf("Pothole detected (pitch %+.1f deg)\n", pitchChange);
          tone(buzzerPin, 1000);
          buzzing = true;
          buzzerStart = millis();
        }
      }
    }
    wasPothole = isPothole;
  }

  if (buzzing && millis() - buzzerStart > 300) {         // 300 ms beep, non-blocking
    noTone(buzzerPin);
    buzzing = false;
  }
} // end of void loop()
