#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>

#define PIEZO_PIN 8
#define CALIBRATION_STILL_DURATION 10000
#define ON_TARGET_THRESHOLD 5.0
#define OFF_TARGET_THRESHOLD 15.0
#define WARMUP_SAMPLES 200

Adafruit_MMA8451 mma = Adafruit_MMA8451();
sensors_event_t event;

float referencePitch = 0.0, referenceRoll = 0.0;
float smoothedPitch = 0.0, smoothedRoll = 0.0;
float pitchBuffer[100], rollBuffer[100];
int bufferIndex = 0;
int sampleCount = 0;
bool isCalibrated = false;
bool isOnTarget = false;
bool wasOnTarget = false;
unsigned long lastCalibrationTime = 0;
unsigned long stillnessStartTime = 0;
bool stillnessDetected = false;

void calculateAngles(float ax, float ay, float az, float *pitch, float *roll) {
  *pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  *roll = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
}

float applyEMA(float newValue, float previousValue, float alpha) {
  return (alpha * newValue) + ((1 - alpha) * previousValue);
}

float calculateVariance(float buffer[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  float mean = sum / size;
  float variance = 0;
  for (int i = 0; i < size; i++) {
    float diff = buffer[i] - mean;
    variance += diff * diff;
  }
  return variance / size;
}

void playDing() {
  tone(PIEZO_PIN, 880, 100);
}

void playOffTargetTone(float deviation) {
  int freq = 200 + (int)(deviation * 20);
  freq = constrain(freq, 200, 600);
  tone(PIEZO_PIN, freq);
}

void stopTone() {
  noTone(PIEZO_PIN);
}

bool checkStill() {
  if (bufferIndex < 100) return false;
  
  float pitchVar = calculateVariance(pitchBuffer, 100);
  float rollVar = calculateVariance(rollBuffer, 100);
  
  return (pitchVar < 1.0 && rollVar < 1.0);
}

void performCalibration() {
  referencePitch = smoothedPitch;
  referenceRoll = smoothedRoll;
  isCalibrated = true;
  lastCalibrationTime = millis();
  
  for (int i = 0; i < 3; i++) {
    tone(PIEZO_PIN, 880, 150);
    delay(200);
  }
  
  Serial.println("{\"CALIBRATED\":1,\"PITCH\":0.00,\"ROLL\":0.00}");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  pinMode(PIEZO_PIN, OUTPUT);
  
  Serial.println("{\"STATUS\":\"STARTING\"}");
  
  if (!mma.begin()) {
    Serial.println("{\"ERROR\":\"SENSOR_NOT_FOUND\"}");
    while (1) delay(1000);
  }
  
  mma.setRange(MMA8451_RANGE_2_G);
  
  mma.getEvent(&event);
  calculateAngles(event.acceleration.x, event.acceleration.y, event.acceleration.z, &smoothedPitch, &smoothedRoll);
  
  referencePitch = smoothedPitch;
  referenceRoll = smoothedRoll;
  
  for (int i = 0; i < 100; i++) {
    pitchBuffer[i] = smoothedPitch;
    rollBuffer[i] = smoothedRoll;
  }
  
  Serial.println("{\"STATUS\":\"READY\"}");
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CAL") {
      performCalibration();
    } else if (cmd == "RESET") {
      stillnessDetected = false;
      isCalibrated = false;
      sampleCount = 0;
      lastCalibrationTime = 0;
      bufferIndex = 0;
      for (int i = 0; i < 100; i++) {
        pitchBuffer[i] = smoothedPitch;
        rollBuffer[i] = smoothedRoll;
      }
      Serial.println("{\"STATUS\":\"RESET\"}");
    }
  }
  
  mma.getEvent(&event);
  
  float rawPitch, rawRoll;
  calculateAngles(event.acceleration.x, event.acceleration.y, event.acceleration.z, &rawPitch, &rawRoll);
  
  smoothedPitch = applyEMA(rawPitch, smoothedPitch, 0.15);
  smoothedRoll = applyEMA(rawRoll, smoothedRoll, 0.15);
  
  sampleCount++;
  
  if (sampleCount > WARMUP_SAMPLES) {
    pitchBuffer[bufferIndex] = smoothedPitch;
    rollBuffer[bufferIndex] = smoothedRoll;
    bufferIndex = (bufferIndex + 1) % 100;
  }
  
  float pitchDev = smoothedPitch - referencePitch;
  float rollDev = smoothedRoll - referenceRoll;
  float deviation = sqrt(pitchDev * pitchDev + rollDev * rollDev);
  
  wasOnTarget = isOnTarget;
  
  if (deviation < ON_TARGET_THRESHOLD) {
    isOnTarget = true;
    if (!wasOnTarget) {
      playDing();
    }
    stopTone();
  } else if (deviation > OFF_TARGET_THRESHOLD) {
    isOnTarget = false;
    playOffTargetTone(deviation);
  } else {
    isOnTarget = false;
    stopTone();
  }
  
  if (!isCalibrated && sampleCount > WARMUP_SAMPLES) {
    if (checkStill()) {
      if (!stillnessDetected) {
        stillnessStartTime = millis();
        stillnessDetected = true;
      } else if (millis() - stillnessStartTime >= CALIBRATION_STILL_DURATION) {
        performCalibration();
        stillnessDetected = false;
      }
    } else {
      stillnessDetected = false;
    }
  }
  
  Serial.print("{\"P\":");
  Serial.print(smoothedPitch, 2);
  Serial.print(",\"R\":");
  Serial.print(smoothedRoll, 2);
  Serial.print(",\"PD\":");
  Serial.print(pitchDev, 2);
  Serial.print(",\"RD\":");
  Serial.print(rollDev, 2);
  Serial.print(",\"DEV\":");
  Serial.print(deviation, 2);
  Serial.print(",\"ONTARGET\":");
  Serial.print(isOnTarget ? 1 : 0);
  Serial.print(",\"CAL\":");
  Serial.print(isCalibrated ? 1 : 0);
  Serial.println("}");
  
  delay(50);
}
