#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <math.h>

// CONFIGURATION
#define DEBUG_MODE false
#define USE_STATUS_LED true
#define STATUS_LED_PIN 13
#define GREEN_LED_PIN 7
#define RED_LED_PIN 9
#define PIEZO_PIN 8
#define SHOOT_BUTTON_PIN 6
#define HORIZONTAL_SERVO_PIN 10
#define VERTICAL_SERVO_PIN 11
#define PITCH_SMOOTHING_FACTOR 0.2
#define ROLL_SMOOTHING_FACTOR 0.2
#define SERIAL_OUTPUT_INTERVAL 50
#define BUZZER_MIN_FREQ 200
#define BUZZER_MAX_FREQ 2000
#define BUZZER_RANGE_MIN 0.5
#define BUZZER_RANGE_MAX 2.0
#define DEBOUNCE_DELAY 300

// OBJECTS
Adafruit_MMA8451 mma = Adafruit_MMA8451();
sensors_event_t event;
Servo horizontalServo;
Servo verticalServo;

// VARIABLES
float referencePitch = 0.0, referenceRoll = 0.0;
float smoothedPitch = 0.0, smoothedRoll = 0.0;
float currentHorizontalPos = 90, currentVerticalPos = 90;

bool isCalibrated = false;
float buzzerFrequency = 440;
bool buzzerEnabled = false;

unsigned long lastShootTime = 0;
bool lastButtonState = HIGH;

void calculateAngles(float ax, float ay, float az, float *pitch, float *roll) {
  *pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  *roll = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
}

float applyEMA(float newValue, float previousValue, float alpha) {
  return (alpha * newValue) + ((1 - alpha) * previousValue);
}

void updateLEDs(bool calibrated) {
  if (calibrated) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
  }
}

void playShootSound() {
  tone(PIEZO_PIN, 150, 100);
  delay(100);
  tone(PIEZO_PIN, 200, 150);
}

void playCalibrationTone() {
  tone(PIEZO_PIN, 880, 100);
  delay(150);
  tone(PIEZO_PIN, 1100, 100);
}

void outputSerialData(float pitch, float roll) {
  static unsigned long lastSerialOutput = 0;
  if (millis() - lastSerialOutput < SERIAL_OUTPUT_INTERVAL) return;
  lastSerialOutput = millis();
  
  Serial.print("{");
  Serial.print("\"P\":"); Serial.print(pitch, 2);
  Serial.print(",\"R\":"); Serial.print(roll, 2);
  Serial.print(",\"CAL\":"); Serial.print(isCalibrated ? 1 : 0);
  Serial.print(",\"HP\":"); Serial.print((int)currentHorizontalPos);
  Serial.print(",\"VP\":"); Serial.print((int)currentVerticalPos);
  Serial.println("}");
}

void checkShootButton() {
  bool buttonState = digitalRead(SHOOT_BUTTON_PIN);
  
  if (buttonState == LOW && lastButtonState == HIGH) {
    if (millis() - lastShootTime > DEBOUNCE_DELAY) {
      lastShootTime = millis();
      playShootSound();
      Serial.println("{\"SHOOT\":1}");
      #if DEBUG_MODE
      Serial.println("SHOOT!");
      #endif
    }
  }
  
  lastButtonState = buttonState;
}

void handleSerialCommand() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "SHOOT") {
      playShootSound();
      Serial.println("{\"SHOOT\":1}");
    } else if (command == "PING") {
      Serial.println("{\"PONG\":1}");
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(PIEZO_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(SHOOT_BUTTON_PIN, INPUT_PULLUP);
  
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  #if DEBUG_MODE
  Serial.println("=== ARCHERY SENSOR v4.0 ===");
  #endif
  
  if (!mma.begin()) {
    #if DEBUG_MODE
    Serial.println("ERROR: MMA8451 not found!");
    #endif
    while (1) {
      digitalWrite(STATUS_LED_PIN, HIGH);
      delay(500);
      digitalWrite(STATUS_LED_PIN, LOW);
      delay(500);
    }
  }
  #if DEBUG_MODE
  Serial.println("MMA8451 initialized");
  #endif
  mma.setRange(MMA8451_RANGE_2_G);
  
  horizontalServo.attach(HORIZONTAL_SERVO_PIN);
  verticalServo.attach(VERTICAL_SERVO_PIN);
  horizontalServo.write(90);
  verticalServo.write(90);
  
  mma.getEvent(&event);
  calculateAngles(event.acceleration.x, event.acceleration.y, event.acceleration.z, &smoothedPitch, &smoothedRoll);
  referencePitch = smoothedPitch;
  referenceRoll = smoothedRoll;
  
  isCalibrated = true;
  updateLEDs(true);
  playCalibrationTone();
  
  #if DEBUG_MODE
  Serial.println("CALIBRATED - Ready!");
  Serial.print("Reference Pitch: "); Serial.println(referencePitch, 2);
  Serial.print("Reference Roll: "); Serial.println(referenceRoll, 2);
  #endif
  
  Serial.println("{\"STATUS\":\"READY\"}");
}

void loop() {
  handleSerialCommand();
  checkShootButton();
  
  mma.getEvent(&event);
  
  float rawPitch, rawRoll;
  calculateAngles(event.acceleration.x, event.acceleration.y, event.acceleration.z, &rawPitch, &rawRoll);
  
  smoothedPitch = applyEMA(rawPitch, smoothedPitch, PITCH_SMOOTHING_FACTOR);
  smoothedRoll = applyEMA(rawRoll, smoothedRoll, ROLL_SMOOTHING_FACTOR);
  
  float pitchDev = smoothedPitch - referencePitch;
  float rollDev = smoothedRoll - referenceRoll;
  
  currentHorizontalPos = 90 + (rollDev * 2.5);
  currentVerticalPos = 90 - (pitchDev * 2.5);
  
  currentHorizontalPos = constrain(currentHorizontalPos, 60, 120);
  currentVerticalPos = constrain(currentVerticalPos, 60, 120);
  
  horizontalServo.write((int)currentHorizontalPos);
  verticalServo.write((int)currentVerticalPos);
  
  float magnitude = sqrt(pitchDev * pitchDev + rollDev * rollDev);
  if (magnitude < BUZZER_RANGE_MIN || magnitude > BUZZER_RANGE_MAX) {
    buzzerEnabled = true;
    buzzerFrequency = map(magnitude * 1000, 500, 2000, BUZZER_MIN_FREQ, BUZZER_MAX_FREQ);
    buzzerFrequency = constrain(buzzerFrequency, BUZZER_MIN_FREQ, BUZZER_MAX_FREQ);
    tone(PIEZO_PIN, (int)buzzerFrequency);
  } else {
    buzzerEnabled = false;
    noTone(PIEZO_PIN);
  }
  
  outputSerialData(smoothedPitch, smoothedRoll);
  
  delay(20);
}