#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MMA8451 mma = Adafruit_MMA8451();
sensors_event_t event;

float smoothedPitch = 0.0, smoothedRoll = 0.0;

void calculateAngles(float ax, float ay, float az, float *pitch, float *roll) {
  *pitch = atan2(ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  *roll = atan2(ay, sqrt(ax * ax + az * az)) * RAD_TO_DEG;
}

float applyEMA(float newValue, float previousValue, float alpha) {
  return (alpha * newValue) + ((1 - alpha) * previousValue);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("{\"STATUS\":\"STARTING\"}");
  
  if (!mma.begin()) {
    Serial.println("{\"ERROR\":\"SENSOR_NOT_FOUND\"}");
    while (1) delay(1000);
  }
  
  mma.setRange(MMA8451_RANGE_2_G);
  
  mma.getEvent(&event);
  calculateAngles(event.acceleration.x, event.acceleration.y, event.acceleration.z, &smoothedPitch, &smoothedRoll);
  
  Serial.println("{\"STATUS\":\"READY\"}");
}

void loop() {
  mma.getEvent(&event);
  
  float rawPitch, rawRoll;
  calculateAngles(event.acceleration.x, event.acceleration.y, event.acceleration.z, &rawPitch, &rawRoll);
  
  smoothedPitch = applyEMA(rawPitch, smoothedPitch, 0.15);
  smoothedRoll = applyEMA(rawRoll, smoothedRoll, 0.15);
  
  Serial.print("{\"P\":");
  Serial.print(smoothedPitch, 2);
  Serial.print(",\"R\":");
  Serial.print(smoothedRoll, 2);
  Serial.println("}");
  
  delay(16);
}
