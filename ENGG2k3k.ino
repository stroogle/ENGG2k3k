
#include <FastLED.h> 
// #include "Interface.h"
#include "Interface.h"

const int SENSOR_PIN = 1;
const int MOTOR_PIN = 2;
const int SPEAKER_PIN = 3;

SensorControl s1;
SensorControl s2;
SensorControl s3;
MotorControl motor;
SpeakerControl speaker;

void setup() {

  // Let outputs print:
  Serial.begin(9600);

  // Sensor Testing START
  s1 = SensorControl(SENSOR_PIN);
  // Sensor Testing END

  // Motor Testing START
  s2 = SensorControl(SENSOR_PIN);
  motor = MotorControl(MOTOR_PIN, s2);
  // Motor Testing END

  // Speaker Testing START
  s3 = SensorControl(SENSOR_PIN);
  speaker = SpeakerControl(SPEAKER_PIN, s3);
  // Speaker Testing END

}

void loop() {

  // Sensor Testing START
  Serial.write(s1.detected());
  delay(50);
  // Sensor Testing END

  // Motor Testing START
  /**
    - Motor should spin for 5 seconds, stop for 1 second.
    - After 21 seconds without a sensor triggering, the motor will stop completely.
    - After sensor is triggered again the internal timer resets.
  */
  motor.run();
  // Motor Testing END

  // Speaker Testing START
  /**
    Tone should play every time a the sensor is triggered.
  */
  speaker.run();
  // Speaker Testing END
    
}