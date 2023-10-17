
#include <FastLED.h> 
// #include "Interface.h"
#include "Interface.h"

const int SENSOR_PIN = 8;
const int MOTOR_PIN = 7;
const int SPEAKER_PIN = 3;

SensorControl s1;
SensorControl s2;
SensorControl s3;
SensorControl s4;
SensorControl s5;
MotorControl motor;
SpeakerControl speaker;
MarbleCountDisplay MCD;
LightingControl lighting;
DisplayControl display;
CounterControl counter;

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

  //MarbleCountDisplay and Counter Testing START
  s4 = SensorControl(SENSOR_PIN);
  display = DisplayControl();
  counter = CounterControl(1);
  MCD = MarbleCountDisplay(display, s4, counter);
  // MarbleCountDisplay and Counter Testing END

  //Lighting Testing START
  s5 = SensorControl(SENSOR_PIN);
  lighting = LightingControl(s5);
  //Lighting Testing END
}

void loop() {

  // Sensor Testing START
  // Serial.println(s1.detected());
  // delay(500);
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

  // MarbleControlDisplay and Counter Testing START
  /*
    - When sensor is detected, it should print updated display count
  */
  MCD.run();
  // delay(500);
  // MarbleControlDisplay and Counter Testing END

  //Lighting Testing START
  // lighting.sendWave();
  //Lighting Testing END
}