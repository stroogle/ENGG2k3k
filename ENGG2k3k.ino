
#include <FastLED.h> 
// #include "Interface.h"
#include "Interface.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

 /*
  Global Variable Creation for Arduino PINS
  */
 const int analogPin = A5;  // the "SCL" LCD display pin
 const int analogPin2 = A4;  // the "SDK" LCD display pin

 Servo motor;
LightingControl light;
SpeakerControl speaker;
SensorControl s1;
SensorControl s2;
CounterControl counter;
DisplayControl display;
MarbleCountDisplay marbleDisplay;

void setup() {

  Serial.begin(9600);
  display.displaySetup();

  s1 = SensorControl(2);
  speaker = SpeakerControl(3, s1);

  s2 = SensorControl(2);
  counter = CounterControl(0);
  marbleDisplay = MarbleCountDisplay(display, s2, counter);

  motor.attach(7);

}

void loop() {

  light.sendWave();

  unsigned long time = (millis() / 1000) % 5;

  if(time == 1) {
      motor.write(90);
      Serial.println("Stopped!");
  } else {
      motor.write(180);
      Serial.println("Rotating!");
  }

  speaker.run();

  marbleDisplay.run();
    
}
