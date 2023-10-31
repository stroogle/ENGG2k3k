
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

// const int LED_PIN = 8; // Pin to which the LED strip is connected
// const int NUMBER_LEDS = 40; // Number of LEDs
// int brightness = 255;
// bool increase = false;
// Adafruit_NeoPixel strip; // Adafruit_NeoPixel object

void setup() {

  Serial.begin(9600);

  // strip = Adafruit_NeoPixel(NUMBER_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
  // strip.begin();

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

  // if (brightness < 255 && increase == true) {
  //     brightness+=5;
  //     if (brightness == 255) {
  //         increase = false;
  //     }
  // }
  // if (brightness >= 0 && increase == false) {
  //     brightness-=5;
  //     if (brightness == 0) {
  //         increase = true;
  //     }
  // }
  // for (int i = 0; i < NUMBER_LEDS; i++) {
  //     strip.setPixelColor(i, strip.Color(0, brightness, 0)); // Set color using Adafruit_NeoPixel's Color function
  //     //strip.show();
  // }

  // strip.show();
    
}