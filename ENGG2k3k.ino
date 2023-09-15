#include <FastLED.h> 
 
#define LED_PIN 7 // whatever pin we connect it to 
#define NUMBER_LEDS 40 // number of LEDS 
 
CRGB LED[NUMBER_LEDS]; 

void setup() {
FastLED.addLeds<WS2812, LED_PIN, GRB>(LED, NUMBER_LEDS); // need to check LED type 

}

void loop() {
    
}