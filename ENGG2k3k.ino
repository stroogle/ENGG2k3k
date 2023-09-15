  /*
  Global Variable Creation for Arduino PINS
  */
 const int analogPin = A5;  // the "SCL" LCD display pin
 const int analogPin = A4;  // the "SDK" LCD display pin

void setup() {
      Serial.begin(115200);
  /*
   * Wait for the chip to be initialized completely, and then exit.
   * Select several bits for initialization, e8Bit for 8 bits and e4Bit for 4 bits.
   */
  while(LED.begin(LED.e8Bit) != 0)
        {
            Serial.println("Failed to initialize the chip , please confirm the chip connection!");
            delay(1000);
        }
  /*
   * Set the display area 
   * Please resend the display value if the display area is changed
   */
        LED.setDisplayArea(0,1,2,3,4,5,6,7); //need to test how setting of the setDisplayArea work
        LED.print("0","0","0","0","0","0","0","0");
  /*
   * Arduino Pin Setup
   */

}

void loop() {
    
}