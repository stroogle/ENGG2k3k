/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef Interface_h
#define Interface_h

// Servo Library
#include <Servo.h>
#include <Arduino.h>
// #include <DFRobot_LedDisplayModule.h> - old LCD display
// #include "DFRobot_LedDisplayModule.h" - old LCD display
#include <FastLED.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>

class SensorControl {
    // Assignee: Alexey
    private:
        enum TriggeredState {Triggered, NotTriggered};
        TriggeredState state = NotTriggered;
        int sensorPin;
        unsigned long lastDetectedTime;
        /**
         * @brief 
         * 
         * @return true - if the light stream between the sensor/reflector is broken.
         * @return false 
         */
        bool lightIsBlocked() {
            int Obstacle;
            Obstacle = digitalRead(sensorPin);

            return Obstacle == LOW;
             
        }

    public:
        SensorControl() {
          sensorPin = 1;
          lastDetectedTime = millis();
          pinMode(sensorPin, INPUT_PULLUP);
        };

        /**
         * @param sensorPinInput is the pin that the sensor will be plugged into
        */
        SensorControl(int sensorPinInput){
            sensorPin = sensorPinInput;
            lastDetectedTime = millis();
            pinMode(sensorPin,INPUT_PULLUP);
        };
        
        /**
         * @brief Checks if the sensor has detected an object and sets a timestamp for this detection time. Will only be true ONCE per trigger
         * 
         * @return true - If the marble has detected an object.
         * @return false - otherwise
         */
        bool detected() {
            bool sensorBlocked = lightIsBlocked();

            if (sensorBlocked && state == Triggered)
            {
                return false;
            }

            else if (sensorBlocked && state == NotTriggered)
            {
                state = Triggered;
                lastDetectedTime = millis();
                return true;
            }

            else 
            {
                state = NotTriggered;
                return false;
            }
          
        }

        /**
         * @brief holds timestamp for last time a marble was detected
         * 
         * 
         * @return timestamp of the last time a marble has passed the sensor in milliseconds
        */
        int lastDetected() {

            return lastDetectedTime;

        }  
};

class MotorControl {
    // Assignee: COLM
    private:
        Servo motor;
        // SensorControl sensor;
        enum MotorState {Rotating, Stopped};
        MotorState state;
        unsigned long stoppedTimeStamp;
        unsigned long rotatingTimeStamp;
        int STOPPED_TIME_MS = 1000;
        int ROTATE_TIME_MS = 5000;
        int STOPPED_SPEED = 90;
        int ROTATE_SPEED = -180;
        int DETECTED_THRESHOLD_MS = 20000;

        /**
         * @brief Set the Speed object
         * 
         * @param speed The speed of the continuous motor
         */
        void setSpeed(int speed) {
            motor.write(speed);
        }

        /**
         * @brief Get the Speed object
         * 
         * @return int the speed of the continuous motor
         */
        int getSpeed() {
            return motor.read();
        }

        /**
         * @brief Used to check if the motor was last stopped timeMs ago.
         * 
         * @param timeMs - The amount of time in MS to check the motor has stopped for
         * @return true - When it has been atleast timeMs since stoppedTimeStamp was set 
         * @return false - Otherwise
         */
        bool hasStoppedFor(int timeMs) {
            return (stoppedTimeStamp + timeMs) < millis();
        }

        /**
         * @brief Used to check if the motor was last rotating timeMs ago
         * 
         * @param timeMs - The amount of time in MS to check the motor has been rotating for
         * @return true - When the motor has been atleast timeMS since rotatingTimeStamp was set.
         * @return false - Otherwise
         */
        bool hasRotatedFor(int timeMs) {
            return (rotatingTimeStamp + timeMs) < millis();
        }

        /**
         * @brief Stops the motor from turning.
         * 
         */
        void stop() {
            stoppedTimeStamp = millis();
            state = Stopped;
            setSpeed(STOPPED_SPEED);
        }

        /**
         * @brief Starts the motor rotating
         * 
         */
        void rotate() {
            rotatingTimeStamp = millis();
            state = Rotating;
            setSpeed(ROTATE_SPEED);
        }

    public:

        MotorControl() {
          motor.attach(1);
        //   sensor = SensorControl();
          stop();
        }

        /**
         * @brief Construct a new Motor Control object
         * 
         * @param motorPin The ping to attach the motor too.
         */
        MotorControl(int motorPin) {
            motor.attach(motorPin);
            // sensor = s;
            stop();
        }

        /**
         * @brief Runs the motor stopping mechanism.
         * 
         */
        void run() {

            unsigned long time = (millis() / 1000) % 5;

            if(time == 1) {
                motor.write(STOPPED_SPEED);
            } else {
                motor.write(ROTATE_SPEED);
            }
        }

        // LEGACY INTERFACE DESIGN, HERE INCASE WE DON'T GET CONTINOUS MOTOR.
        /**
         * @brief Rotates the motor the specified number of degrees.
         * 
         * @param deg 
         */
        void rotate(int deg) {
            int currentRotation = getRotation();
            motor.write(currentRotation + deg);
        }

        /**
         * @brief Gets the servo's rotation
         * 
         */
        int getRotation() {
            return motor.read();
        }
};

class LightingControl {
private: 
    static const int LED_PIN = 7; // Pin to which the LED strip is connected
    static const int NUMBER_LEDS = 40; // Number of LEDs
    int brightness = 255;
    bool increase = false;
    Adafruit_NeoPixel strip; // Adafruit_NeoPixel object

public:
    LightingControl() : strip(NUMBER_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800) {
        strip.begin(); // Initialize the LED strip
    }

    /**
     * @brief Sends LED light wave up the Archimedes Screw
     */
    void sendWave() {
        if (brightness < 255 && increase == true) {
            brightness;
            if (brightness == 255) {
                increase = false;
            }
        }
        if (brightness >= 0 && increase == false) {
            brightness-=5;
            if (brightness == 0) {
                increase = true;
            }
        }
        for (int i = 0; i < NUMBER_LEDS; i++) {
            strip.setPixelColor(i, strip.Color(0, brightness, 0)); // Set color using Adafruit_NeoPixel's Color function
        }
        strip.show();
    }
};

class DisplayControl {
        private:
        //LiquidCrystal_I2C_h lcd(0x27,16,2);
        public:
        LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27,16,2);
          // volatile uint8_t LCD_Addr = 0x27; //I2C Address of our display
         // LiquidCrystal_I2C lcd(0x27,16,2);
          // DisplayControl() {
          //     Serial.begin(115200);
          //     /*
          //     * Wait for the chip to be initialized completely, and then exit.
          //     * Select several bits for initialization, e8Bit for 8 bits and e4Bit for 4 bits.
          //     */
          //     while(LED.begin(LED.e8Bit) != 0)
          //     {
          //         Serial.println("Failed to initialize the chip , please confirm the chip connection!");
          //         delay(1000);
          //     }
          //     /*
          //     * Set the display area 
          //     * Please resend the display value if the display area is changed
          //     */
          //     LED.setDisplayArea(0,1,2,3,4,5,6,7); //need to test how setting of the setDisplayArea work
          //     LED.print("0","0","0","0","0","0","0","0");
              
          // }

      DisplayControl() {
        // lcd = LiquidCrystal_I2C;
    // Assignee: Andrei Ziganshin
        /**
         * @brief Displays the number provided onto the LED display 2x16 (bottom row). 
         *  
         *        Call showCount(number_of_marbles);
         * 
         * @param number
         *
         * put displaySetup(); and i2C_display_scan(); at the start of the main setup function
         */
       //  pinMode(A5, OUTPUT); //SCL pin **pending setup of the pin
       //  pinnMode(A4, OUTPUT); //SDA pin **pending setup of the pin
      }
        char hex(int value) {
        return "0123456789ABCDEF"[value & 0x0f]; 
        }
        void showCount(uint16_t val) {
            lcd.setCursor(15,1);
            lcd.print(hex(val%10));
            val = val/10;
            
            lcd.setCursor(14,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(13,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(12,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(11,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(10,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(9,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(8,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(7,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(6,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(5,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(4,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(3,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(2,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(1,1);
            lcd.print(hex(val%10));
            val = val/10;

            lcd.setCursor(0,1);
            lcd.print(hex(val%10));
        }
        void displaySetup(){
            
            lcd.init();                      // initialize the lcd 
            // Print a message to the LCD.
            lcd.backlight();
            lcd.setCursor(0,0); // (1st number indicate the row "0" - top , "1" - bottom , 2nd- position starts from 0!)
            lcd.print("T1 Marble Count:"); //top row message
            
        }
        void i2C_display_scan(){
            byte error, address;
            int nDevices;
            
            Serial.println("Scanning...");
            
            nDevices = 0;
            for(address = 1; address < 127; address++ )
            {
                // The i2c_scanner uses the return value of
                // the Write.endTransmisstion to see if
                // a device did acknowledge to the address.
                Wire.beginTransmission(address);
                error = Wire.endTransmission();
            
                if (error == 0)
                {
                Serial.print("I2C device found at address 0x");
                if (address<16)
                    Serial.print("0");
                Serial.print(address,HEX);
                Serial.println("  !");
            
                nDevices++;
                }
                else if (error==4)
                {
                Serial.print("Unknown error at address 0x");
                if (address<16)
                    Serial.print("0");
                Serial.println(address,HEX);
                }    
            }
            if (nDevices == 0)
                Serial.println("No I2C devices found\n");
            else
                Serial.println("done\n");
            
            delay(5000);           // wait 5 seconds for next scan

        }
};

class CounterControl {
    // Assignee: Ibrahim
    private:
        int counter;

    public:
        CounterControl() {
            counter = 1;
        }

        CounterControl(int number) {
            counter = number;
        }
        /**
         * @brief Get the Count
         * 
         * @return int - The current count
         */
        int getCount() {
            return counter;  // Return counter
        }

        /**
         * @brief Set the Count object
         * 
         * @return int - The previous value of the counter
         */
        int setCount(int number) {

            if(number > 99999999){
                number = 0;
            }
            
            int previousValue = counter; // Store the previous value
            counter = number;           // Update the counter
            return previousValue;       // Return the previous value
        }

        /**
         * @brief Increment the Count of the object
         * 
         * @return int - The previous value of the counter
         */
        int incrementCount() {
            return setCount(counter + 1); // Take stored value from setCount and incrementing by 1 and return it
        }
};

class MarbleCountDisplay {
    // Assignee: Thomas
    private:
        DisplayControl display;
        SensorControl sensor;
        CounterControl counter;

    public:
        MarbleCountDisplay() {
          display = DisplayControl();
          sensor = SensorControl();
          counter = CounterControl();
        }

        MarbleCountDisplay(DisplayControl d, SensorControl s, CounterControl c) {
          display = d;
          sensor = s;
          counter = c;
        }

        /**
         * @brief Begins the Marble Counter Component
         * 
         */
        void run() {
            if(sensor.detected()) {
                counter.incrementCount();
            }
            display.showCount(counter.getCount());
        }
};

class SpeakerControl {

    private:
        SensorControl sensor;
        int TONE_DELAY_MS = 40;
        unsigned long LAST_TONE_PLAY;
        int SPEAKER_PIN;

    public:
        SpeakerControl(int speakerPin, SensorControl s) {
            sensor = s;
            SPEAKER_PIN = speakerPin;
            LAST_TONE_PLAY = millis() + TONE_DELAY_MS;
        }

        void run() {
            if(LAST_TONE_PLAY + TONE_DELAY_MS < millis()) {
              noTone(SPEAKER_PIN);
            } else if(sensor.detected()) {
              tone(SPEAKER_PIN, 1047, 8);
              LAST_TONE_PLAY = millis();
            } 
        }
};

#endif