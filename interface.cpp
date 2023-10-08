// Servo Library
#include <Servo.h>
#include <Arduino.h>
// #include <DFRobot_LedDisplayModule.h>
#include "DFRobot_LedDisplayModule.h"
#include <FastLED.h> 
#include <TMRpcm.h>

class SensorControl {
    // Assignee: Alexey
    private:
        enum TriggeredState {Triggered, NotTriggered};
        TriggeredState state = NotTriggered;
        int sensorPin;
        int lastDetectedTime;
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
          pinMode(sensorPin, INPUT);
        };

        /**
         * @param sensorPinInput is the pin that the sensor will be plugged into
        */
        SensorControl(int sensorPinInput){
            sensorPin = sensorPinInput;
            lastDetectedTime = millis();
            pinMode(sensorPin,INPUT);
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
        SensorControl sensor;
        enum MotorState {Rotating, Stopped};
        MotorState state;
        int stoppedTimeStamp;
        int rotatingTimeStamp;
        int STOPPED_TIME_MS = 1000;
        int ROTATE_TIME_MS = 5000;
        int STOPPED_SPEED = 0;
        int ROTATE_SPEED = 180;
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
          sensor = SensorControl();
          stop();
        }

        /**
         * @brief Construct a new Motor Control object
         * 
         * @param motorPin The ping to attach the motor too.
         */
        MotorControl(int motorPin, SensorControl s) {
            motor.attach(motorPin);
            sensor = s;
            stop();
        }

        /**
         * @brief Runs the motor stopping mechanism.
         * 
         */
        void run() {
            if(sensor.lastDetected() + DETECTED_THRESHOLD_MS < millis()) {
                stop();
                return;
            }
            if(state == Stopped && hasStoppedFor(STOPPED_TIME_MS))
            {
                rotate();
            } else if (state == Rotating && hasRotatedFor(ROTATE_TIME_MS))
            {
                stop();
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
    // Assignee: Eli
    private: 
        SensorControl entrySensor;
        int brightness;
        bool increase;
        static const int LED_PIN = 7; // whatever pin we connect it to 
        static const int NUMBER_LEDS = 40; // number of LEDS
        CRGB LED[40]; 

    public:
        
        LightingControl() {
            entrySensor = SensorControl();
            brightness = 0;
            increase = true;
            FastLED.addLeds<WS2812, LightingControl::LED_PIN, GRB>(LED, LightingControl::NUMBER_LEDS);
        }

        LightingControl (SensorControl s) {
            entrySensor = s;
            brightness = 0;
            increase = true;
            FastLED.addLeds<WS2812, LightingControl::LED_PIN, GRB>(LED, LightingControl::NUMBER_LEDS);
        }
        /**
         * @brief Sends LED light wave up the Archimedes Screw
         */
        void sendWave() {
            if(entrySensor.detected()){
                if(brightness <= 255 && increase == true){
                    brightness++;
                    if(brightness == 255) increase == false;
                }
                if(brightness >= 0 && increase == false){
                    // QUESTION FOR ELI: when will increase ever be false? 
                    brightness--;
                    if(brightness==0) increase == true;
                }
                for (int i = 0; i < NUMBER_LEDS; i++) {
                    LED[i] = CRGB(0, brightness, 0); // Set color for all LEDs
                }
                FastLED.show(); // Display the updated LED colors
            }
        }
    };

class DisplayControl {
        public:
          static DFRobot_LedDisplayModule LED;

        public:
          DisplayControl() {
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
              
          }

      // Assignee: Andrei Ziganshin
          /**
          * @brief Displays the number provided onto the LED display.
          * 
          * @param number 
          */
          void showCount(int number) {
              LED.print(number);
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
        int NUMBER_OF_FILES = 3;
        char* FILES;
        SensorControl sensor;
        TMRpcm audio;

    public:
        SpeakerControl(int speakerPin, char* files, SensorControl s) {
            audio.speakerPin = speakerPin;
            FILES = files;
            sensor = s;
        }

        void playRandom() {
            int fileIdx = millis() % NUMBER_OF_FILES;
            // audio.play(&FILES[fileIdx]);
            playFile(FILES[fileIdx]);
        }

        void playFile(char* file) {
            audio.play(file);
        }

        void run() {
            if(sensor.detected()) {
              playRandom(); // This might be blocking... We will have to test.
            }
        }
};