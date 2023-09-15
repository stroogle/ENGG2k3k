// Servo Library
#include <Servo.h>
#include <Arduino.h>
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
    public:
        /**
         * @brief Sends LED light wave up the Archimedes Screw
         */
        void sendWave() {}
};

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

class DisplayControl {
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
        char* FILES[NUMBER_OF_FILES] = {"file1.WAV", "file2.WAV", "file3.WAV"};
        SensorControl sensor;

    public:
        SpeakerControl(int speakerPin, char* files, SensorControl s) {
            tmrcpm.speakerPin = speakerPin;
            FILES = files;
            sensor = s;
        }

        void playRandom() {
            int fileIdx = millis() % NUMBER_OF_FILES;
            tmrcpm.play(FILES[fileIdx]);
        }

        void playFile(String file) {
            tmrcpm.play(file);
        }

        void run() {
            if(sensor.detected()) {
              playRandom(); // This might be blocking... We will have to test.
            }
        }
}