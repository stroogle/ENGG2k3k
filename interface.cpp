// Servo Library
#include <Servo.h>


class MotorControl {
    // Assignee: COLM
    private:
        Servo motor;

    public:

        /**
         * @brief Construct a new Motor Control object
         * 
         * @param motorPin The ping to attach the motor too.
         */
        MotorControl(int motorPin) {
            motor.attach(motorPin);
        }
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
    // Assignee: Andre
    public:
        /**
         * @brief Displays the number provided onto the LED display.
         * 
         * @param number 
         */
        void showCount(int number) {}
};

class CounterControl {
    // Assignee: Ibrahim
    private:
        int counter;

    public:
        /**
         * @brief Get the Count
         * 
         * @return int - The current count
         */
        int getCount() {}

        /**
         * @brief Set the Count object
         * 
         * @return int - The previous value of the counter
         */
        int setCount(int number) {}

        /**
         * @brief Increment the Count of the object
         * 
         * @return int - The previous value of the counter
         */
        int incrementCount() {}
};

class MarbleCountDisplay {
    // Assignee: Thomas
    private:
        DisplayControl display;
        SensorControl sensor;
        CounterControl counter;

    public:
        MarbleCountDisplay(DisplayControl d, SensorControl s, CounterControl c) {}

        /**
         * @brief Begins the Marble Counter Component
         * 
         */
        void run() {}
};