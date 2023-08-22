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
    // Assignee: Alexy
    private:
        enum TriggeredState {Triggered, NotTriggered};
        TriggeredState state = NotTriggered;

        /**
         * @brief 
         * 
         * @return true - if the light stream between the sensor/reflector is broken.
         * @return false 
         */
        bool lightIsBlocked() {}

    public:
        /**
         * @brief Checks if the sensor has detected an object. Will only be true ONCE per trigger
         * 
         * @return true - If the marble has detected an object.
         * @return false - otherwise
         */
        bool detected() {}
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