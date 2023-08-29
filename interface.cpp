// Servo Library
#include <Servo.h>


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