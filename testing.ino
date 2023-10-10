#include <Arduino.h>
#include <AUnit.h>

// Include functions and classes to be tested here.
#include "Interface.h"


// SENSOR CONTROL: JAMES

test(testSensorWorking) {
    // Define a mock class for SensorControl
    class MockSensorControl : public SensorControl {
    private:
        detectedOnce = false;
        bool lightIsBlocked() override{
            return detectedOnce;
        }
    public: 
        void toggleBlocked() {
            detectedOnce = !detectedOnce;
        }
    };   

    MockSensorControl sensor = new MockSensorControl();

    bool lightBlocked = sensor.detected();
    
    assertEqual(lightBlocked, false);

    sensor.toggleBlocked();
    lightBlocked = sensor.detected();
    assertEqual(lightBlocked, true);

    lightBlocked = sensor.detected();
    assertEqual(lightBlocked, false);

}

test(testSensorTimeGap) {
    // Define a mock class for SensorControl
    class MockSensorControl : public SensorControl {
    private:
        detectedOnce = false;
        bool lightIsBlocked() override{
            return detectedOnce;
        }
    public: 
        void toggleBlocked() {
            detectedOnce = !detectedOnce;
        }
    };    

    MockSensorControl sensor = new MockSensorControl();
    assertTrue(if(sensor.lastDetected > 0))
} 


// MOTOR CONTROL: ALEXEY
test(testMotorRotation) {
    // This will test that the motor logic for running and stopping the motor works
    
    SensorControl sensor = new SensorControl();
    MotorControl motor = new MotorControl(sensor);

    // Running the motor as soon at the box is initialised (the rotation should be 0)
    motor.run();
    assertEqual(motor.getRotation(), 90);

    // Delaying time for 1.1 seconds to allow for the motor to start and running motor again (rotation should be 180)
    delay(1100);
    motor.run();
    assertEqual(motor.getRotation(), 180);

    // Delayinhg time for 21 second to ensure that the no dection threshold is hit and running motor again (roation should be 0)
    delay(21000);       // change this value if the DETECTED_THRESHOLD_MS value changes
    motor.run();
    assertEqual(motor.getRotation(), 90);
}

// LIGHTING CONTROL: ERIK
test(lightingIsNotTriggered) {
    // this tests whether lighting is correctly triggered by isDetected
    SensorControl sensor = new SensorControl();

    LightingControl lighting = new LightingControl(sensor);

    // sensor.detected() will return false, send wave will not run
    lighting.sendWave();

    // the lights are not on.
    assertEqual(lighting.brightness, 0)
}

test(lightingIsTriggeredByIsDetected) {
    
    // Define a mock class for SensorControl
    class MockSensorControl : public SensorControl {
    private:
        detectedOnce = false;

    public:
        bool detected() override {
            // Spoof the detected() function to always return true during testing
            if (detectedOnce) {
                return true
            } else {
                return false;
            }
        }
    };

    // this tests whether lighting is correctly triggered by isDetected
    MockSensorControl sensor = new MockSensorControl();

    LightingControl lighting = new LightingControl(sensor);
    
    lighting.sendWave();

    // the lights have been turned on.
    assertMore(lighting.brightness, 0)
}

test(isDetectedIsVeryQuick) {
    // this tests whether their is an issue if isDetected() is very quick, and the wave hasn't finished.
    LightingControl lighting = new LightingControl();

    lighting.sendWave();
}

// DISPLAY CONTROL: ELI

test(initialisation) {
    // this tests whether their is an issue if isDetected() is very quick, and the wave hasn't finished.
    DisplayControl display = new DisplayControl();

    assertEqual(display.LED, 00000000)
}

test(testCount) {
    // this tests whether their is an issue if isDetected() is very quick, and the wave hasn't finished.
     DisplayControl display = new DisplayControl();

    assertEqual(display.LED, 10)
}
// COUNTER CONTROL: ANDREI


// MARBLE-COUNT DISPLAY: IBRAHIM
test(getMarbleCountTest) {
    int expectedCount = 10; // number of marbles we manually insert and expect to see as "detected". Subject to change.
    CounterControl count = new CounterControl(0);  // a new counter for when we run this test

    int actualCount = count.getCount();
    assertEqual(expectedCount, actualCount);
}

// SPEAKER CONTROL: THOMAS



void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for the serial port to connect.
    }

    // Initialize the test runner and run the tests.
    TestRunner::run();
}

void loop() {
    // Nothing to do in the loop for unit testing as we only want tests to run once.
}
