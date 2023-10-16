#include <Arduino.h>
#include <AUnit.h>

// Include functions and classes to be tested here.
#include "Interface.h"


// SENSOR CONTROL: JAMES


// MOTOR CONTROL: ALEXY
test(testMotorOn) {
    SensorControl sensor = new SensorControl(2);
    Motor testMotor = new Motor(9, sensor);  // a new counter for when we run this test
    testMotor.rotate();

    bool motorOn = true;
    bool motorActualState = if(testMotor.getSpeed)
    
    assertEqual(motorOn, motorActualState);
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


// COUNTER CONTROL: ANDREI


// MARBLE-COUNT DISPLAY: IBRAHIM
test(getMarbleCountTest) {
    int expectedCount = 10; // number of marbles we manually insert and expect to see as "detected". Subject to change.
    CounterControl count = new CounterControl(0);  // a new counter for when we run this test

    int actualCount = count.getCount();
    assertEqual(expectedCount, actualCount);
    
   

}

test(detectedSensorOnDisplay) {  //  checking if Sensor works and send counts into LCD

    // Define a mock class for SensorControl
    class MockSensorControl : public SensorControl {
    private:
        int detectedAmount = 5;

    public:
        bool detected() override {
            // Spoof the detected() function to always return true during testing
            if (detectedAmount > 0) {
                detectedAmount--;
                return true
            } else {
                return false;
            }
        }
    };

    MockSensorControl sensor = new MockSensorControl(); // Assume sersor detected 5 times 
    
    CounterControl count = new CounterControl(0);  // a new counter for when we run this test


    for(int i = 0; i <10; i++) {
        // sensor is detected only 5 of the times.
        if(sensor.detected()) {
            count.incrementCount();   
        }
    }

    assertEqual(count.getCount, 5);     //Assume this pass
}   
   


test(mockMarbleCountDisplayPass){   // Mock sensor and display shows the same value so the function works
    
    // Define a mock class for SensorControl
    class MockSensorControl : public SensorControl {
    private:
        int detectedAmount = 10;

    public:
        bool detected() override {
            // Spoof the detected() function to always return true during testing
            if (detectedAmount > 0) {
                detectedAmount--;
                return true
            } else {
                return false;
            }
        }
    };

    MockSensorControl sensor = new MockSensorControl(); // Getting true 
    
    CounterControl count = new CounterControl(10);  // Assume CounterControl works  


    for(int i = 0; i <11; i++) {
        // sensor is detected only 10 of the times.
        if(sensor.detected()) {
            count.incrementCount();  // As CounterControl, count will increase
        }
    }

    assertEqual(display.showCount(counter.getCount()), 10);  // This test should pass
}

test(mockMarbleCountDisplayFail){ // Mock sensor and display shows different value 
                                 //so the function does not work.
    
    // Define a mock class for SensorControl
    class MockSensorControl : public SensorControl {
    private:
        int detectedAmount = 10;

    public:
        bool detected() override {
            // Spoof the detected() function to always return true during testing
            if (detectedAmt > 0) {
                detectedAmt--;
                return true
            } else {
                return false;
            }
        }
    };

    MockSensorControl sensor = new MockSensorControl();  // Getting true 
    
    CounterControl count = new CounterControl(8);  // Assume CounterControl does not work


    for(int i = 0; i <11; i++) {
        // sensor is detected only 10 of the times.
        if(sensor.detected()) {
            count.incrementCount();     // As CounterControl, count will increase til only 8
        }
    }

    assertEqual(display.showCount(counter.getCount()), 10);  // This test should fail 
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
