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
test(lightingIsTriggeredByIsDetected) {
    // this tests whether lighting is correctly triggered by isDetected
    
    LightingControl lighting = new LightingControl();

    lighting.sendWave();
}

test(isDetectedIsVeryQuick) {
    // this tests whether their is an issue if isDetected() is very quick, and the wave hasn't finished.
    LightingControl lighting = new LightingControl();

    lighting.sendWave();
}

test(isDetectedWhilstLoopIsStillRunning) {
    // is detected whilst the loop is still running (if possible, depending on the logic of the isDetected algorithm)
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
