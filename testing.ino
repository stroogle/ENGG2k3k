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

test(TPM10testIRSensorResponsiveness) {
   
    SensorControl sensor;

    // Ensure the sensor is initially not detected
    assertEqual(sensor.detected(), false);

    // Start measuring time
    unsigned long startTime = millis();

    // Simulate a marble passing through by calling detected()
    bool marbleDetected = sensor.detected();

    // Check if the sensor responds and increments ball count within 200ms
    unsigned long endTime = millis();
    unsigned long responseTime = endTime - startTime;

    // Verify that the response time is within the acceptable range (200ms)
    assertLessThanOrEqual(responseTime, 200);

    // Ensure that the sensor's status reflects the marble detection
    assertEqual(marbleDetected, true);
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
