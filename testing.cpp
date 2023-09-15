// Here is the document I will start to write tests in.
// #include <Arduino.h>
#include <iostream>
#include <stdlib.h>
#include <AUnit.h>

/*
Arduino have a unit testing library, AUnit. We will utilise this for the unit testing of our code. 
Below is sample code before I knew about the AUnit library. Below I used user defined assertEquals similar
to JUnit testing. 
*/

void setup()
{
    // Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

// SAMPLE: Function to add two integers
int add(int a, int b)
{
    return a + b;
}

/*
NOTE: This testing code will be added to the end of the interface.cpp file.
This will make it easier for function calls to be used for testing.
*/

// eg:
void testLightOn(int currentLightVal)
{
    // Serial.print("testLightOn :");
    if (currentLightVal > 0)
    {
        // Serial.print("PASSED");
    }
    else
        // Serial.print("FAILED");
}

// Coded in assertEquals similar to JUnit testing. Can be adjusted for different dataTypes.
bool assertEquals(int actual, int expected, const std::string &testName)
{
    if (actual == expected)
    {
        // std::cout << std::endl is to be changed so we can use Serial.println()
        std::cout << "PASS - " << testName << std::endl;
        return true;
    }
    else
    {
        std::cout << "FAIL - " << testName << " (Expected: " << expected << ", Actual: " << actual << ")" << std::endl;
        return false;
    }
}

int main()
{
    // Call all tests. Using a test framework would simplify this.

    // Test case 1
    int result1 = add(5, 3);
    assertEquals(result1, 8, "Test Case 1");

    // Test case 2
    int result2 = add(-2, 7);
    assertEquals(result2, 5, "Test Case 2");

    // Test case 3
    int result3 = add(0, 0);
    assertEquals(result3, 0, "Test Case 3");

    return 0;
}
