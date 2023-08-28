// Here is the document I will start to write tests in.
#include <iostream>
#include <string>

// Function to test
bool function1(int a) {
    return a > 5;   
}

// If parameter is not true, test fails
// This check function would be provided by the test framework
#define IS_TRUE(x) { if (!(x)) std::cout << __FUNCTION__ << " failed on line " << __LINE__ << std::endl; }

// Test for function1()
// You would need to write these even when using a framework
void test_function1()
{
    IS_TRUE(!function1(0));
    IS_TRUE(!function1(5));
    IS_TRUE(function1(10));
}

// SAMPLE: Function to add two integers
int add(int a, int b) {
    return a + b;
}

// Coded in assertEquals similar to JUnit testing. Can be adjusted for different dataTypes. 
bool assertEquals(int actual, int expected, const std::string& testName) {
    if (actual == expected) {
        std::cout << "PASS - " << testName << std::endl;
        return true;
    } else {
        std::cout << "FAIL - " << testName << " (Expected: " << expected << ", Actual: " << actual << ")" << std::endl;
        return false;
    }
}

int main() {
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



