#include "test/test_runner.h"
#include <cmath>

namespace test {
    static int tests_passed = 0;
    static int tests_failed = 0;

    void assert_true(bool condition, const char* message) {
        if (condition) {
            tests_passed++;
        } else {
            tests_failed++;
            Brain.Screen.print("[FAIL] %s", message);
            Brain.Screen.newLine();
        }
    }

    void assert_false(bool condition, const char* message) {
        assert_true(!condition, message);
    }

    void assert_equal(double actual, double expected, double tolerance, const char* message) {
        if (std::abs(actual - expected) <= tolerance) {
            tests_passed++;
        } else {
            tests_failed++;
            Brain.Screen.print("[FAIL] %s. Expected: %f, Got: %f", message, expected, actual);
            Brain.Screen.newLine();
        }
    }

    void run_test(void (*test_func)(), const char* test_name) {
        Brain.Screen.print("Running: %s...", test_name);
        Brain.Screen.newLine();
        int before_passed = tests_passed;
        int before_failed = tests_failed;
        
        test_func();

        if(tests_failed == before_failed) {
             Brain.Screen.print("...%s [OK]", test_name);
             Brain.Screen.newLine();
        }
    }

    void report_results() {
        Brain.Screen.newLine();
        Brain.Screen.print("====================");
        Brain.Screen.newLine();
        Brain.Screen.print("Test Results:");
        Brain.Screen.newLine();
        Brain.Screen.print("Passed: %d", tests_passed);
        Brain.Screen.newLine();
        Brain.Screen.print("Failed: %d", tests_failed);
        Brain.Screen.newLine();
        Brain.Screen.print("====================");
        Brain.Screen.newLine();

        if (tests_failed > 0) {
            Brain.Screen.print("SOME TESTS FAILED!");
        } else {
            Brain.Screen.print("ALL TESTS PASSED!");
        }
    }
}
