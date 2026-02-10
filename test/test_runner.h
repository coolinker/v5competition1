#pragma once
#include "vex.h"

namespace test {
    void assert_true(bool condition, const char* message);
    void assert_false(bool condition, const char* message);
    void assert_equal(double actual, double expected, double tolerance, const char* message);
    void run_test(void (*test_func)(), const char* test_name);
    void report_results();
}
