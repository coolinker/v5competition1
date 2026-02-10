#include "test/test_pid.h"
#include "test/test_runner.h"
#include "control/pid.h"
#include "hal/time.h"

namespace test {
    void test_pid_controller() {
        PIDController pid(1.0, 0.5, 0.1);
        
        // Test 1: Proportional term
        double output = pid.calculate(10.0, 5.0);
        assert_true(output > 0, "P-term should produce positive output for positive error");
        wait_ms(20); // simulate time passing

        // Test 2: Integral term
        pid.calculate(10.0, 5.0); // Run a few times to build up integral
        wait_ms(20);
        double output2 = pid.calculate(10.0, 5.0);
        assert_true(output2 > output, "I-term should increase output over time");

        // Test 3: Reset
        pid.reset();
        double output3 = pid.calculate(10.0, 5.0);
        assert_equal(output, output3, 0.001, "Reset should clear integral and derivative history");
    }
}
