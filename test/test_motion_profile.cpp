#include "test/test_motion_profile.h"
#include "test/test_runner.h"
#include "control/motion_profile.h"
#include "config.h"

namespace test {
    void test_motion_profile_logic() {
        MotionProfile profile(MAX_VELOCITY, MAX_ACCELERATION);

        // Test 1: Acceleration phase
        double v1 = profile.get_target_velocity(0.1, 2.0); // 0.1s elapsed, 2m to go
        assert_equal(v1, MAX_ACCELERATION * 0.1, 0.001, "Profile should be in acceleration phase");

        // Test 2: Max velocity phase
        double v2 = profile.get_target_velocity(1.0, 2.0); // 1.0s elapsed, 2m to go
        assert_equal(v2, MAX_VELOCITY, 0.001, "Profile should be at max velocity");

        // Test 3: Deceleration phase
        double v3 = profile.get_target_velocity(1.0, 0.1); // 1.0s elapsed, 0.1m to go
        assert_true(v3 < MAX_VELOCITY, "Profile should be in deceleration phase");
    }
}
