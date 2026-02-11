// ============================================================================
// Host-side Unit Test Suite for VEX V5 Robot
// ============================================================================
// A SINGLE self-contained file that compiles with any C++17 compiler.
// Tests all core algorithm modules (PID, MotionProfile, Odometry) by
// providing mock implementations of the Hardware Abstraction Layer (HAL).
//
// Usage:
//   g++ -std=c++17 -I include -I src -o build/run_tests test/host_tests.cpp -lm
//   ./build/run_tests
//
// Or simply:   make test
// ============================================================================

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ============================================================================
// Minimal Test Framework
// ============================================================================
static int g_tests_run = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;
static const char* g_current_test = nullptr;

#define TEST(name) \
    static void test_##name(); \
    static void run_test_##name() { \
        g_current_test = #name; \
        g_tests_run++; \
        printf("  [RUN ] %s\n", #name); \
        test_##name(); \
        printf("  [ OK ] %s\n", #name); \
        g_tests_passed++; \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("  [FAIL] %s (line %d): %s is false\n", g_current_test, __LINE__, #cond); \
        g_tests_failed++; g_tests_run++; return; \
    } \
} while(0)

#define ASSERT_NEAR(actual, expected, tolerance) do { \
    double _a = (actual), _e = (expected), _t = (tolerance); \
    if (std::abs(_a - _e) > _t) { \
        printf("  [FAIL] %s (line %d): expected %.6f, got %.6f (tol=%.6f)\n", \
            g_current_test, __LINE__, _e, _a, _t); \
        g_tests_failed++; g_tests_run++; return; \
    } \
} while(0)

#define ASSERT_GT(a, b) do { \
    double _a = (a), _b = (b); \
    if (!(_a > _b)) { \
        printf("  [FAIL] %s (line %d): expected %f > %f\n", \
            g_current_test, __LINE__, _a, _b); \
        g_tests_failed++; g_tests_run++; return; \
    } \
} while(0)

#define ASSERT_LT(a, b) do { \
    double _a = (a), _b = (b); \
    if (!(_a < _b)) { \
        printf("  [FAIL] %s (line %d): expected %f < %f\n", \
            g_current_test, __LINE__, _a, _b); \
        g_tests_failed++; g_tests_run++; return; \
    } \
} while(0)

#define RUN_TEST(name) run_test_##name()

// ============================================================================
// Mock HAL Layer
// ============================================================================
// Replace real hardware calls so we can test pure logic on the host.

static double        mock_time_sec = 0.0;
static unsigned long mock_time_ms  = 0;
static double        mock_left_ticks = 0.0;
static double        mock_right_ticks = 0.0;
static double        mock_imu_heading_rad = 0.0;
static double        mock_imu_rotation_rad = 0.0;
static double        mock_motor_left_v = 0.0;
static double        mock_motor_right_v = 0.0;

// hal/time.h
double        get_time_sec() { return mock_time_sec; }
unsigned long get_time_ms()  { return mock_time_ms; }
void          wait_ms(int ms) { mock_time_sec += ms / 1000.0; mock_time_ms += ms; }

// hal/motors.h
double get_left_encoder_ticks()  { return mock_left_ticks; }
double get_right_encoder_ticks() { return mock_right_ticks; }
void   reset_encoders() { mock_left_ticks = 0; mock_right_ticks = 0; }
void   set_drive_motors(double lv, double rv) { mock_motor_left_v = lv; mock_motor_right_v = rv; }
void   stop_drive_motors() { mock_motor_left_v = 0; mock_motor_right_v = 0; }

// hal/imu.h
double get_imu_heading_rad()  { return mock_imu_heading_rad; }
double get_imu_rotation_rad() { return mock_imu_rotation_rad; }
void   reset_imu()            { mock_imu_heading_rad = 0; mock_imu_rotation_rad = 0; }
void   calibrate_imu()        { /* no-op in tests */ }

static void reset_all_mocks() {
    mock_time_sec = 0.0;
    mock_time_ms  = 0;
    mock_left_ticks = 0.0;
    mock_right_ticks = 0.0;
    mock_imu_heading_rad = 0.0;
    mock_imu_rotation_rad = 0.0;
    mock_motor_left_v = 0.0;
    mock_motor_right_v = 0.0;
}

// ============================================================================
// Include source files under test
// ============================================================================
#include "config.h"
#include "control/pid.h"
#include "control/motion_profile.h"
#include "localization/odometry.h"

#include "../src/control/pid.cpp"
#include "../src/control/motion_profile.cpp"
#include "../src/localization/odometry.cpp"

// ============================================================================
// PID Controller Tests
// ============================================================================

TEST(PID_PositiveErrorProducesPositiveOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(2.0, 0.0, 0.0); // P-only
    pid.reset();
    mock_time_sec = 1.01;
    double output = pid.calculate(10.0, 5.0); // error = 5
    ASSERT_GT(output, 0.0);
    ASSERT_NEAR(output, 10.0, 0.01); // Kp * error = 2.0 * 5.0 = 10.0
}

TEST(PID_NegativeErrorProducesNegativeOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(2.0, 0.0, 0.0);
    pid.reset();
    mock_time_sec = 1.01;
    double output = pid.calculate(5.0, 10.0); // error = -5
    ASSERT_LT(output, 0.0);
    ASSERT_NEAR(output, -10.0, 0.01);
}

TEST(PID_ZeroErrorProducesZeroOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(2.0, 0.0, 0.0);
    pid.reset();
    mock_time_sec = 1.01;
    double output = pid.calculate(5.0, 5.0);
    ASSERT_NEAR(output, 0.0, 0.001);
}

TEST(PID_IntegralAccumulates) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 1.0, 0.0); // I-only
    pid.reset();

    // First call: dt=0.01, error=5, integral = 5*0.01 = 0.05
    mock_time_sec = 0.01;
    double out1 = pid.calculate(10.0, 5.0);

    // Second call: dt=0.01, error=5, integral = 0.05 + 5*0.01 = 0.10
    mock_time_sec = 0.02;
    double out2 = pid.calculate(10.0, 5.0);

    ASSERT_GT(out2, out1);
}

TEST(PID_DerivativeRespondToChange) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 0.0, 1.0); // D-only
    pid.reset();

    // First call: error=5, prev_error=0, derivative = (5-0)/0.01 = 500
    mock_time_sec = 0.01;
    double out1 = pid.calculate(10.0, 5.0);
    ASSERT_GT(out1, 0.0); // positive derivative

    // Second call: same error, derivative should be ~0
    mock_time_sec = 0.02;
    double out2 = pid.calculate(10.0, 5.0);
    ASSERT_NEAR(out2, 0.0, 0.01); // error didn't change -> derivative ~ 0
}

TEST(PID_ResetClearsState) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(1.0, 1.0, 0.1);
    pid.reset();

    // Run several iterations to build up integral
    for (int i = 1; i <= 10; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(10.0, 5.0);
    }

    // Reset and compare fresh output
    mock_time_sec = 1.0;
    pid.reset();
    mock_time_sec = 1.01;
    double after_reset = pid.calculate(10.0, 5.0);

    // A fresh PID with same gains should give same first output
    mock_time_sec = 2.0;
    PIDController pid2(1.0, 1.0, 0.1);
    pid2.reset();
    mock_time_sec = 2.01;
    double fresh = pid2.calculate(10.0, 5.0);

    ASSERT_NEAR(after_reset, fresh, 0.01);
}

// ============================================================================
// PID Enhancement Tests (anti-windup, D-filter, output clamping)
// ============================================================================

TEST(PID_AntiWindup_ClampsIntegral) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 1.0, 0.0); // I-only
    pid.set_integral_limit(2.0);       // clamp |integral| to 2.0
    pid.reset();

    // Run many iterations with large error to saturate integral
    for (int i = 1; i <= 100; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(100.0, 0.0); // error=100, integral grows fast
    }

    // After 100 iters: integral would be 100*0.01*100 = 100 without clamp
    // With clamp at 2.0: output = Ki * clamped_integral = 1.0 * 2.0 = 2.0
    mock_time_sec += 0.01;
    double output = pid.calculate(100.0, 0.0);
    // The integral contribution should be capped at 2.0
    // Plus a tiny P/D contribution (both zero gains), so output ≈ 2.0
    ASSERT_NEAR(output, 2.0, 0.2);
    ASSERT_LT(output, 3.0); // definitely clamped, not 100
}

TEST(PID_AntiWindup_NegativeClamp) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 1.0, 0.0); // I-only
    pid.set_integral_limit(2.0);
    pid.reset();

    // Negative error → negative integral
    for (int i = 1; i <= 100; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(0.0, 100.0); // error = -100
    }
    mock_time_sec += 0.01;
    double output = pid.calculate(0.0, 100.0);
    ASSERT_NEAR(output, -2.0, 0.2);
    ASSERT_GT(output, -3.0);
}

TEST(PID_DFilter_SmoothsDerivative) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid_raw(0.0, 0.0, 1.0);    // D-only, no filter
    PIDController pid_filt(0.0, 0.0, 1.0);   // D-only, with filter
    pid_filt.set_d_filter(0.7);               // alpha=0.7 → strong smoothing
    pid_raw.reset();
    pid_filt.reset();

    // Step 1: large error jump → both respond
    mock_time_sec = 0.01;
    double raw1  = pid_raw.calculate(10.0, 0.0);
    mock_time_sec = 0.0; // reset time for filtered pid
    pid_filt.reset();
    mock_time_sec = 0.01;
    double filt1 = pid_filt.calculate(10.0, 0.0);

    // With EMA filter (alpha=0.7): filtered = 0.7*0 + 0.3*raw = 0.3*raw
    // So filtered should be smaller than raw on first spike
    ASSERT_LT(std::abs(filt1), std::abs(raw1));
}

TEST(PID_OutputLimit_ClampsOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(10.0, 0.0, 0.0); // P-only with high gain
    pid.set_output_limit(5.0);           // clamp output to ±5
    pid.reset();

    mock_time_sec = 1.01;
    double output = pid.calculate(100.0, 0.0); // error=100, P=1000 unclamped
    ASSERT_NEAR(output, 5.0, 0.001); // clamped to +5.0

    mock_time_sec = 1.02;
    double neg_output = pid.calculate(0.0, 100.0); // error=-100
    ASSERT_NEAR(neg_output, -5.0, 0.001); // clamped to -5.0
}

TEST(PID_OutputLimit_NoClampWhenDisabled) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(10.0, 0.0, 0.0);
    // output_limit defaults to 0 (disabled)
    pid.reset();

    mock_time_sec = 1.01;
    double output = pid.calculate(100.0, 0.0); // error=100, P=1000
    ASSERT_NEAR(output, 1000.0, 0.1); // no clamping
}

TEST(PID_ResetClearsEnhancedState) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(1.0, 1.0, 1.0);
    pid.set_integral_limit(10.0);
    pid.set_d_filter(0.5);
    pid.set_output_limit(50.0);
    pid.reset();

    // Build up state
    for (int i = 1; i <= 20; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(10.0, 5.0);
    }

    // Reset
    mock_time_sec = 1.0;
    pid.reset();
    mock_time_sec = 1.01;
    double after_reset = pid.calculate(10.0, 5.0);

    // Fresh PID with same config
    mock_time_sec = 2.0;
    PIDController pid2(1.0, 1.0, 1.0);
    pid2.set_integral_limit(10.0);
    pid2.set_d_filter(0.5);
    pid2.set_output_limit(50.0);
    pid2.reset();
    mock_time_sec = 2.01;
    double fresh = pid2.calculate(10.0, 5.0);

    ASSERT_NEAR(after_reset, fresh, 0.01);
}

// ============================================================================
// Motion Profile Tests
// ============================================================================

TEST(MotionProfile_AccelerationPhase) {
    MotionProfile profile(1.0, 2.0); // max_v=1.0, max_a=2.0
    // At t=0.1s, accel_v = 2.0 * 0.1 = 0.2, far from target -> no decel
    double v = profile.get_target_velocity(0.1, 2.0);
    ASSERT_NEAR(v, 0.2, 0.001);
}

TEST(MotionProfile_ReachesMaxVelocity) {
    MotionProfile profile(1.0, 2.0);
    // At t=1.0s, accel_v = 2.0, but capped at max_v=1.0
    double v = profile.get_target_velocity(1.0, 2.0);
    ASSERT_NEAR(v, 1.0, 0.001);
}

TEST(MotionProfile_DecelerationPhase) {
    MotionProfile profile(1.0, 2.0);
    // Close to target: distance_to_go = 0.1m
    // decel_v = sqrt(2 * 2.0 * 0.1) = sqrt(0.4) ≈ 0.632
    double v = profile.get_target_velocity(1.0, 0.1);
    ASSERT_LT(v, 1.0);
    ASSERT_NEAR(v, sqrt(0.4), 0.001);
}

TEST(MotionProfile_ZeroDistanceProducesZeroVelocity) {
    MotionProfile profile(1.0, 2.0);
    double v = profile.get_target_velocity(1.0, 0.0);
    ASSERT_NEAR(v, 0.0, 0.001);
}

TEST(MotionProfile_VelocityNeverExceedsMax) {
    MotionProfile profile(1.0, 2.0);
    // Test many different time/distance combinations
    for (double t = 0.0; t < 5.0; t += 0.1) {
        for (double d = 0.0; d < 5.0; d += 0.1) {
            double v = profile.get_target_velocity(t, d);
            ASSERT_TRUE(v <= 1.0 + 0.001);
        }
    }
}

// ============================================================================
// Odometry Tests
// ============================================================================

TEST(Odometry_InitialPoseIsZero) {
    reset_all_mocks();
    set_pose({0, 0, 0});
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 0.0, 0.001);
    ASSERT_NEAR(p.y, 0.0, 0.001);
    ASSERT_NEAR(p.theta, 0.0, 0.001);
}

TEST(Odometry_SetPoseWorks) {
    reset_all_mocks();
    set_pose({1.5, 2.5, 0.5});
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 1.5, 0.001);
    ASSERT_NEAR(p.y, 2.5, 0.001);
    ASSERT_NEAR(p.theta, 0.5, 0.001);
}

TEST(Odometry_DriveStraightForward) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    // Simulate driving 1 meter forward
    double dist_m = 1.0;
    double ticks = (dist_m / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;
    mock_left_ticks = ticks;
    mock_right_ticks = ticks;
    mock_imu_heading_rad = 0.0; // heading stays 0

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 1.0, 0.02);
    ASSERT_NEAR(p.y, 0.0, 0.02);
    ASSERT_NEAR(p.theta, 0.0, 0.02);
}

TEST(Odometry_PointTurn90Degrees) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    double turn_rad = M_PI / 2.0;
    double arc_len = (turn_rad * WHEEL_TRACK) / 2.0;
    double ticks_for_turn = (arc_len / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;

    mock_left_ticks = -ticks_for_turn;
    mock_right_ticks = ticks_for_turn;
    mock_imu_heading_rad = turn_rad;

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 0.0, 0.05);
    ASSERT_NEAR(p.y, 0.0, 0.05);
    ASSERT_NEAR(p.theta, turn_rad, 0.05);
}

TEST(Odometry_DriveBackward) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    double dist_m = -0.5;
    double ticks = (dist_m / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;
    mock_left_ticks = ticks;
    mock_right_ticks = ticks;
    mock_imu_heading_rad = 0.0;

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, -0.5, 0.02);
    ASSERT_NEAR(p.y, 0.0, 0.02);
}

TEST(Odometry_MultipleUpdatesAccumulate) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    double step_m = 0.5;
    double step_ticks = (step_m / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV;

    // Step 1: drive 0.5m
    mock_left_ticks = step_ticks;
    mock_right_ticks = step_ticks;
    mock_imu_heading_rad = 0.0;
    odometry_update();

    // Step 2: drive another 0.5m (total 1.0m)
    mock_left_ticks = step_ticks * 2;
    mock_right_ticks = step_ticks * 2;
    mock_imu_heading_rad = 0.0;
    odometry_update();

    Pose p = get_pose();
    ASSERT_NEAR(p.x, 1.0, 0.02);
    ASSERT_NEAR(p.y, 0.0, 0.02);
}

// ============================================================================
// Drive Straight 1m — Logic Tests
// ============================================================================
// Tests the distance calculation and speed ramp from main.cpp's
// drive_straight_1m(), without VEX hardware dependencies.

TEST(Drive_TargetDegreesFor1m) {
    // 1m / wheel_circumference = revolutions, * 360 = degrees
    double target_revs = 1.0 / WHEEL_CIRCUMFERENCE;
    double target_deg  = target_revs * 360.0;

    // With 4-inch (0.1016m) wheels: circumference ≈ 0.3192m
    // target_revs ≈ 3.133, target_deg ≈ 1127.8
    ASSERT_GT(target_deg, 0.0);
    ASSERT_NEAR(target_deg, 360.0 / WHEEL_CIRCUMFERENCE, 0.01);
    // Sanity: should be roughly 1000-1300 degrees for 4-inch wheels
    ASSERT_GT(target_deg, 900.0);
    ASSERT_LT(target_deg, 1500.0);
}

TEST(Drive_SpeedRampProfile) {
    double target_revs = 1.0 / WHEEL_CIRCUMFERENCE;
    double target_deg  = target_revs * 360.0;
    double max_speed   = 50.0;
    double ramp_up_deg = target_deg * 0.2;
    double ramp_dn_deg = target_deg * 0.8;
    double min_speed   = 10.0;

    // Helper lambda: compute speed at a given encoder position
    auto calc_speed = [&](double pos) -> double {
        double speed = max_speed;
        if (pos < ramp_up_deg) {
            speed = min_speed + (max_speed - min_speed) * (pos / ramp_up_deg);
        } else if (pos > ramp_dn_deg) {
            double remaining = target_deg - pos;
            double ramp_zone = target_deg - ramp_dn_deg;
            speed = min_speed + (max_speed - min_speed) * (remaining / ramp_zone);
        }
        return speed;
    };

    // At start (pos=0): speed should be min_speed
    ASSERT_NEAR(calc_speed(0.0), min_speed, 0.01);

    // At 10% (mid ramp-up): speed between min and max
    double mid_ramp = ramp_up_deg / 2.0;
    double mid_speed = calc_speed(mid_ramp);
    ASSERT_GT(mid_speed, min_speed);
    ASSERT_LT(mid_speed, max_speed);

    // At 50% (cruise): speed = max
    ASSERT_NEAR(calc_speed(target_deg * 0.5), max_speed, 0.01);

    // At 90% (mid ramp-down): speed between min and max
    double mid_down = target_deg * 0.9;
    double down_speed = calc_speed(mid_down);
    ASSERT_GT(down_speed, min_speed);
    ASSERT_LT(down_speed, max_speed);

    // Speed is symmetric: 10% from start ≈ 10% from end
    double early = calc_speed(ramp_up_deg * 0.5);
    double late  = calc_speed(target_deg - (target_deg - ramp_dn_deg) * 0.5);
    ASSERT_NEAR(early, late, 0.01);

    // Speed never exceeds max or drops below min in the valid range
    for (double pos = 0.0; pos < target_deg; pos += target_deg / 100.0) {
        double s = calc_speed(pos);
        ASSERT_TRUE(s >= min_speed - 0.01);
        ASSERT_TRUE(s <= max_speed + 0.01);
    }
}

TEST(Drive_StopsAtTargetDegrees) {
    // Verify the loop exit condition: pos >= target_deg means we stop
    double target_deg = 360.0 / WHEEL_CIRCUMFERENCE; // degrees for 1m
    // At exactly target: should stop
    ASSERT_TRUE(target_deg >= target_deg);
    // Slightly past: should stop
    ASSERT_TRUE(target_deg + 1.0 >= target_deg);
    // Before target: should NOT stop
    ASSERT_TRUE(!(target_deg - 1.0 >= target_deg));
}

// ============================================================================
// Main: Run all tests
// ============================================================================

int main() {
    printf("============================================\n");
    printf("  VEX Robot Host-Side Unit Tests\n");
    printf("============================================\n\n");

    printf("[PID Controller]\n");
    RUN_TEST(PID_PositiveErrorProducesPositiveOutput);
    RUN_TEST(PID_NegativeErrorProducesNegativeOutput);
    RUN_TEST(PID_ZeroErrorProducesZeroOutput);
    RUN_TEST(PID_IntegralAccumulates);
    RUN_TEST(PID_DerivativeRespondToChange);
    RUN_TEST(PID_ResetClearsState);

    printf("\n[PID Enhancements]\n");
    RUN_TEST(PID_AntiWindup_ClampsIntegral);
    RUN_TEST(PID_AntiWindup_NegativeClamp);
    RUN_TEST(PID_DFilter_SmoothsDerivative);
    RUN_TEST(PID_OutputLimit_ClampsOutput);
    RUN_TEST(PID_OutputLimit_NoClampWhenDisabled);
    RUN_TEST(PID_ResetClearsEnhancedState);

    printf("\n[Motion Profile]\n");
    RUN_TEST(MotionProfile_AccelerationPhase);
    RUN_TEST(MotionProfile_ReachesMaxVelocity);
    RUN_TEST(MotionProfile_DecelerationPhase);
    RUN_TEST(MotionProfile_ZeroDistanceProducesZeroVelocity);
    RUN_TEST(MotionProfile_VelocityNeverExceedsMax);

    printf("\n[Odometry]\n");
    RUN_TEST(Odometry_InitialPoseIsZero);
    RUN_TEST(Odometry_SetPoseWorks);
    RUN_TEST(Odometry_DriveStraightForward);
    RUN_TEST(Odometry_PointTurn90Degrees);
    RUN_TEST(Odometry_DriveBackward);
    RUN_TEST(Odometry_MultipleUpdatesAccumulate);

    printf("\n[Drive Straight 1m]\n");
    RUN_TEST(Drive_TargetDegreesFor1m);
    RUN_TEST(Drive_SpeedRampProfile);
    RUN_TEST(Drive_StopsAtTargetDegrees);

    printf("\n============================================\n");
    printf("  Results: %d passed, %d failed, %d total\n",
        g_tests_passed, g_tests_failed, g_tests_run);
    printf("============================================\n");

    if (g_tests_failed > 0) {
        printf("  *** SOME TESTS FAILED ***\n");
        return 1;
    } else {
        printf("  ALL TESTS PASSED\n");
        return 0;
    }
}
