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
