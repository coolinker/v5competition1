#include "test/test_odometry.h"
#include "test/test_runner.h"
#include "localization/odometry.h"
#include "config.h"
#include <cmath>

// Mock HAL functions for testing
static double mock_left_ticks = 0;
static double mock_right_ticks = 0;
static double mock_imu_rad = 0;

double get_left_encoder_ticks() { return mock_left_ticks; }
double get_right_encoder_ticks() { return mock_right_ticks; }
double get_imu_heading_rad() { return mock_imu_rad; }
void reset_encoders() { mock_left_ticks = 0; mock_right_ticks = 0; }
void reset_imu() { mock_imu_rad = 0; }
void set_drive_motors(double l, double r) {} // No-op

namespace test {
    void test_odometry_logic() {
        // Test 1: Drive straight
        set_pose({0, 0, 0});
        double dist_m = 1.0;
        double ticks_for_dist = (dist_m / (M_PI * WHEEL_DIAMETER)) * TICKS_PER_REV;
        mock_left_ticks = ticks_for_dist;
        mock_right_ticks = ticks_for_dist;
        odometry_update();
        Pose pose1 = get_pose();
        assert_equal(pose1.x, 1.0, 0.01, "Odom should track straight distance in X");
        assert_equal(pose1.y, 0.0, 0.01, "Odom should not change Y when driving straight");
        assert_equal(pose1.theta, 0.0, 0.01, "Odom should not change theta when driving straight");

        // Test 2: Point turn
        set_pose({0, 0, 0});
        double turn_rad = M_PI / 2.0; // 90 degrees
        double arc_len = (turn_rad * WHEEL_TRACK) / 2.0;
        double ticks_for_turn = (arc_len / (M_PI * WHEEL_DIAMETER)) * TICKS_PER_REV;
        mock_left_ticks = -ticks_for_turn;
        mock_right_ticks = ticks_for_turn;
        mock_imu_rad = turn_rad; // Assume IMU is perfect for this test
        odometry_update();
        Pose pose2 = get_pose();
        assert_equal(pose2.x, 0.0, 0.01, "Odom should not change X on point turn");
        assert_equal(pose2.y, 0.0, 0.01, "Odom should not change Y on point turn");
        assert_equal(pose2.theta, turn_rad, 0.01, "Odom should track point turn angle");
    }
}
