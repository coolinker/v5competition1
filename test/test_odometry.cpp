#include "test/test_odometry.h"
#include "test/test_runner.h"
#include "localization/odometry.h"
#include "config.h"
#include <cmath>

// Mock HAL functions for testing (tracking wheel based odometry)
static double mock_tracking_left = 0;
static double mock_tracking_right = 0;
static double mock_imu_rotation = 0;
static double mock_left_ticks = 0;
static double mock_right_ticks = 0;

double tracking_get_left_distance_m()  { return mock_tracking_left; }
double tracking_get_right_distance_m() { return mock_tracking_right; }
double get_imu_rotation_rad()          { return mock_imu_rotation; }
double get_left_encoder_ticks()        { return mock_left_ticks; }
double get_right_encoder_ticks()       { return mock_right_ticks; }
void   tracking_wheels_init()          { }
void   tracking_wheels_reset()         { mock_tracking_left = 0; mock_tracking_right = 0; }
void   reset_encoders()                { mock_left_ticks = 0; mock_right_ticks = 0; }
void   reset_imu()                     { mock_imu_rotation = 0; }
void   set_drive_motors(double, double) {}

namespace test {
    void test_odometry_logic() {
        // Test 1: Drive straight 1m via tracking wheels
        set_pose({0, 0, 0});
        mock_tracking_left  = 1.0;
        mock_tracking_right = 1.0;
        mock_imu_rotation   = 0.0;
        odometry_update();
        Pose pose1 = get_pose();
        assert_equal(pose1.x, 1.0, 0.02, "Odom should track straight distance in X");
        assert_equal(pose1.y, 0.0, 0.02, "Odom should not change Y when driving straight");
        assert_equal(pose1.theta, 0.0, 0.02, "Odom should not change theta when driving straight");

        // Test 2: Point turn 90 degrees
        set_pose({0, 0, 0});
        double turn_rad = M_PI / 2.0;
        double arc_len = (turn_rad * TRACKING_WHEEL_TRACK) / 2.0;
        mock_tracking_left  = -arc_len;
        mock_tracking_right =  arc_len;
        mock_imu_rotation   = turn_rad;
        odometry_update();
        Pose pose2 = get_pose();
        assert_equal(pose2.x, 0.0, 0.05, "Odom should not change X on point turn");
        assert_equal(pose2.y, 0.0, 0.05, "Odom should not change Y on point turn");
        assert_equal(pose2.theta, turn_rad, 0.05, "Odom should track point turn angle");
    }
}
