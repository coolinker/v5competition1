// ============================================================================
//  localization/odometry.cpp — 垂直双轮里程计 + IMU 角度
// ============================================================================
//
//  【算法原理——垂直双轮方案】
//
//    与平行双轮不同，垂直双轮可以同时测量前后和左右两个方向的位移：
//
//    第 1 步：读取传感器
//      • 纵向轮 → 这次前后走了多远（Δforward）
//      • 横向轮 → 这次左右滑了多远（Δlateral）
//      • IMU    → 这次转了多少角度（Δθ）
//
//    第 2 步：补偿旋转引起的假位移
//      当机器人旋转时，偏离旋转中心的轮子会画弧线。
//      这段弧线不是真正的前后/左右平移，需要减掉：
//        Δforward_corrected = Δforward - FORWARD_WHEEL_OFFSET × Δθ
//        Δlateral_corrected = Δlateral - LATERAL_WHEEL_OFFSET × Δθ
//
//    第 3 步：转换到全局坐标
//      用中点近似法，把机器人坐标系的位移转到场地坐标系：
//        x += Δforward × cos(θ+Δθ/2) - Δlateral × sin(θ+Δθ/2)
//        y += Δforward × sin(θ+Δθ/2) + Δlateral × cos(θ+Δθ/2)
//        θ += Δθ
//
//    以 100Hz（每秒 100 次）在后台线程中不断重复以上 3 步。
//
// ============================================================================
#include "localization/odometry.h"
#include "config.h"
#include "hal/motors.h"
#include "hal/imu.h"
#include "hal/hal_log.h"
#include "hal/tracking_wheels.h"
#include "vex.h"
#include <cmath>

// ---- 线程安全的位姿变量 ----
static vex::mutex pose_mutex;
static Pose current_pose = {0.0, 0.0, 0.0};

// ---- 上一次的传感器读数（用于计算增量）----
static double prev_forward_dist  = 0.0;   // 上一次纵向轮累计距离
static double prev_lateral_dist  = 0.0;   // 上一次横向轮累计距离
static double prev_imu_rotation  = 0.0;   // 上一次 IMU 累计旋转量

// ---- 后台任务 ----
static vex::task* odom_task_ptr = nullptr;

static int odometry_task_fn() {
    while (true) {
        odometry_update();
        vex::task::sleep(LOOP_INTERVAL_MS);
    }
    return 0;
}

void odometry_start_task() {
    if (odom_task_ptr == nullptr) {
        odom_task_ptr = new vex::task(odometry_task_fn);
        hal_log("Odometry task started (100 Hz, perpendicular tracking wheels)");
    }
}

void odometry_stop_task() {
    if (odom_task_ptr != nullptr) {
        odom_task_ptr->stop();
        delete odom_task_ptr;
        odom_task_ptr = nullptr;
        hal_log("Odometry task stopped");
    }
}

// ---- 核心：一次里程计更新 ----
void odometry_update() {
    // 第 1 步：读取传感器当前累计值，然后算出增量
    double fwd_dist = tracking_get_forward_distance_m();
    double lat_dist = tracking_get_lateral_distance_m();
    double d_forward = fwd_dist - prev_forward_dist;   // 纵向轮这一步走了多远
    double d_lateral = lat_dist - prev_lateral_dist;    // 横向轮这一步滑了多远
    prev_forward_dist = fwd_dist;
    prev_lateral_dist = lat_dist;

    // 从 IMU 读取旋转角度增量
    double imu_rotation = get_imu_rotation_rad();
    double dtheta = imu_rotation - prev_imu_rotation;
    prev_imu_rotation = imu_rotation;

    // 第 2 步：补偿旋转引起的假位移
    //   当机器人转动 Δθ 时，偏移旋转中心的轮子会画弧线，
    //   弧长 = 偏移量 × Δθ，这不是真正的平移，需要减掉。
    double d_fwd_corrected = d_forward - FORWARD_WHEEL_OFFSET * dtheta;
    double d_lat_corrected = d_lateral - LATERAL_WHEEL_OFFSET * dtheta;

    // 第 3 步：从机器人坐标系转换到场地全局坐标系
    pose_mutex.lock();
    double mid_theta = current_pose.theta + dtheta / 2.0;
    //   纵向位移沿机器人前方，横向位移沿机器人右方
    //   注意：场地坐标系 y 轴朝左，所以横向向右为负 y
    current_pose.x     += d_fwd_corrected * cos(mid_theta) - d_lat_corrected * sin(mid_theta);
    current_pose.y     += d_fwd_corrected * sin(mid_theta) + d_lat_corrected * cos(mid_theta);
    current_pose.theta += dtheta;
    pose_mutex.unlock();
}

Pose get_pose() {
    pose_mutex.lock();
    Pose copy = current_pose;
    pose_mutex.unlock();
    return copy;
}

void set_pose(const Pose& new_pose) {
    pose_mutex.lock();
    current_pose       = new_pose;
    prev_forward_dist  = 0;
    prev_lateral_dist  = 0;
    prev_imu_rotation  = 0.0;
    pose_mutex.unlock();

    reset_encoders();
    reset_imu();
    tracking_wheels_reset();
}

void set_pose_no_reset(const Pose& new_pose) {
    pose_mutex.lock();
    current_pose = new_pose;
    pose_mutex.unlock();
}
