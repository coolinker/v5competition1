// ============================================================================
//  hal/imu.cpp — 惯性传感器 (IMU) 的实现
// ============================================================================
//
//  这里是 IMU（陀螺仪）接口的具体实现。
//  主要做两件事：
//  1. 读取角度并转换成弧度（程序内部统一用弧度）
//  2. 校准 IMU（开机时必须在机器人静止时执行一次）
//
// ============================================================================
#include "hal/imu.h"
#include "vex.h"
#include <cmath>       // M_PI（圆周率 π ≈ 3.14159）
#include "hal/hal_log.h"

// IMU 传感器对象在 main.cpp 里创建，这里用 extern 声明"借用"它
extern vex::inertial DrivetrainInertial;

// ---- 获取当前航向角（弧度） ----
// VEX 返回的是"度"（0~360），我们乘以 π/180 转成弧度
double get_imu_heading_rad() {
    return DrivetrainInertial.heading(vex::rotationUnits::deg) * M_PI / 180.0;
}

// ---- 获取累计旋转量（弧度） ----
// heading 只在 0~360 之间，但 rotation 可以一直累加
// 比如转了两圈，heading 回到 0°，但 rotation 显示 720°
double get_imu_rotation_rad() {
    return DrivetrainInertial.rotation(vex::rotationUnits::deg) * M_PI / 180.0;
}

// ---- 重置 IMU ----
void reset_imu() {
    DrivetrainInertial.resetRotation();  // 累计旋转归零
    DrivetrainInertial.resetHeading();   // 当前航向归零
    hal_log("IMU reset");
}

// ---- 校准 IMU ----
// 校准时陀螺仪要测量"零漂"（静止时的读数偏差），所以机器人必须一动不动！
// 如果 3 秒内没校准完（可能传感器没插好），就超时退出并打印警告。
void calibrate_imu() {
    DrivetrainInertial.calibrate();       // 启动校准
    hal_log("IMU calibration started");

    // 每 50 毫秒检查一次校准是否完成，最多等 3 秒
    int elapsed = 0;
    while (DrivetrainInertial.isCalibrating() && elapsed < 3000) {
        vex::task::sleep(50);  // 等 50ms
        elapsed += 50;         // 累计已等时间
    }

    if (elapsed >= 3000) {
        // 超时了！可能传感器没连好
        hal_log("IMU calibration TIMEOUT — sensor may not be connected");
    } else {
        hal_log("IMU calibration finished");
    }
}
