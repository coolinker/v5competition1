// ============================================================================
//  hal/tracking_wheels.cpp — 追踪轮的实现（垂直双轮方案）
// ============================================================================
//
//  【原理：怎么把"轮子转了多少度"变成"走了多远"？】
//
//    想象一下：一个直径 2.75 英寸的轮子转一整圈（360°），
//    地上滚过的距离正好等于轮子的周长 = π × 直径。
//    如果只转了 90°（四分之一圈），就滚了周长的 1/4。
//
//    公式：距离 = (转过的角度 / 360°) × 周长
//
//  【垂直双轮布局】
//    纵向轮（Forward）：朝机器人前进方向安装，测量前后位移
//    横向轮（Lateral）：垂直于前进方向安装，测量左右侧滑
//
// ============================================================================
#include "hal/tracking_wheels.h"
#include "config.h"
#include "vex.h"
#include "hal/hal_log.h"

// 旋转传感器对象在 main.cpp 里创建，这里用 extern "借用"
extern vex::rotation ForwardTrackingSensor;
extern vex::rotation LateralTrackingSensor;

// ---- 初始化 ----
void tracking_wheels_init() {
    ForwardTrackingSensor.resetPosition();  // 纵向传感器读数归零
    LateralTrackingSensor.resetPosition();  // 横向传感器读数归零
    hal_log("Tracking wheels initialized (perpendicular layout)");
}

// ---- 重置读数 ----
void tracking_wheels_reset() {
    ForwardTrackingSensor.resetPosition();
    LateralTrackingSensor.resetPosition();
}

// ---- 纵向追踪轮距离（米） ----
// 读取传感器返回的"转了多少度"，然后用公式转换成距离
// 正数 = 向前，负数 = 向后
double tracking_get_forward_distance_m() {
    double degrees = ForwardTrackingSensor.position(vex::rotationUnits::deg);
    // 距离 = (度数 / 360) × 周长
    return (degrees / 360.0) * TRACKING_WHEEL_CIRCUMFERENCE;
}

// ---- 横向追踪轮距离（米） ----
// 正数 = 向右，负数 = 向左
double tracking_get_lateral_distance_m() {
    double degrees = LateralTrackingSensor.position(vex::rotationUnits::deg);
    return (degrees / 360.0) * TRACKING_WHEEL_CIRCUMFERENCE;
}

// ---- 检查传感器连接状态 ----
// 两个都连好了才返回 true
bool tracking_wheels_connected() {
    return ForwardTrackingSensor.installed() && LateralTrackingSensor.installed();
}
