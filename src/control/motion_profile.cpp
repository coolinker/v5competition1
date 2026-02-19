// ============================================================================
//  control/motion_profile.cpp — 梯形速度规划器的实现
// ============================================================================
//
//  【核心思想】每个时刻的目标速度是三个约束中最小的那个：
//
//    1. 加速限制：v = a × t
//       → 不能一下子从 0 加速到最大速度，要慢慢来
//       → 就像汽车起步时慢慢踩油门
//
//    2. 最大速度：v = v_max
//       → 再怎么加速也不能超过限速
//
//    3. 减速限制：v = √(2 × a × d)
//       → 快到终点时必须减速，不然刹不住！
//       → 剩余距离 d 越小，允许的速度越低
//       → 这个公式来自物理学：v² = 2ad（匀减速运动）
//
//  取三者中最小的值，速度曲线自然就变成梯形了！
//
// ============================================================================
#include "control/motion_profile.h"
#include <cmath>       // sqrt() 平方根函数
#include <algorithm>   // std::min() 取最小值函数

// 构造函数：记住最大速度和最大加速度
MotionProfile::MotionProfile(double max_v, double max_a)
    : _max_velocity(max_v), _max_acceleration(max_a) {}

// ---- 计算当前时刻的目标速度 ----
double MotionProfile::get_target_velocity(double time_elapsed, double distance_to_go) {
    // 约束 1：加速段——从零开始，每秒增加 _max_acceleration 的速度
    double accel_v = _max_acceleration * time_elapsed;

    // 约束 2：减速段——要在剩余距离内停下来
    //   从物理公式 v² = 2·a·d 推导出 v = √(2·a·d)
    //   std::abs() 取绝对值，防止负距离导致 sqrt 出错
    double decel_v = sqrt(2.0 * _max_acceleration * std::abs(distance_to_go));

    // 约束 3：最大速度
    // std::min({...}) 从三个值里取最小的——最保守的那个约束决定实际速度
    return std::min({_max_velocity, accel_v, decel_v});
}
