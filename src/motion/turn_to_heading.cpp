// ============================================================================
//  motion/turn_to_heading.cpp — 原地转向控制器的实现
// ============================================================================
//
//  【做什么】让机器人原地旋转到指定的朝向角度。
//
//  【原理】
//    1. 计算"我现在朝哪" 和 "我要朝哪" 之间的差值（航向误差）
//    2. 用 PID 控制器决定转弯力度
//    3. 差速驱动：左边往后转、右边往前转（或反过来）→ 原地旋转
//    4. 持续足够时间误差很小 → 认为到位
//
//    关键技巧：航向误差归一化
//      角度是"循环"的——350° 和 10° 之间，差值应该是 20°，不是 -340°。
//      用 atan2(sin(差值), cos(差值)) 可以自动归一化到 [-180°, +180°]。
//      这样机器人永远走最短的旋转路径！
//
// ============================================================================
#include "motion/turn_to_heading.h"
#include "config.h"
#include "control/pid.h"
#include "hal/motors.h"
#include "hal/time.h"
#include "localization/odometry.h"
#include <cmath>

// 模块级 PID 控制器 —— 用 static 让它只在这个文件可见
// 为什么放在函数外面？因为 drive_to_pose 也会通过下面
// 的 turn_to_heading_pid_calculate() 函数借用它来计算角度修正。
static PIDController turn_pid(TURN_KP, TURN_KI, TURN_KD);

// 暴露给 drive_to_pose.cpp 使用的公共接口
// 输入航向误差，输出应该施加的旋转力度
double turn_to_heading_pid_calculate(double error) {
    return turn_pid.calculate(0.0, -error);
}

void turn_to_heading(double target_heading_rad) {
    // 每次新的转弯任务开始时重置 PID（清除积分和上次误差）
    turn_pid.reset();
    turn_pid.set_integral_limit(TURN_INTEGRAL_LIMIT);
    turn_pid.set_d_filter(TURN_D_FILTER);
    turn_pid.set_output_limit(12.0);  // 最大输出 ±12V（电机极限）

    unsigned long settle_start = 0;   // 开始"到位计时"的时刻
    bool settling = false;            // 是否正在到位计时
    unsigned long start_time = get_time_ms();

    // ---- 主控制循环 ----
    while (true) {
        // 超时保护：不管什么原因没转到位，到时间就强制退出
        unsigned long elapsed = get_time_ms() - start_time;
        if (elapsed > TURN_TIMEOUT_MS) break;

        // 读取当前位姿（其中 theta 是当前朝向）
        Pose current = get_pose();

        // 计算航向误差并归一化到 [-π, π]
        // ← 这个公式确保机器人永远走"近路"而不是绕大圈
        double error = target_heading_rad - current.theta;
        error = atan2(sin(error), cos(error));  // 归一化！

        // ── 到位检测 ──
        // 误差绝对值小于阈值？开始计时。持续够久？认为到位！
        if (std::abs(error) < TURN_SETTLE_RAD) {
            if (!settling) {
                settling = true;
                settle_start = get_time_ms();
            } else if (get_time_ms() - settle_start >= TURN_SETTLE_TIME_MS) {
                break;  // 转到位了！退出循环
            }
        } else {
            settling = false;  // 又偏了，重新计时
        }

        // 用 PID 控制器计算旋转力度（ω）
        double omega = turn_to_heading_pid_calculate(error);

        // 差速驱动实现原地转弯：
        //   左轮速度 = −ω × (轮距/2)
        //   右轮速度 = +ω × (轮距/2)
        // ω > 0 时：左轮后退、右轮前进 → 逆时针转
        // ω < 0 时：左轮前进、右轮后退 → 顺时针转
        double left_v  = -omega * WHEEL_TRACK / 2.0;
        double right_v =  omega * WHEEL_TRACK / 2.0;
        set_drive_motors(left_v, right_v);

        wait_ms(LOOP_INTERVAL_MS);  // 等一个控制周期
    }

    stop_drive_motors();  // 刹停
}
