// ============================================================================
//  motion/drive_to_pose.cpp — Boomerang 弧线控制器的实现
// ============================================================================
//
//  【Boomerang 算法原理】
//
//    在目标点前方沿目标航向反方向放一个"胡萝卜"引导点（carrot）。
//    机器人始终朝着引导点开；随着靠近目标，引导点逐渐收敛到目标点上。
//    这样机器人会走出一条平滑的弧线，最终以正确的角度到达目标。
//
//           胡萝卜 ◄── lead 距离 ── 目标点
//             ╱                       ↑ 目标朝向 θ
//          机器人
//
//    想象一下：你要把车停进一个车位（面朝特定方向），
//    你不会直接冲过去然后原地掉头——你会走一个优雅的弧线。
//    Boomerang 就是让机器人自动计算这条弧线！
//
//  【退出条件】
//    ① 距离目标 < DRIVE_SETTLE_M 持续 DRIVE_SETTLE_TIME_MS → 成功到达
//    ② 运行时间 > DRIVE_TIMEOUT_MS → 超时退出（防止卡死）
//
// ============================================================================
#include "motion/drive_to_pose.h"
#include "config.h"
#include "control/pid.h"
#include "hal/motors.h"
#include "hal/time.h"
#include "localization/odometry.h"
#include <cmath>

void drive_to_pose(const Pose& target_pose, bool reverse) {
    // 创建角度 PID 控制器（用来修正航向偏差）
    PIDController angular_pid(TURN_KP, TURN_KI, TURN_KD);
    angular_pid.set_integral_limit(TURN_INTEGRAL_LIMIT);
    angular_pid.set_d_filter(TURN_D_FILTER);
    angular_pid.set_output_limit(12.0);  // 最大输出 ±12V
    angular_pid.reset();

    unsigned long start_time = get_time_ms();
    unsigned long settle_start = 0;   // 开始"到位计时"的时刻
    bool settling = false;            // 是否正在到位计时中
    double prev_cmd_v = 0.0;          // 上一次的速度命令（用于加速度限幅）

    // ---- 主控制循环 ----
    while (true) {
        // 超时检测
        if (get_time_ms() - start_time > DRIVE_TIMEOUT_MS) break;

        // 读取当前位姿
        Pose cur = get_pose();

        // 计算到目标的距离
        double dx = target_pose.x - cur.x;
        double dy = target_pose.y - cur.y;
        double dist = sqrt(dx * dx + dy * dy);  // 勾股定理

        // ── 到位检测 ──
        // 如果距离目标足够近，开始计时；如果持续够久，就认为到达了
        if (dist < DRIVE_SETTLE_M) {
            if (!settling) {
                settling = true;
                settle_start = get_time_ms();
            } else if (get_time_ms() - settle_start >= DRIVE_SETTLE_TIME_MS) {
                break;  // 到位了！退出循环
            }
        } else {
            settling = false;  // 又跑远了，重新开始计时
        }

        // ── Boomerang 引导点（"胡萝卜"）计算 ──
        // 引导点在目标前方 lead × dist 的位置（沿目标航向反方向）
        // 离目标越远，引导点越远；接近目标时引导点收敛到目标本身
        double carrot_x = target_pose.x - BOOMERANG_LEAD * dist * cos(target_pose.theta);
        double carrot_y = target_pose.y - BOOMERANG_LEAD * dist * sin(target_pose.theta);

        // 计算机器人应该朝向引导点的方向
        double target_heading = atan2(carrot_y - cur.y, carrot_x - cur.x);
        if (reverse) target_heading += M_PI;  // 倒车：方向反转 180°

        // 计算航向误差（归一化到 [-π, π]，防止绕远路）
        // 比如当前朝向 350°，目标 10°，误差应该是 +20° 而不是 -340°
        double heading_error = atan2(sin(target_heading - cur.theta),
                                     cos(target_heading - cur.theta));

        // ── 线速度计算 ──
        // 减速限制：v = √(2 × a × d)  ← 物理公式，确保能刹住
        double decel_v = sqrt(2.0 * MAX_ACCELERATION * dist);
        double raw_v = (decel_v < MAX_VELOCITY) ? decel_v : MAX_VELOCITY;

        // 余弦节流：如果航向偏差大（比如要往左开但机器人朝右），
        // 就降低线速度，先转对方向再加速。
        // cos(heading_error) 在误差=0时=1（全速），误差=90°时=0（停下来转）
        double cos_err = cos(heading_error);
        if (cos_err < 0.0) cos_err = 0.0;  // 误差超过 90° 时直接停下来转
        raw_v *= cos_err;
        if (reverse) raw_v = -raw_v;  // 倒车速度取负

        // 加速度限幅：防止突然加速或减速（保护机构 + 防止轮子打滑）
        double max_dv = MAX_ACCELERATION * (LOOP_INTERVAL_MS / 1000.0);
        if (raw_v - prev_cmd_v >  max_dv) raw_v = prev_cmd_v + max_dv;
        if (prev_cmd_v - raw_v >  max_dv) raw_v = prev_cmd_v - max_dv;
        prev_cmd_v = raw_v;

        // ── 角速度计算（PID） ──
        // 用 PID 控制器计算应该给多大的转弯力度
        double omega = angular_pid.calculate(0.0, -heading_error);

        // ── 差速驱动 ──
        // 差速原理：左右电机速度不同就能转弯
        //   直行：左=右
        //   左转：左<右（左轮慢，右轮快）
        //   右转：左>右（左轮快，右轮慢）
        double left_v  = raw_v - omega * WHEEL_TRACK / 2.0;
        double right_v = raw_v + omega * WHEEL_TRACK / 2.0;
        set_drive_motors(left_v, right_v);

        wait_ms(LOOP_INTERVAL_MS);  // 等待一个控制周期
    }

    stop_drive_motors();  // 循环结束，刹停
}
