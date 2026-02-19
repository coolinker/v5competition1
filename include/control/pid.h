#pragma once
// ============================================================================
//  control/pid.h — PID 控制器（让机器人精准到达目标的"大脑"）
// ============================================================================
//
//  【什么是 PID？用骑自行车来理解！】
//
//    想象你骑自行车，要停在马路对面的一棵树前：
//
//    P（比例/Proportional）= 看到离树还远，就用力蹬；离得近了就轻轻蹬
//      → 距离越远，输出越大。就像"离目标远就使劲，近了就轻一点"
//
//    I（积分/Integral）= 如果轻轻蹬了半天还差一点点到不了（比如有上坡），
//      就慢慢加力直到真正到了
//      → 消除持续的小误差。就像"一直差一点？那我再加点力！"
//
//    D（微分/Derivative）= 你发现车速太快快要冲过头了，就捏刹车减速
//      → 防止冲过头。就像"速度太快？赶紧刹车！"
//
//    合起来的公式：
//      输出 = Kp×误差 + Ki×误差的累积 + Kd×误差的变化速度
//
//  【怎么用？】
//    PIDController pid(2.0, 0.0, 0.1);    // 创建，设定三个增益系数
//    pid.reset();                           // 每次新运动前重置
//    double output = pid.calculate(目标值, 当前值);  // 每次循环调用一次
//
//  【增强功能（默认关闭，需要时手动开启）】
//    • 积分限幅 (Anti-windup)  — 防止 I 项累积太大导致失控
//    • D 项低通滤波 (EMA)     — 平滑 D 项，减少噪声引起的抖动
//    • 输出限幅               — 把输出值限制在安全范围内
//
// ============================================================================

class PIDController {
public:
    /// 创建 PID 控制器，需要指定三个增益系数
    /// @param kp  比例增益（越大，离目标远时越用力）
    /// @param ki  积分增益（越大，消除小误差越快，但容易震荡）
    /// @param kd  微分增益（越大，刹车力度越大，但可能反应过度）
    PIDController(double kp, double ki, double kd);

    /// 计算一次 PID 输出
    /// @param setpoint  目标值（你想让机器人到达的位置/角度）
    /// @param pv        当前值（机器人现在的位置/角度）
    /// @return 修正输出（给电机的电压或转速指令）
    double calculate(double setpoint, double pv);

    /// 重置控制器（清除积分累积和导数状态）
    /// 每次新运动开始前必须调用！不然上一次的累积数据会影响这次
    void reset();

    // ---- 增强功能开关 ----
    /// 设定积分限幅（防止 I 项累积太大）
    /// 比如 set_integral_limit(5.0) 表示积分值最多 ±5
    void set_integral_limit(double limit);

    /// 设定 D 项低通滤波系数（0=不滤波, 0.5~0.8=典型值）
    /// 滤波后 D 项会更平滑，不容易被传感器噪声干扰
    void set_d_filter(double alpha);

    /// 设定输出限幅（0=不限制）
    /// 比如 set_output_limit(12.0) 表示输出值最多 ±12
    void set_output_limit(double limit);

private:
    double _kp, _ki, _kd;       // 三个增益系数
    double _integral;            // 误差的累积值（∫error·dt）
    double _prev_error;          // 上一次的误差（用来算 D 项）
    double _last_time;           // 上一次计算的时间（用来算 dt）
    double _integral_limit;      // 积分限幅值（0=不限制）
    double _d_filter_alpha;      // D 项滤波系数（0=不滤波）
    double _filtered_deriv;      // 滤波后的导数值
    double _output_limit;        // 输出限幅值（0=不限制）
};
