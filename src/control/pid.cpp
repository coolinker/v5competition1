// ============================================================================
//  control/pid.cpp — PID 控制器的实现
// ============================================================================
//
//  核心公式：
//    输出 = Kp × 误差  +  Ki × ∫误差·dt  +  Kd × d(误差)/dt
//
//  【重要细节】
//    • 使用真实的时间间隔 (dt) 计算 I 和 D 项，确保结果不受循环速度影响
//    • 防止 dt=0（第一次调用或循环太快时会出现）
//    • 可选功能：积分限幅、D 项滤波、输出限幅
//
// ============================================================================
#include "control/pid.h"
#include "hal/time.h"

// ---- 构造函数 ----
// 初始化所有成员变量。增强功能默认关闭（值为 0）
PIDController::PIDController(double kp, double ki, double kd)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _prev_error(0), _last_time(0),
      _integral_limit(0), _d_filter_alpha(0),
      _filtered_deriv(0), _output_limit(0) {}

// ---- 核心：计算一次 PID 输出 ----
double PIDController::calculate(double setpoint, double pv) {
    // 第一步：算出时间间隔 dt（从上次调用到现在过了多久）
    double now = get_time_sec();
    double dt  = now - _last_time;

    // 保护：如果 dt 为 0 或负数（第一次调用时会发生），用 0.01 秒代替
    // 防止后面的除法出错（除以 0 会得到无穷大！）
    if (dt <= 0.0) dt = 0.01;

    // 第二步：计算误差 = 目标值 - 当前值
    // 误差为正 → 还没到目标，需要正方向努力
    // 误差为负 → 超过目标了，需要反方向修正
    double error = setpoint - pv;

    // ── P 项（比例）──
    // 误差越大，输出越大。简单粗暴但有效！
    double p_out = _kp * error;

    // ── I 项（积分）──
    // 把每次的误差按时间累加起来（像攒零花钱）
    // 如果始终有小误差，积分会慢慢变大，推动输出去消除它
    _integral += error * dt;
    // 积分限幅：防止积分累积太大（像存钱罐满了就不再存）
    // 如果不限制，在目标突然变化时积分值会很大，导致超调（冲过头）
    if (_integral_limit > 0.0) {
        if (_integral >  _integral_limit) _integral =  _integral_limit;
        if (_integral < -_integral_limit) _integral = -_integral_limit;
    }
    double i_out = _ki * _integral;

    // ── D 项（微分）──
    // 看误差变化有多快：(这次误差 - 上次误差) / 时间
    // 如果误差在快速减小 → 说明已经在快速接近目标 → D 项会"踩刹车"
    double raw_deriv = (error - _prev_error) / dt;
    // 可选的低通滤波（EMA = 指数移动平均）：
    // 传感器数据有噪声，微分会把噪声放大。滤波可以平滑掉噪声。
    // alpha 越大，滤波越强（越平滑但反应越慢）
    if (_d_filter_alpha > 0.0) {
        _filtered_deriv = _d_filter_alpha * _filtered_deriv
                        + (1.0 - _d_filter_alpha) * raw_deriv;
    } else {
        _filtered_deriv = raw_deriv;  // 不滤波，直接用原始值
    }
    double d_out = _kd * _filtered_deriv;

    // 保存状态给下次调用使用
    _prev_error = error;
    _last_time  = now;

    // 合并 P + I + D 三项
    double output = p_out + i_out + d_out;

    // 输出限幅：把输出值控制在 ±output_limit 以内
    // 防止给电机的命令太大（比如电机最多承受 12V）
    if (_output_limit > 0.0) {
        if (output >  _output_limit) output =  _output_limit;
        if (output < -_output_limit) output = -_output_limit;
    }

    return output;
}

// ---- 重置控制器 ----
// 每次新运动开始前调用，清除上一次运动留下的数据
void PIDController::reset() {
    _integral       = 0;              // 清除积分累积
    _prev_error     = 0;              // 清除上一次误差
    _filtered_deriv = 0;              // 清除滤波器状态
    _last_time      = get_time_sec(); // 记录当前时间作为新起点
}

// ---- 增强功能设定 ----
void PIDController::set_integral_limit(double limit) { _integral_limit = limit; }
void PIDController::set_d_filter(double alpha) { _d_filter_alpha = alpha; }
void PIDController::set_output_limit(double limit) { _output_limit = limit; }
