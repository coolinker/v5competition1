// ============================================================================
//  hal/motors.cpp — 六电机底盘驱动的实现
// ============================================================================
//
//  【电机布局（俯视图）】
//
//     前进方向 →
//    ┌───────────────────┐
//    │ LeftFront   RightFront │   前轮
//    │ LeftMid     RightMid   │   中轮
//    │ LeftRear    RightRear  │   后轮
//    └───────────────────┘
//
//  同一侧的 3 个电机收到相同的电压（像三胞胎一起使劲）。
//  左右两侧电压不同就能转弯（差速转向）。
//
// ============================================================================
#include "hal/motors.h"
#include "config.h"
#include "vex.h"
#include "hal/hal_log.h"

// ---- 电压限幅函数 ----
// 确保给电机的电压不超过 ±12V（V5 电机最大 12V）
// 就像汽车油门踩到底也只能到 100%，不能到 120%
static double clamp_voltage(double v) {
    if (v >  12.0) return  12.0;
    if (v < -12.0) return -12.0;
    return v;
}

// 6 个电机对象在 main.cpp 里创建，这里用 extern 声明"借用"它们
extern vex::motor LeftFront;
extern vex::motor LeftMid;
extern vex::motor LeftRear;
extern vex::motor RightFront;
extern vex::motor RightMid;
extern vex::motor RightRear;

// 把电机放进数组，方便用 for 循环统一操作
// 这样增减电机数量只需改数组，不用改每个函数
static vex::motor* left_motors[]  = { &LeftFront,  &LeftMid,  &LeftRear  };
static vex::motor* right_motors[] = { &RightFront, &RightMid, &RightRear };

// ---- 设定电机电压 ----
void set_drive_motors(double left_voltage, double right_voltage) {
    // 先限幅，防止超过 ±12V
    left_voltage  = clamp_voltage(left_voltage);
    right_voltage = clamp_voltage(right_voltage);

    // 给同侧 3 个电机设定相同电压
    // 注意：VEX API 的 spin() 接受的单位是毫伏 (mV)，所以要乘以 1000
    // 例如 12V × 1000 = 12000mV
    for (int i = 0; i < MOTORS_PER_SIDE; ++i) {
        left_motors[i]->spin(vex::fwd,  left_voltage  * 1000.0, vex::voltageUnits::mV);
        right_motors[i]->spin(vex::fwd, right_voltage * 1000.0, vex::voltageUnits::mV);
    }
}

// ---- 刹停所有电机 ----
void stop_drive_motors() {
    for (int i = 0; i < MOTORS_PER_SIDE; ++i) {
        // brakeType::brake = 主动刹车（电机反向阻力），比 coast（惯性滑行）停得更快
        left_motors[i]->stop(vex::brakeType::brake);
        right_motors[i]->stop(vex::brakeType::brake);
    }
}

// ---- 读取电机编码器 ----
// 这两个函数目前没被里程计使用（里程计用追踪轮代替了），
// 但保留着以备将来需要。
double get_left_encoder_ticks() {
    return LeftMid.position(vex::rotationUnits::raw);
}

double get_right_encoder_ticks() {
    return RightMid.position(vex::rotationUnits::raw);
}

// ---- 重置所有电机编码器 ----
void reset_encoders() {
    for (int i = 0; i < MOTORS_PER_SIDE; ++i) {
        left_motors[i]->resetPosition();
        right_motors[i]->resetPosition();
    }
}
