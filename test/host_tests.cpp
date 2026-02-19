// ============================================================================
//  host_tests.cpp — 本机（电脑端）单元测试合集
// ============================================================================
//
//  【什么是单元测试？】
//    单元测试就是"自动化的检查"。就像老师出考试题来验证你会不会做一样，
//    程序员写单元测试来验证代码是否正确。每个测试检查代码的一个小功能。
//
//  【为什么不在机器人上测试？】
//    在电脑上测试有很多好处：
//    ① 不需要硬件 → 随时随地都能测
//    ② 速度快 → 0.1 秒跑完所有测试
//    ③ 可重复 → 每次结果一样（机器人可能没电、传感器漂移等）
//    ④ 更早发现 Bug → 代码一改完就能测
//
//  【怎么做到脱离硬件？】
//    用 Mock（模拟）替代真实硬件！
//    比如真实 IMU 需要传感器芯片才能读取角度，
//    而 Mock IMU 只是一个变量——你给它赋值多少它就返回多少。
//    这样测试只检验"算法逻辑"，不依赖硬件。
//
//  【编译与运行】
//    方法 1（手动）:
//      g++ -std=c++17 -I include -I src -o build/run_tests test/host_tests.cpp -lm
//      ./build/run_tests
//
//    方法 2（推荐）:
//      make test
//
//  【文件结构】
//    本文件是一个"全合一"文件，包含：
//    ① 迷你测试框架（TEST / ASSERT 宏）
//    ② Mock HAL（模拟硬件层）
//    ③ 24 个测试用例（覆盖 PID、运动曲线、里程计）
//    ④ main() 函数（运行所有测试、打印结果）
//
// ============================================================================

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <string>

// ============================================================================
//  迷你测试框架
// ============================================================================
//
//  这里定义了一套简易测试工具，功能类似 Google Test 但更轻量。
//  TEST(name)          → 定义一个测试函数
//  ASSERT_TRUE(cond)   → 断言条件为真，否则测试失败
//  ASSERT_NEAR(a,b,t)  → 断言 a 和 b 之差小于 t（用于浮点数比较）
//  ASSERT_GT(a, b)     → 断言 a > b
//  ASSERT_LT(a, b)     → 断言 a < b
//  RUN_TEST(name)      → 运行指定测试
//
// ============================================================================
static int g_tests_run = 0;     // 已运行的测试数
static int g_tests_passed = 0;  // 通过的测试数
static int g_tests_failed = 0;  // 失败的测试数
static const char* g_current_test = nullptr;  // 当前正在运行的测试名

// TEST 宏：定义一个测试用例。展开后会生成两个函数：
//   test_xxx()     — 实际测试代码
//   run_test_xxx() — 包装器：打印名称、计数、调用 test_xxx()
#define TEST(name) \
    static void test_##name(); \
    static void run_test_##name() { \
        g_current_test = #name; \
        g_tests_run++; \
        printf("  [RUN ] %s\n", #name); \
        test_##name(); \
        printf("  [ OK ] %s\n", #name); \
        g_tests_passed++; \
    } \
    static void test_##name()

// ASSERT_TRUE：如果条件为假，打印失败信息并提前退出当前测试
#define ASSERT_TRUE(cond) do { \
    if (!(cond)) { \
        printf("  [FAIL] %s (line %d): %s is false\n", g_current_test, __LINE__, #cond); \
        g_tests_failed++; return; \
    } \
} while(0)

// ASSERT_NEAR：浮点数"近似相等"判断
// 为什么不用 == ？因为浮点数有精度误差，0.1+0.2 ≠ 0.3！
// 所以用"差值 < 容差"来判断
#define ASSERT_NEAR(actual, expected, tolerance) do { \
    double _a = (actual), _e = (expected), _t = (tolerance); \
    if (std::abs(_a - _e) > _t) { \
        printf("  [FAIL] %s (line %d): expected %.6f, got %.6f (tol=%.6f)\n", \
            g_current_test, __LINE__, _e, _a, _t); \
        g_tests_failed++; return; \
    } \
} while(0)

// ASSERT_GT：断言 a 大于 b（GT = Greater Than）
#define ASSERT_GT(a, b) do { \
    double _a = (a), _b = (b); \
    if (!(_a > _b)) { \
        printf("  [FAIL] %s (line %d): expected %f > %f\n", \
            g_current_test, __LINE__, _a, _b); \
        g_tests_failed++; return; \
    } \
} while(0)

// ASSERT_LT：断言 a 小于 b（LT = Less Than）
#define ASSERT_LT(a, b) do { \
    double _a = (a), _b = (b); \
    if (!(_a < _b)) { \
        printf("  [FAIL] %s (line %d): expected %f < %f\n", \
            g_current_test, __LINE__, _a, _b); \
        g_tests_failed++; return; \
    } \
} while(0)

// RUN_TEST：运行一个测试（展开成调用 run_test_xxx()）
#define RUN_TEST(name) run_test_##name()

// ============================================================================
//  Mock HAL（模拟硬件抽象层）
// ============================================================================
//
//  这些全局变量模拟了真实硬件的状态。
//  测试时你可以随意设置它们的值，就像遥控硬件一样。
//
//  例如：mock_imu_heading_rad = 1.57;
//        → 假装 IMU 此刻读到的朝向是 90°（π/2 弧度）
//
// ============================================================================

static double        mock_time_sec = 0.0;            // 模拟时钟（秒）
static unsigned long mock_time_ms  = 0;              // 模拟时钟（毫秒）
static double        mock_left_ticks = 0.0;          // 左电机编码器刻度（预留）
static double        mock_right_ticks = 0.0;         // 右电机编码器刻度（预留）
static double        mock_imu_heading_rad = 0.0;     // 模拟 IMU 朝向
static double        mock_imu_rotation_rad = 0.0;    // 模拟 IMU 累计旋转
static double        mock_motor_left_v = 0.0;        // 最后设置的左电机电压
static double        mock_motor_right_v = 0.0;       // 最后设置的右电机电压
static double        mock_tracking_forward_dist = 0.0;  // 纵向追踪轮行驶距离
static double        mock_tracking_lateral_dist = 0.0;   // 横向追踪轮行驶距离

// ── 时间 Mock ──
// wait_ms 不真的等待，只是把模拟时钟往前拨（测试瞬间完成！）
double        get_time_sec() { return mock_time_sec; }
unsigned long get_time_ms()  { return mock_time_ms; }
void          wait_ms(int ms) { mock_time_sec += ms / 1000.0; mock_time_ms += ms; }

// ── 电机 Mock ──
double get_left_encoder_ticks()  { return mock_left_ticks; }
double get_right_encoder_ticks() { return mock_right_ticks; }
void   reset_encoders() { mock_left_ticks = 0; mock_right_ticks = 0; }
void   set_drive_motors(double lv, double rv) { mock_motor_left_v = lv; mock_motor_right_v = rv; }
void   stop_drive_motors() { mock_motor_left_v = 0; mock_motor_right_v = 0; }

// ── IMU Mock ──
double get_imu_heading_rad()  { return mock_imu_heading_rad; }
double get_imu_rotation_rad() { return mock_imu_rotation_rad; }
void   reset_imu()            { mock_imu_heading_rad = 0; mock_imu_rotation_rad = 0; }
void   calibrate_imu()        { /* 测试中不需要真的校准 */ }

// ── 追踪轮 Mock ──
void   tracking_wheels_init()  { }
void   tracking_wheels_reset() { mock_tracking_forward_dist = 0; mock_tracking_lateral_dist = 0; }
double tracking_get_forward_distance_m() { return mock_tracking_forward_dist; }
double tracking_get_lateral_distance_m() { return mock_tracking_lateral_dist; }
bool   tracking_wheels_connected()     { return true; }

// ── 日志 Mock（不输出任何东西） ──
void hal_log(const std::string& /*msg*/, bool /*print*/) {}
void hal_log_level(int, const std::string&, bool) {}
void hal_log_odom_csv(unsigned long, double, double, double, double) {}

// 重置所有 Mock 状态（每个测试开始前调用，确保测试互不干扰）
static void reset_all_mocks() {
    mock_time_sec = 0.0;
    mock_time_ms  = 0;
    mock_left_ticks = 0.0;
    mock_right_ticks = 0.0;
    mock_imu_heading_rad = 0.0;
    mock_imu_rotation_rad = 0.0;
    mock_motor_left_v = 0.0;
    mock_motor_right_v = 0.0;
    mock_tracking_forward_dist = 0.0;
    mock_tracking_lateral_dist = 0.0;
}

// ============================================================================
//  引入被测试的源代码
// ============================================================================
//  把算法的 .cpp 文件直接 #include 进来，这样它们会使用上面定义的
//  Mock 函数，而不是真实的硬件驱动。这是一种常见的"源文件包含"测试技巧。
// ============================================================================
#include "config.h"
#include "control/pid.h"
#include "control/motion_profile.h"
#include "localization/odometry.h"

#include "../src/control/pid.cpp"
#include "../src/control/motion_profile.cpp"
#include "../src/localization/odometry.cpp"

// ============================================================================
//  PID 控制器基础测试（6 个）
// ============================================================================

// 正误差 → 正输出（目标 10，实际 5，需要加速 → 输出 > 0）
TEST(PID_PositiveErrorProducesPositiveOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(2.0, 0.0, 0.0);  // 仅 P 项，Kp=2.0
    pid.reset();
    mock_time_sec = 1.01;
    double output = pid.calculate(10.0, 5.0);  // 误差 = 10-5 = 5, 输出 = 2×5 =10
    ASSERT_GT(output, 0.0);
    ASSERT_NEAR(output, 10.0, 0.01);
}

// 负误差 → 负输出（目标 5，实际 10，需要减速 → 输出 < 0）
TEST(PID_NegativeErrorProducesNegativeOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(2.0, 0.0, 0.0);
    pid.reset();
    mock_time_sec = 1.01;
    double output = pid.calculate(5.0, 10.0);  // 误差 = 5-10 = -5
    ASSERT_LT(output, 0.0);
    ASSERT_NEAR(output, -10.0, 0.01);
}

// 零误差 → 零输出（目标=实际，不需要任何修正）
TEST(PID_ZeroErrorProducesZeroOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(2.0, 0.0, 0.0);
    pid.reset();
    mock_time_sec = 1.01;
    double output = pid.calculate(5.0, 5.0);
    ASSERT_NEAR(output, 0.0, 0.001);
}

// 积分项会随时间累积（第二次输出应该比第一次大）
TEST(PID_IntegralAccumulates) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 1.0, 0.0);  // 仅 I 项
    pid.reset();

    mock_time_sec = 0.01;
    double out1 = pid.calculate(10.0, 5.0);  // 积分 = 5×0.01 = 0.05

    mock_time_sec = 0.02;
    double out2 = pid.calculate(10.0, 5.0);  // 积分 = 0.05 + 5×0.01 = 0.10
    ASSERT_GT(out2, out1);                   // 第二次输出更大
}

// 微分项响应误差变化（误差不变时 D 项趋近于 0）
TEST(PID_DerivativeRespondToChange) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 0.0, 1.0);  // 仅 D 项
    pid.reset();

    mock_time_sec = 0.01;
    double out1 = pid.calculate(10.0, 5.0);  // 误差从 0 变到 5 → D 有输出
    ASSERT_GT(out1, 0.0);

    mock_time_sec = 0.02;
    double out2 = pid.calculate(10.0, 5.0);  // 误差不变 → D 趋近 0
    ASSERT_NEAR(out2, 0.0, 0.01);
}

// reset() 清除所有内部状态（重置后行为与全新 PID 一致）
TEST(PID_ResetClearsState) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(1.0, 1.0, 0.1);
    pid.reset();

    // 先运行 10 次让积分等状态积累起来
    for (int i = 1; i <= 10; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(10.0, 5.0);
    }

    // 重置
    mock_time_sec = 1.0;
    pid.reset();
    mock_time_sec = 1.01;
    double after_reset = pid.calculate(10.0, 5.0);

    // 创建一个全新的 PID 控制器
    mock_time_sec = 2.0;
    PIDController pid2(1.0, 1.0, 0.1);
    pid2.reset();
    mock_time_sec = 2.01;
    double fresh = pid2.calculate(10.0, 5.0);

    // 重置后的输出应该和全新 PID 的输出一样
    ASSERT_NEAR(after_reset, fresh, 0.01);
}

// ============================================================================
//  PID 增强功能测试（6 个）
// ============================================================================

// 积分防饱和（Anti-Windup）：积分值被限幅在 ±2.0
TEST(PID_AntiWindup_ClampsIntegral) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 1.0, 0.0);
    pid.set_integral_limit(2.0);  // 积分上限 = 2.0
    pid.reset();

    // 大误差跑 100 次——没有限幅的话积分会达到 ~50
    for (int i = 1; i <= 100; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(100.0, 0.0);
    }

    // 有限幅 → 输出被卡在 ~2.0
    mock_time_sec += 0.01;
    double output = pid.calculate(100.0, 0.0);
    ASSERT_NEAR(output, 2.0, 0.2);
    ASSERT_LT(output, 3.0);       // 确保不会超太多
}

// 积分防饱和（负方向）：积分值不会低于 -2.0
TEST(PID_AntiWindup_NegativeClamp) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(0.0, 1.0, 0.0);
    pid.set_integral_limit(2.0);
    pid.reset();

    for (int i = 1; i <= 100; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(0.0, 100.0);  // 负误差方向
    }
    mock_time_sec += 0.01;
    double output = pid.calculate(0.0, 100.0);
    ASSERT_NEAR(output, -2.0, 0.2);
    ASSERT_GT(output, -3.0);
}

// D 项 EMA 滤波器使微分更平滑（滤波后的值比原始值更小）
TEST(PID_DFilter_SmoothsDerivative) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid_raw(0.0, 0.0, 1.0);    // 无滤波
    PIDController pid_filt(0.0, 0.0, 1.0);
    pid_filt.set_d_filter(0.7);                // α=0.7 → 较强平滑
    pid_raw.reset();
    pid_filt.reset();

    mock_time_sec = 0.01;
    double raw1  = pid_raw.calculate(10.0, 0.0);   // 突变 → D 很大
    mock_time_sec = 0.0;
    pid_filt.reset();
    mock_time_sec = 0.01;
    double filt1 = pid_filt.calculate(10.0, 0.0);  // 同样突变，但被平滑

    ASSERT_LT(std::abs(filt1), std::abs(raw1));     // 滤波后更小
}

// 输出限幅：输出被钳位在 ±5.0
TEST(PID_OutputLimit_ClampsOutput) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(10.0, 0.0, 0.0);    // Kp=10, 误差=100 → 原始输出=1000
    pid.set_output_limit(5.0);             // 但限幅到 ±5
    pid.reset();

    mock_time_sec = 1.01;
    double output = pid.calculate(100.0, 0.0);
    ASSERT_NEAR(output, 5.0, 0.001);      // 正方向限在 5.0

    mock_time_sec = 1.02;
    double neg_output = pid.calculate(0.0, 100.0);
    ASSERT_NEAR(neg_output, -5.0, 0.001); // 负方向限在 -5.0
}

// 未设置输出限幅时不钳位（默认不限制）
TEST(PID_OutputLimit_NoClampWhenDisabled) {
    reset_all_mocks();
    mock_time_sec = 1.0;
    PIDController pid(10.0, 0.0, 0.0);   // 不设置 output_limit
    pid.reset();

    mock_time_sec = 1.01;
    double output = pid.calculate(100.0, 0.0);
    ASSERT_NEAR(output, 1000.0, 0.1);    // 可以自由达到 1000
}

// reset() 同时清除增强功能的状态（滤波器、积分等）
TEST(PID_ResetClearsEnhancedState) {
    reset_all_mocks();
    mock_time_sec = 0.0;
    PIDController pid(1.0, 1.0, 1.0);
    pid.set_integral_limit(10.0);
    pid.set_d_filter(0.5);
    pid.set_output_limit(50.0);
    pid.reset();

    // 运行 20 次让状态积累
    for (int i = 1; i <= 20; i++) {
        mock_time_sec = i * 0.01;
        pid.calculate(10.0, 5.0);
    }

    // 重置后再计算一次
    mock_time_sec = 1.0;
    pid.reset();
    mock_time_sec = 1.01;
    double after_reset = pid.calculate(10.0, 5.0);

    // 全新 PID 的第一次计算
    mock_time_sec = 2.0;
    PIDController pid2(1.0, 1.0, 1.0);
    pid2.set_integral_limit(10.0);
    pid2.set_d_filter(0.5);
    pid2.set_output_limit(50.0);
    pid2.reset();
    mock_time_sec = 2.01;
    double fresh = pid2.calculate(10.0, 5.0);

    // 两者应该完全一致
    ASSERT_NEAR(after_reset, fresh, 0.01);
}

// ============================================================================
//  运动曲线（Motion Profile）测试（5 个）
// ============================================================================

// 加速阶段：刚起步时速度 = a × t
TEST(MotionProfile_AccelerationPhase) {
    MotionProfile profile(1.0, 2.0);  // 最大速度 1.0 m/s，加速度 2.0 m/s²
    double v = profile.get_target_velocity(0.1, 2.0);  // t=0.1s，离终点 2m
    ASSERT_NEAR(v, 0.2, 0.001);  // v = 2.0 × 0.1 = 0.2 m/s
}

// 巡航阶段：加速够久后速度停在最大值
TEST(MotionProfile_ReachesMaxVelocity) {
    MotionProfile profile(1.0, 2.0);
    double v = profile.get_target_velocity(1.0, 2.0);  // t=1s，离终点 2m
    ASSERT_NEAR(v, 1.0, 0.001);  // 已达最大速度 1.0
}

// 减速阶段：接近终点时速度降低（v = √(2ad)）
TEST(MotionProfile_DecelerationPhase) {
    MotionProfile profile(1.0, 2.0);
    double v = profile.get_target_velocity(1.0, 0.1);  // 离终点只剩 0.1m
    ASSERT_LT(v, 1.0);                                 // 需要减速
    ASSERT_NEAR(v, sqrt(0.4), 0.001);                   // √(2×2.0×0.1) ≈ 0.632
}

// 到终点了 → 速度为 0
TEST(MotionProfile_ZeroDistanceProducesZeroVelocity) {
    MotionProfile profile(1.0, 2.0);
    double v = profile.get_target_velocity(1.0, 0.0);
    ASSERT_NEAR(v, 0.0, 0.001);
}

// 任何情况下速度都不超过最大值（遍历大量组合验证）
TEST(MotionProfile_VelocityNeverExceedsMax) {
    MotionProfile profile(1.0, 2.0);
    for (double t = 0.0; t < 5.0; t += 0.1) {
        for (double d = 0.0; d < 5.0; d += 0.1) {
            double v = profile.get_target_velocity(t, d);
            ASSERT_TRUE(v <= 1.0 + 0.001);
        }
    }
}

// ============================================================================
//  里程计（Odometry）测试（7 个）—— 使用追踪轮
// ============================================================================

// 初始位姿应该是 (0, 0, 0)
TEST(Odometry_InitialPoseIsZero) {
    reset_all_mocks();
    set_pose({0, 0, 0});
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 0.0, 0.001);
    ASSERT_NEAR(p.y, 0.0, 0.001);
    ASSERT_NEAR(p.theta, 0.0, 0.001);
}

// set_pose() 能正确设置位姿
TEST(Odometry_SetPoseWorks) {
    reset_all_mocks();
    set_pose({1.5, 2.5, 0.5});
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 1.5, 0.001);
    ASSERT_NEAR(p.y, 2.5, 0.001);
    ASSERT_NEAR(p.theta, 0.5, 0.001);
}

// 直线前进：纵向轮走 1 米，横向轮不动，IMU 没转 → x 增加
TEST(Odometry_DriveStraightForward) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    // 模拟：纵向轮走了 1 米，横向轮没动，IMU 没转
    mock_tracking_forward_dist = 1.0;
    mock_tracking_lateral_dist = 0.0;
    mock_imu_rotation_rad = 0.0;

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 1.0, 0.02);    // 前进了 1 米
    ASSERT_NEAR(p.y, 0.0, 0.02);    // 没有横移
    ASSERT_NEAR(p.theta, 0.0, 0.02); // 没有转弯
}

// 原地转弯 90°：IMU 转了 90°，追踪轮只记录因偏移产生的弧线
TEST(Odometry_PointTurn90Degrees) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    double turn_rad = M_PI / 2.0;  // 90°

    // 纵向轮在中轴线上（offset=0），旋转时不动
    mock_tracking_forward_dist = FORWARD_WHEEL_OFFSET * turn_rad;
    // 横向轮偏离旋转中心，旋转时画弧
    mock_tracking_lateral_dist = LATERAL_WHEEL_OFFSET * turn_rad;
    mock_imu_rotation_rad = turn_rad;

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 0.0, 0.05);          // 原地转，x 不变
    ASSERT_NEAR(p.y, 0.0, 0.05);          // y 也不变
    ASSERT_NEAR(p.theta, turn_rad, 0.05); // 朝向变为 90°
}

// 倒车：纵向轮走负距离 → x 减少
TEST(Odometry_DriveBackward) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    mock_tracking_forward_dist = -0.5;
    mock_tracking_lateral_dist = 0.0;
    mock_imu_rotation_rad = 0.0;

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, -0.5, 0.02);   // 后退了 0.5 米
    ASSERT_NEAR(p.y, 0.0, 0.02);
}

// 多次更新会累积（分两步各走 0.5m = 总共 1m）
TEST(Odometry_MultipleUpdatesAccumulate) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    // 第 1 步：走 0.5m
    mock_tracking_forward_dist = 0.5;
    mock_tracking_lateral_dist = 0.0;
    mock_imu_rotation_rad = 0.0;
    odometry_update();

    // 第 2 步：累计走了 1.0m（注意：Mock 给的是总量，不是增量）
    mock_tracking_forward_dist = 1.0;
    mock_tracking_lateral_dist = 0.0;
    mock_imu_rotation_rad = 0.0;
    odometry_update();

    Pose p = get_pose();
    ASSERT_NEAR(p.x, 1.0, 0.02);    // 总共前进 1 米
    ASSERT_NEAR(p.y, 0.0, 0.02);
}

// 纯横向滑动：横向轮走了 0.3m，纵向轮不动，IMU 没转
// → 机器人向左平移（y 增加）
TEST(Odometry_LateralSlide) {
    reset_all_mocks();
    set_pose({0, 0, 0});

    mock_tracking_forward_dist = 0.0;
    mock_tracking_lateral_dist = 0.3;   // 横向滑了 0.3m
    mock_imu_rotation_rad = 0.0;

    odometry_update();
    Pose p = get_pose();
    ASSERT_NEAR(p.x, 0.0, 0.02);    // 没有前进
    ASSERT_NEAR(p.y, 0.3, 0.02);    // 向左平移 0.3m
    ASSERT_NEAR(p.theta, 0.0, 0.02); // 没有转弯
}

// ============================================================================
//  主函数：运行所有 24 个测试
// ============================================================================

int main() {
    printf("============================================\n");
    printf("  VEX Robot Host-Side Unit Tests\n");
    printf("  Config: 6-motor + perpendicular tracking wheels\n");
    printf("============================================\n\n");

    // ── PID 基础测试 ──
    printf("[PID Controller]\n");
    RUN_TEST(PID_PositiveErrorProducesPositiveOutput);
    RUN_TEST(PID_NegativeErrorProducesNegativeOutput);
    RUN_TEST(PID_ZeroErrorProducesZeroOutput);
    RUN_TEST(PID_IntegralAccumulates);
    RUN_TEST(PID_DerivativeRespondToChange);
    RUN_TEST(PID_ResetClearsState);

    // ── PID 增强功能测试 ──
    printf("\n[PID Enhancements]\n");
    RUN_TEST(PID_AntiWindup_ClampsIntegral);
    RUN_TEST(PID_AntiWindup_NegativeClamp);
    RUN_TEST(PID_DFilter_SmoothsDerivative);
    RUN_TEST(PID_OutputLimit_ClampsOutput);
    RUN_TEST(PID_OutputLimit_NoClampWhenDisabled);
    RUN_TEST(PID_ResetClearsEnhancedState);

    // ── 运动曲线测试 ──
    printf("\n[Motion Profile]\n");
    RUN_TEST(MotionProfile_AccelerationPhase);
    RUN_TEST(MotionProfile_ReachesMaxVelocity);
    RUN_TEST(MotionProfile_DecelerationPhase);
    RUN_TEST(MotionProfile_ZeroDistanceProducesZeroVelocity);
    RUN_TEST(MotionProfile_VelocityNeverExceedsMax);

    // ── 里程计测试 ──
    printf("\n[Odometry — Perpendicular Tracking Wheels]\n");
    RUN_TEST(Odometry_InitialPoseIsZero);
    RUN_TEST(Odometry_SetPoseWorks);
    RUN_TEST(Odometry_DriveStraightForward);
    RUN_TEST(Odometry_PointTurn90Degrees);
    RUN_TEST(Odometry_DriveBackward);
    RUN_TEST(Odometry_MultipleUpdatesAccumulate);
    RUN_TEST(Odometry_LateralSlide);

    // ── 汇总 ──
    printf("\n============================================\n");
    printf("  Results: %d passed, %d failed, %d total\n",
        g_tests_passed, g_tests_failed, g_tests_run);
    printf("============================================\n");

    if (g_tests_failed > 0) {
        printf("  *** SOME TESTS FAILED ***\n");
        return 1;
    } else {
        printf("  ALL TESTS PASSED\n");
        return 0;
    }
}
