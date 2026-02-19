// ============================================================================
//  main.cpp — 竞赛主程序（整个机器人的"大脑"）
// ============================================================================
//
//  【这个文件做什么】
//    这是整个程序的入口。就像你打开一个 App 时的启动界面一样，
//    VEX 竞赛系统会从这里开始运行你的代码。
//
//  【竞赛流程】
//    ① 开机 → main() 被调用
//    ② main() 注册"自治"和"操控"两个回调函数
//    ③ main() 调用 pre_auton() 做初始化
//    ④ 裁判按下"自治阶段"按钮 → autonomous() 被调用
//    ⑤ 裁判按下"操控阶段"按钮 → usercontrol() 被调用
//
//  【硬件配置】
//    6 × V5 电机 (蓝色墨盒 600RPM, 3个左 + 3个右)
//    2 × V5 旋转传感器 (追踪轮)
//    1 × V5 惯性传感器 (陀螺仪)
//    1 × AI 视觉传感器 (识别 AprilTag)
//
//  【后台任务】
//    后台任务就像手机的后台应用——主程序在前台跑，
//    它们在后台默默干活，互不干扰。
//    1. 里程计 — 100 Hz 持续计算位置（追踪轮 + IMU 融合）
//    2. 屏幕   — 20 Hz 在 Brain 屏幕上显示调试信息
//    3. 视觉   — 20 Hz 用 AprilTag 修正位置
//    4. 日志   — 10 Hz 把位置数据记到 SD 卡（CSV 格式）
//
// ============================================================================

#include "vex.h"
#include "config.h"
#include <cmath>
#include "hal/imu.h"
#include "hal/motors.h"
#include "hal/hal_log.h"
#include "hal/time.h"
#include "hal/vision.h"
#include "hal/tracking_wheels.h"
#include "localization/odometry.h"
#include "localization/vision_localizer.h"
#include "motion/drive_to_pose.h"
#include "motion/turn_to_heading.h"

using namespace vex;

// ============================================================================
//  硬件定义 —— 告诉程序每个硬件插在哪个端口
// ============================================================================
// Brain：机器人的"大脑"，有屏幕和 SD 卡槽
brain      Brain;
// Controller1：手柄（用来操控机器人）
controller Controller1 = controller(primary);
// Competition：竞赛管理器（由裁判系统控制比赛阶段切换）
competition Competition;

// ── 传感器 ──────────────────────────────────────────────────────────────────
// 惯性传感器（陀螺仪）——测量机器人朝哪个方向
inertial DrivetrainInertial = inertial(IMU_PORT);
// AI 视觉传感器——识别场地上的 AprilTag 标签
aivision VisionSensor       = aivision(VISION_PORT);

// ── 追踪轮 (V5 旋转传感器，垂直双轮方案) ──────────────────────────────────
// 纵向轮：测量前后行驶距离（装在机器人中轴线上或附近）
// 横向轮：测量左右滑动距离（装在垂直于前进方向的位置）
rotation ForwardTrackingSensor = rotation(FORWARD_TRACKING_PORT, FORWARD_TRACKING_REVERSED);
rotation LateralTrackingSensor = rotation(LATERAL_TRACKING_PORT, LATERAL_TRACKING_REVERSED);

// ── 六电机底盘 ─────────────────────────────────────────────────────────────
// 左边 3 个电机反转（reversed = true），右边 3 个正转（reversed = false）
// 因为左右电机安装方向对称，物理转向相反
motor  LeftFront  = motor(LEFT_FRONT_MOTOR_PORT,  ratio6_1, true);
motor  LeftMid    = motor(LEFT_MID_MOTOR_PORT,    ratio6_1, true);
motor  LeftRear   = motor(LEFT_REAR_MOTOR_PORT,   ratio6_1, true);
motor  RightFront = motor(RIGHT_FRONT_MOTOR_PORT, ratio6_1, false);
motor  RightMid   = motor(RIGHT_MID_MOTOR_PORT,   ratio6_1, false);
motor  RightRear  = motor(RIGHT_REAR_MOTOR_PORT,  ratio6_1, false);

// ============================================================================
//  后台任务 ①: Brain 屏幕调试显示
// ============================================================================
//  在 Brain 的屏幕上实时显示当前位置、朝向、传感器状态，
//  方便你在比赛前调试。
// ============================================================================
static int screen_task_fn() {
    while (true) {
        Pose p = get_pose();  // 读取当前位姿
        double heading_deg = p.theta * 180.0 / M_PI;  // 弧度 → 角度

        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("=== 6M Tracking Odom ===");

        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("X: %.3f m", p.x);          // X 坐标（米）
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Y: %.3f m", p.y);          // Y 坐标（米）
        Brain.Screen.setCursor(4, 1);
        Brain.Screen.print("Heading: %.1f deg", heading_deg);  // 朝向（度）

        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print("Enc: TrackingWheels  IMU: %s",
            DrivetrainInertial.installed() ? "OK" : "NC");  // IMU 是否连接

        Brain.Screen.setCursor(7, 1);
        int tags = vision_localizer_tag_count();
        Brain.Screen.print("Vision tags: %d", tags);  // 检测到几个 AprilTag

        vex::task::sleep(SCREEN_UPDATE_INTERVAL_MS);  // 等 50ms（20Hz）
    }
    return 0;
}

// ============================================================================
//  后台任务 ②: 视觉定位器
// ============================================================================
//  用 AI 视觉传感器识别 AprilTag，算出绝对坐标，修正里程计的漂移。
//  就像手机 GPS 替你纠正方向一样。
// ============================================================================
static int vision_task_fn() {
    while (true) {
        VisionEstimate est = vision_localizer_update();  // 拍照 + 计算位置
        if (est.valid) {
            vision_correct_odometry(est);  // 有效就修正里程计
        }
        vex::task::sleep(VISION_UPDATE_INTERVAL_MS);  // 等 50ms（20Hz）
    }
    return 0;
}

// ============================================================================
//  后台任务 ③: CSV 位姿日志 (10 Hz)
// ============================================================================
//  每 100ms 把当前位置和到目标的距离记录到 SD 卡的 CSV 文件。
//  比赛后拔出 SD 卡，用 Excel 打开就能画轨迹图！
// ============================================================================
static Pose auton_target = {0, 0, 0};  // 当前自治目标点（日志用）

static int csv_logger_task_fn() {
    while (true) {
        Pose p = get_pose();
        // 计算到当前目标的距离（用于记录跟踪误差）
        double dx = auton_target.x - p.x;
        double dy = auton_target.y - p.y;
        double error_dist = sqrt(dx * dx + dy * dy);
        // 写入一行 CSV：时间戳, x, y, theta, 误差
        hal_log_odom_csv(get_time_ms(), p.x, p.y, p.theta, error_dist);
        vex::task::sleep(100);  // 100ms = 10Hz
    }
    return 0;
}

// ============================================================================
//  pre_auton() — 开机初始化（比赛开始前运行）
// ============================================================================
//  这个函数在你打开机器人后立刻运行，在比赛开始前完成所有准备工作：
//    1. 校准传感器
//    2. 设置起始位置
//    3. 启动后台任务
// ============================================================================
void pre_auton() {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Initializing...");

    hal_log("=== Pre-Auton Init ===");

    // 1. 校准惯性传感器（需要 ~2 秒，这段时间机器人不能动！）
    calibrate_imu();

    // 2. 初始化追踪轮（把编码器归零，准备开始测量）
    tracking_wheels_init();
    if (!tracking_wheels_connected()) {
        hal_log_level(LOG_WARN, "Tracking wheels NOT detected!");
        // 警告：追踪轮没检测到！检查线缆连接
    }

    // 3. 初始化视觉传感器（配置 AprilTag 检测模式）
    vision_init();
    vision_localizer_init();

    // 4. 设置起始位姿：告诉里程计"我现在在原点，朝向 0°"
    //    比赛时要根据你把机器人放的实际位置来调整！
    set_pose({0.0, 0.0, 0.0});

    // 5. 启动后台任务（它们会在后台默默运行，直到关机）
    odometry_start_task();                    // 里程计（100Hz）
    vex::task screenTask(screen_task_fn);     // 屏幕显示（20Hz）
    vex::task visionTask(vision_task_fn);     // 视觉定位（20Hz）
    vex::task csvTask(csv_logger_task_fn);    // CSV 日志（10Hz）

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Ready!");
    hal_log("Pre-auton complete");
}

// ============================================================================
//  autonomous() — 自治阶段（裁判按下按钮后自动运行）
// ============================================================================
//  在这个函数里写你的自治策略。机器人会按照代码自动行动，
//  不需要人操控。通常有 15 秒或 60 秒的时间限制。
//
//  使用 drive_to_pose (Boomerang) 走弧线到目标位置，
//  使用 turn_to_heading 原地转向。
//
//  下面是示例路线——请根据你的比赛策略修改！
// ============================================================================
void autonomous() {
    hal_log("=== Autonomous Start ===");

    // ─── 示例路线 (请替换为你的比赛策略！) ─────────────────────────────

    // 第 1 步：前进到 (0.5, 0) 米
    auton_target = {0.5, 0.0, 0.0};
    drive_to_pose(auton_target);

    // 第 2 步：原地左转 90°
    turn_to_heading(M_PI / 2.0);  // π/2 ≈ 90°

    // 第 3 步：前进到 (0.5, 0.5) 米，朝向 90°
    auton_target = {0.5, 0.5, M_PI / 2.0};
    drive_to_pose(auton_target);

    // 第 4 步：原地转回 0°
    turn_to_heading(0.0);

    // 第 5 步：回到原点
    auton_target = {0.0, 0.0, 0.0};
    drive_to_pose(auton_target);

    hal_log("=== Autonomous End ===");
}

// ============================================================================
//  usercontrol() — 操控阶段（选手拿手柄操控机器人）
// ============================================================================
//  坦克驱动模式：
//    左摇杆（Axis3）控制左边三个电机
//    右摇杆（Axis2）控制右边三个电机
//    两个摇杆同时推 → 直行
//    只推一个 → 原地转弯
//    推相反方向 → 快速旋转
// ============================================================================
void usercontrol() {
    hal_log("=== Driver Control Start ===");

    while (true) {
        // 读取摇杆位置（-100 到 +100 的百分比）
        double left_pct  = Controller1.Axis3.position(percent);
        double right_pct = Controller1.Axis2.position(percent);

        // 死区处理：摇杆偏移 < 5% 时当作 0
        // 为什么需要？因为摇杆回中后不一定精确到 0，
        // 可能有 1~3% 的漂移，不处理的话机器人会缓慢移动
        if (fabs(left_pct)  < 5.0) left_pct  = 0.0;
        if (fabs(right_pct) < 5.0) right_pct = 0.0;

        // 百分比 → 电压（12V = 满速）
        // 比如摇杆推到 50% → 6V → 半速
        double left_v  = (left_pct  / 100.0) * 12.0;
        double right_v = (right_pct / 100.0) * 12.0;

        set_drive_motors(left_v, right_v);  // 发送到电机
        wait(20, msec);  // 50Hz 刷新率
    }
}

// ============================================================================
//  main() — 程序入口（整个程序从这里开始）
// ============================================================================
//  这是 C++ 程序的标准入口点。VEX 系统启动后会调用这个函数。
//
//  工作流程：
//    1. 告诉竞赛管理器：自治阶段用 autonomous()，操控阶段用 usercontrol()
//    2. 调用 pre_auton() 做所有初始化
//    3. 无限循环等待（竞赛管理器会在合适的时候调用 autonomous/usercontrol）
// ============================================================================
int main() {
    // 注册回调函数（告诉系统：比赛开始时调用哪些函数）
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // 初始化（校准传感器、启动后台任务等）
    pre_auton();

    // 无限等待——不要让 main 函数结束！
    // 竞赛管理器会在后台切换自治/操控阶段
    while (true) {
        wait(100, msec);
    }
}
