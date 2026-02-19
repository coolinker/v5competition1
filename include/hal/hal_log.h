#pragma once
// ============================================================================
//  hal/hal_log.h — 日志系统（让你看到机器人"脑子里在想什么"）
// ============================================================================
//
//  【这个文件干什么？】
//    就像你写数学题时在草稿纸上写计算过程一样，机器人也需要
//    把运行过程中的重要信息记下来，方便调试和排错。
//    日志会写到 SD 卡上的文件里，比赛后可以拔出来在电脑上看。
//
//  【日志级别——不是所有信息都一样重要】
//    ERROR = 0 : 严重错误（传感器掉线、校准失败）—— 必须关注！
//    WARN  = 1 : 可恢复的问题（视觉修正结果太离谱被拒绝了）
//    INFO  = 2 : 一般信息（校准完成、电机在动了）
//    DEBUG = 3 : 超详细数据（每次循环的数值）—— 平时关着，需要调试时再开
//    只有级别 <= config.h 里的 LOG_VERBOSITY 的日志才会被记录
//
// ============================================================================
#include <string>
#include <stdio.h>

// ---- 数字转字符串的辅助函数 ----
// VEX V5 用的编译器比较旧（GCC 4.9.3），不支持标准库的 std::to_string()，
// 所以我们自己写了一个替代品：把任何数字变成字符串。
// 例如：to_str(3.14) 返回 "3.14"，to_str(42) 返回 "42"
template<typename T>
inline std::string to_str(T value) {
    char buf[64];
    // snprintf 把数字格式化成字符串，放进 buf 数组里
    // %g 是一种自动选择最佳格式的占位符（整数不带小数点，小数带小数点）
    snprintf(buf, sizeof(buf), "%g", static_cast<double>(value));
    return std::string(buf);
}

// ---- 日志级别常量 ----
constexpr int LOG_ERROR = 0;  // 严重错误（最高优先级）
constexpr int LOG_WARN  = 1;  // 警告
constexpr int LOG_INFO  = 2;  // 一般信息
constexpr int LOG_DEBUG = 3;  // 详细调试（最低优先级，通常关闭）

/// 记录一条日志（默认会显示在 Brain 屏幕上）
void hal_log(const std::string& message, bool printToScreen = true);

/// 记录一条指定级别的日志
/// 只有 level <= config.h 里的 LOG_VERBOSITY 时才会被记录
void hal_log_level(int level, const std::string& message, bool printToScreen = false);

/// 往 SD 卡上的 CSV 文件 (/usd/odom_log.csv) 追加一行位姿数据
/// CSV 格式：时间戳, X坐标, Y坐标, 航向角, 距离误差
/// 比赛后可以用 Excel 或 Python 打开这个文件，画出机器人的行驶轨迹！
void hal_log_odom_csv(unsigned long time_ms, double x, double y, double theta, double error);
