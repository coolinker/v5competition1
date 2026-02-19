// ============================================================================
//  hal/hal_log.cpp — 日志系统的实现
// ============================================================================
//
//  日志就像机器人的"日记"。它把运行过程中的重要事件写到
//  SD 卡上的文件里（Brain 上插的那张 microSD 卡）。
//  比赛后拔出 SD 卡，就能在电脑上看到完整的运行记录。
//
//  这个文件包含两种日志：
//  1. 文字日志 (hal_log.txt)  —— 记录事件，像"IMU校准完成"
//  2. CSV 数据 (odom_log.csv) —— 记录位姿数据，可以用 Excel 画轨迹图
//
// ============================================================================
#include "hal/hal_log.h"
#include "config.h"
#include "vex.h"
#include <fstream>    // 文件读写
#include <cstring>

using namespace vex;

// Brain 对象在 main.cpp 里定义，这里用 extern 声明"借用"它
extern brain Brain;

// SD 卡上的日志文件路径（/usd/ 是 VEX Brain 上 SD 卡的挂载路径）
#define HAL_LOG_FILE  "/usd/hal_log.txt"   // 文字日志
#define ODOM_CSV_FILE "/usd/odom_log.csv"  // CSV 数据日志

// 标记：CSV 文件的表头是否已经写过了
// 第一次写数据时先写一行表头（time_ms,x,y,theta,error），之后就不再重复写了
static bool odom_csv_header_written = false;

// ---- 简便版日志函数 ----
// 不指定级别时默认按 INFO 级别记录
void hal_log(const std::string& message, bool printToScreen) {
    hal_log_level(LOG_INFO, message, printToScreen);
}

// ---- 带级别的日志函数（核心） ----
void hal_log_level(int level, const std::string& message, bool printToScreen) {
    // 第一步：级别过滤
    // 如果消息级别 > config.h 里设定的 LOG_VERBOSITY，就直接忽略
    // 比如 LOG_VERBOSITY=2(INFO)，那 DEBUG(3) 消息就不会被记录
    if (level > LOG_VERBOSITY) return;

    // 第二步：根据级别加前缀，方便在日志文件里快速筛选
    const char* prefix = "";
    switch (level) {
        case LOG_ERROR: prefix = "ERR "; break;  // 严重错误
        case LOG_WARN:  prefix = "WRN "; break;  // 警告
        case LOG_INFO:  prefix = "INF "; break;  // 信息
        case LOG_DEBUG: prefix = "DBG "; break;  // 调试
    }

    // 第三步：获取系统时间戳（毫秒），比如 [12345] 表示开机第 12.345 秒
    unsigned long ms = (unsigned long)vex::timer::system();
    char timebuf[32];
    snprintf(timebuf, sizeof(timebuf), "%lu", ms);

    // 第四步：拼接最终的日志条目，格式: [时间戳] 级别 消息
    std::string log_entry = std::string("[") + timebuf + "] " + prefix + message + "\n";

    // 第五步：追加写入 SD 卡上的日志文件
    // std::ios::app 表示"追加模式"——每次都写在文件末尾，不覆盖之前的内容
    std::ofstream log_file(HAL_LOG_FILE, std::ios::app);
    if (log_file.is_open()) {
        log_file << log_entry;
        log_file.close();
    }

    // 第六步：严重错误和警告一定显示在 Brain 屏幕上（方便现场发现问题）
    if (printToScreen || level <= LOG_WARN) {
        Brain.Screen.print("%s", log_entry.c_str());
        Brain.Screen.newLine();
    }
}

// ---- CSV 位姿数据日志 ----
// 每次调用往 SD 卡上的 CSV 文件追加一行数据
// CSV = Comma-Separated Values（用逗号隔开的数值），Excel 可以直接打开
void hal_log_odom_csv(unsigned long time_ms, double x, double y, double theta, double error) {
    std::ofstream csv(ODOM_CSV_FILE, std::ios::app);
    if (!csv.is_open()) return;  // SD 卡没插好？打开失败就算了

    // 第一次调用时写入表头行
    if (!odom_csv_header_written) {
        csv << "time_ms,x,y,theta,error\n";
        odom_csv_header_written = true;
    }

    // 写入一行数据，%.4f 表示保留 4 位小数
    char buf[128];
    snprintf(buf, sizeof(buf), "%lu,%.4f,%.4f,%.4f,%.4f\n",
             time_ms, x, y, theta, error);
    csv << buf;
    csv.close();
}
