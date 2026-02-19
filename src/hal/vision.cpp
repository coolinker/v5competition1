// ============================================================================
//  hal/vision.cpp — AI 视觉传感器的实现（AprilTag 检测）
// ============================================================================
//
//  【工作流程】
//    1. vision_init()     → 开机时初始化，打开 AprilTag 检测模式
//    2. vision_snapshot()  → "拍一张照"，传感器分析画面找 AprilTag
//    3. vision_get_tag(i)  → 取出第 i 个检测到的标签的信息
//
//  【AprilTag 是什么？】
//    一种黑白方块图案（像超级简化的二维码），被贴在赛场墙壁上。
//    VEX 使用 36h11 系列标签——每个标签有唯一 ID。
//    因为每个标签在赛场上的坐标是已知的，看到标签就能算出
//    "我自己在哪里"。这是视觉定位的关键！
//
// ============================================================================
#include "hal/vision.h"
#include "config.h"
#include "vex.h"

// 视觉传感器对象在 main.cpp 里创建，这里用 extern "借用"
extern vex::aivision VisionSensor;

// ---- 内部缓冲区 ----
// 每次拍照的结果存在这里，上层通过 vision_get_tag() 来取
static TagDetection tag_buffer[VISION_MAX_TAGS];  // 最多存 8 个标签
static int          tag_count = 0;                 // 上次拍到了几个标签

// ---- 初始化 ----
void vision_init() {
    // 打开物体检测功能（包括 AprilTag 检测）
    VisionSensor.objectDetection(true);
    hal_log("Vision sensor initialized (AprilTag mode)");
}

// ---- 拍照并检测标签 ----
int vision_snapshot() {
    tag_count = 0;  // 清空上次的计数

    // 让传感器拍一张照，检测所有类型的物体
    VisionSensor.takeSnapshot(vex::aivision::ALL_AIOBJS);
    int raw_count = VisionSensor.objectCount;  // 传感器看到了几个物体

    // 遍历所有检测到的物体，只保留 AprilTag 类型的
    for (int i = 0; i < raw_count && tag_count < VISION_MAX_TAGS; ++i) {
        auto& obj = VisionSensor.objects[i];

        // 过滤：只要 AprilTag，忽略其他物体（比如彩色方块等）
        if (obj.type == vex::aivision::kAiVisAprilTag) {
            TagDetection& t = tag_buffer[tag_count];
            t.id       = obj.id;        // 标签 ID
            t.center_x = obj.centerX;   // 图片中的水平位置
            t.center_y = obj.centerY;   // 图片中的垂直位置
            t.width    = obj.width;     // 标签在图片里的宽度（越大=越近）
            t.height   = obj.height;    // 标签在图片里的高度
            t.angle    = obj.angle;     // 标签旋转角度
            t.valid    = true;          // 标记为有效
            tag_count++;
        }
    }

    // 如果看到了标签，记一条日志
    if (tag_count > 0) {
        hal_log("Vision: " + to_str(tag_count) + " AprilTag(s) detected");
    }
    return tag_count;
}

// ---- 取出一个标签的检测结果 ----
TagDetection vision_get_tag(int index) {
    // 索引合法就返回缓冲区里的结果
    if (index >= 0 && index < tag_count) {
        return tag_buffer[index];
    }
    // 索引不合法就返回一个"无效"的空结构体
    TagDetection empty;
    empty.valid    = false;
    empty.id       = -1;
    empty.center_x = 0;
    empty.center_y = 0;
    empty.width    = 0;
    empty.height   = 0;
    empty.angle    = 0;
    return empty;
}

// ---- 检查传感器连接状态 ----
bool vision_is_connected() {
    return VisionSensor.installed();
}
