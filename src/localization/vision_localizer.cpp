// ============================================================================
//  localization/vision_localizer.cpp — 视觉定位的实现（AprilTag + 里程计融合）
// ============================================================================
//
//  【算法概览——怎么从"拍到一张照片"算出"我在哪"？】
//
//  对每一个检测到的 AprilTag 标签：
//
//    第 1 步：估算距离（从标签在图片里有多大来判断）
//      就像你看远处的人——越远的人在你视野里越小。
//      公式：距离 = (标签真实尺寸 × 焦距) / 标签在图片里的像素大小
//      这叫"小孔成像模型"——初中物理学过！
//
//    第 2 步：估算方位角（标签在图片里偏左还是偏右）
//      如果标签在图片正中间，方位角=0（正前方）
//      偏右就是正角度，偏左就是负角度
//      公式：方位角 = arctan(像素偏移量 / 焦距)
//
//    第 3 步：计算机器人位置
//      已知"标签在赛场上的坐标"和"我和标签的距离、角度"，
//      就能算出"我自己在赛场上的坐标"。
//      就像看到远处一个你知道位置的路标，你就能判断自己在哪。
//
//    第 4 步：置信度评分
//      越近的标签、在图片里越大的标签，结果越可信。
//      太远或太小的标签会被丢弃（不可信）。
//
//  如果同时看到多个标签，取置信度最高的那个。
//
//  【视觉 → 里程计融合】
//    不是直接替换里程计的位置（那样会跳来跳去），而是慢慢修正：
//    新位置 = (1 - α) × 里程计位置 + α × 视觉位置
//    α 越大修正越猛，越小修正越柔和。这叫"互补滤波器"。
//
// ============================================================================
#include "localization/vision_localizer.h"
#include "config.h"
#include "hal/vision.h"
#include "hal/hal_log.h"
#include <cmath>

// ============================================================================
//  赛场标签地图 —— 每个标签在赛场上的已知位置
// ============================================================================
//
//  ⚠ 根据你的具体赛场布局修改这里！
//  每个标签需要：ID、X坐标、Y坐标、离地高度、朝向角度
//
//  VEX V5 赛场尺寸：3.6576m × 3.6576m（12英尺 × 12英尺）
//  标签通常贴在赛场四周的围墙上。
//
//  facing（朝向）= 标签表面法线方向（标签面朝哪个方向）
//    例如：贴在左墙上的标签面朝右 → facing = 0（+x 方向）
//
//  赛场俯视图：
//  ┌───────────────────────────────────┐
//  │             +y 墙壁               │  ← 标签面朝 -y (3π/2)
//  │ Tag3                         Tag4 │
//  │                                   │
//  │ +x墙壁                      -x墙壁│
//  │ Tag1                         Tag2 │
//  │ (面朝+x)                 (面朝-x) │
//  │                                   │
//  │ Tag5                         Tag6 │
//  │             -y 墙壁               │  ← 标签面朝 +y (π/2)
//  └───────────────────────────────────┘
//     原点(0,0) = 左下角
//
static const FieldTag FIELD_TAGS[] = {
    // ID,  X坐标(m), Y坐标(m), 高度(m), 朝向(弧度)
    {  1,    0.0,      1.22,     0.15,   0.0         },  // 左墙下方
    {  2,    3.6576,   1.22,     0.15,   M_PI        },  // 右墙下方
    {  3,    0.0,      2.44,     0.15,   0.0         },  // 左墙上方
    {  4,    3.6576,   2.44,     0.15,   M_PI        },  // 右墙上方
    {  5,    0.91,     0.0,      0.15,   M_PI / 2    },  // 下墙左侧
    {  6,    2.74,     0.0,      0.15,   M_PI / 2    },  // 下墙右侧
    {  7,    0.91,     3.6576,   0.15,   3*M_PI / 2  },  // 上墙左侧
    {  8,    2.74,     3.6576,   0.15,   3*M_PI / 2  },  // 上墙右侧
};
static constexpr int NUM_FIELD_TAGS = sizeof(FIELD_TAGS) / sizeof(FIELD_TAGS[0]);

// 记录上一次拍到了几个标签
static int last_tag_count = 0;

// ============================================================================
//  辅助函数
// ============================================================================

/// 查找标签 ID 对应的赛场已知位置。找不到返回 nullptr
static const FieldTag* find_field_tag(int id) {
    for (int i = 0; i < NUM_FIELD_TAGS; ++i) {
        if (FIELD_TAGS[i].id == id) return &FIELD_TAGS[i];
    }
    return nullptr;
}

/// 用"小孔成像模型"估算标签距离
/// 原理：真实大小 × 焦距 / 像素大小
/// 就像你用手指比划——远处的东西在指间显得更小
static double estimate_distance(double pixel_size) {
    if (pixel_size < MIN_TAG_PIXELS) return -1.0;  // 标签太小，不可信
    return (APRILTAG_REAL_SIZE * VISION_FOCAL_LENGTH) / pixel_size;
}

/// 估算标签的方位角（标签在图片中偏左还是偏右）
/// 图片中心 = 正前方（方位角=0）
/// 偏右 = 正角度，偏左 = 负角度
static double estimate_bearing(double center_x) {
    double pixel_offset = center_x - (VISION_IMAGE_WIDTH / 2.0);
    return atan2(pixel_offset, VISION_FOCAL_LENGTH);
}

/// 计算置信度（0~1，越大越可信）
/// 距离越近 + 标签在图片里越大 → 越可信
static double compute_confidence(double distance, double pixel_size) {
    if (distance <= 0 || distance > MAX_VISION_RANGE) return 0.0;

    // 距离因子：越近越好（线性递减）
    double dist_conf = 1.0 - (distance / MAX_VISION_RANGE);
    if (dist_conf < 0) dist_conf = 0;

    // 大小因子：图片里标签越大越好
    // 100 像素大约对应近距离，设为满分 1.0
    double size_conf = pixel_size / 100.0;
    if (size_conf > 1.0) size_conf = 1.0;

    // 两个因子相乘 → 只有距离近且标签够大时，置信度才高
    return dist_conf * size_conf;
}

// ============================================================================
//  公开接口
// ============================================================================

void vision_localizer_init() {
    last_tag_count = 0;
    hal_log("Vision localizer initialized with " + to_str(NUM_FIELD_TAGS) + " field tags");
}

// ---- 拍照 + 处理标签 → 返回最佳位置估算 ----
VisionEstimate vision_localizer_update() {
    // 先准备一个"无效"的结果（如果什么都没看到就返回这个）
    VisionEstimate best_estimate;
    best_estimate.valid = false;
    best_estimate.confidence = 0.0;
    best_estimate.x = 0;
    best_estimate.y = 0;
    best_estimate.heading = 0;

    // 拍一张照
    int count = vision_snapshot();
    last_tag_count = count;

    if (count == 0) return best_estimate;  // 什么都没看到

    // 获取里程计的当前航向（需要它来从"摄像头视角"转换到"赛场视角"）
    Pose current = get_pose();

    // 逐个处理检测到的标签
    for (int i = 0; i < count; ++i) {
        TagDetection tag = vision_get_tag(i);
        if (!tag.valid) continue;

        // 查找这个标签在赛场上的已知位置
        const FieldTag* field_tag = find_field_tag(tag.id);
        if (field_tag == nullptr) {
            hal_log("Vision: unknown tag ID " + to_str(tag.id) + ", skipped");
            continue;  // 不认识的标签，跳过
        }

        // 第 1 步：从标签像素大小估算距离
        double pixel_size = (tag.width > tag.height) ? tag.width : tag.height;
        double distance = estimate_distance(pixel_size);
        if (distance < 0) continue;  // 标签太远/太小，不可信

        // 第 2 步：估算方位角（摄像头视角）
        double bearing_camera = estimate_bearing(tag.center_x);

        // 第 3 步：把摄像头视角的方位角转换成赛场视角
        // 赛场方位 = 机器人当前航向 + 摄像头安装角度 + 摄像头看到的角度
        double bearing_field = current.theta + VISION_CAMERA_ANGLE + bearing_camera;

        // 第 4 步：从标签位置反推机器人位置
        // 逻辑：已知标签在 (tag_x, tag_y)，机器人在距标签 distance 的方向上
        // 还要减去摄像头相对机器人中心的偏移量
        double est_x = field_tag->x - distance * cos(bearing_field)
                      - VISION_CAMERA_OFFSET_X * cos(current.theta)
                      + VISION_CAMERA_OFFSET_Y * sin(current.theta);
        double est_y = field_tag->y - distance * sin(bearing_field)
                      - VISION_CAMERA_OFFSET_X * sin(current.theta)
                      - VISION_CAMERA_OFFSET_Y * cos(current.theta);

        // 第 5 步：计算置信度
        double conf = compute_confidence(distance, pixel_size);

        // 保留置信度最高的估算结果
        if (conf > best_estimate.confidence) {
            best_estimate.x          = est_x;
            best_estimate.y          = est_y;
            best_estimate.heading    = current.theta;  // 航向仍用里程计的（更准）
            best_estimate.confidence = conf;
            best_estimate.valid      = true;
        }
    }

    if (best_estimate.valid) {
        hal_log("Vision est: (" + to_str(best_estimate.x) + ", "
                + to_str(best_estimate.y) + ") conf="
                + to_str(best_estimate.confidence));
    }

    return best_estimate;
}

// ---- 把视觉结果融合到里程计中 ----
void vision_correct_odometry(const VisionEstimate& estimate) {
    // 无效或置信度太低→不修正
    if (!estimate.valid) return;
    if (estimate.confidence < VISION_MIN_CONFIDENCE) return;

    Pose current = get_pose();

    // 互补滤波器：α 越大，越信任视觉；越小，越信任里程计
    // 实际 α = 基础 α × 置信度（置信度越高，修正力度越大）
    double alpha = VISION_CORRECTION_ALPHA * estimate.confidence;

    // α 不能太大，防止一次异常的视觉读数导致位置大幅跳跃
    if (alpha > VISION_MAX_CORRECTION_ALPHA) alpha = VISION_MAX_CORRECTION_ALPHA;

    // 加权融合：新位置 = (1-α) × 里程计 + α × 视觉
    Pose corrected;
    corrected.x     = (1.0 - alpha) * current.x     + alpha * estimate.x;
    corrected.y     = (1.0 - alpha) * current.y     + alpha * estimate.y;
    corrected.theta = current.theta;  // 航向角不用视觉修正（IMU 更可靠）

    // 异常值检测：如果修正量太大（超过安全阈值），说明可能是误检
    // 直接拒绝这次修正，防止机器人"瞬移"
    double dx = corrected.x - current.x;
    double dy = corrected.y - current.y;
    double correction_dist = sqrt(dx * dx + dy * dy);

    if (correction_dist < VISION_MAX_CORRECTION_M) {
        // 修正量合理 → 应用（用 set_pose_no_reset 轻轻微调，不打断编码器）
        set_pose_no_reset(corrected);
        hal_log("Vision correction applied: dx=" + to_str(dx)
                + " dy=" + to_str(dy) + " alpha=" + to_str(alpha));
    } else {
        // 修正量太大 → 拒绝（可能是误检或传感器异常）
        hal_log("Vision correction REJECTED: dist=" + to_str(correction_dist)
                + " > max=" + to_str(VISION_MAX_CORRECTION_M));
    }
}

int vision_localizer_tag_count() {
    return last_tag_count;
}
