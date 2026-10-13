#ifndef TYPES_H
#define TYPES_H
#include <QString>
#include <QVector>
#include <QPointF>

// 墨卡托是保角投影：同一点上 1 墨卡托米只相当于 cos(lat) 真实米。全工程的 X/Y 统一按参考纬度
// 缩放成真实米，所以每一处经纬度 <-> X/Y 的换算都必须乘/除 MERC_SCALE，
// 不要再写内联的 R*lon*pi/180 —— 漏乘会让该处的坐标与全局差 15%。
constexpr double MERC_SCALE = 0.8460443260071842; // cos(32.216 deg)，尹公洲航段平均纬度
constexpr double MERC_R = 6378137.0;              // WGS84 赤道半径

// AIS数据结构
struct AISData {
    QString time;
    QString mmsi;
    double longitude;
    double latitude;
    double speed;
    double heading;
    double course;
    // 根据CSV文件添加更多AIS数据字段
};

// 水深数据结构
struct DepthPoint {
    double x;      // mercator x
    double y;      // mercator y
    double depth;  // depth 水深
};

// 水深线数据结构
struct DepthLine {
    QVector<QPointF> points; // 水深线上的点序列
    double value;            // 水深值
};

// 【新增】水深散点数据结构 (用于 SOUNDG 处理)
struct SoundingPoint {
    double x;      // mercator x
    double y;      // mercator y
    double depth;  // 水深值 (m)
};

// 航道多边形数据结构
struct ChannelPoly {
    int id;
    int group;
    QString name;                  // 名称属性（用于显示桥梁名称等）
    QVector<QPointF> points;      // 原始边界（用于避碰检测）
    QVector<QVector<QPointF>> rings; // 完整环列表：rings[0] 外环，其后为内环（洞）
    QVector<QPointF> centerLine;  // 计划航线（用于航迹保持）
    double depth = 0.0; // 【新增】存储最小水深 (DRVAL1)
};

// 分隔线结构体
struct SeparationLine {
    int id;             // RCID
    QVector<QPointF> points; // 线上的点序列
};

// 【新增】浮标结构体 (用于 BOYLAT 处理)
struct Buoy {
    QPointF pos;       // 坐标 (mercator)
    int colorCode;     // 颜色代码 (3=红, 4=绿, 5=黄等)
    int catlam;        // 侧面标类别 (1=左, 2=右, 3=推荐右, 4=推荐左)
    QString name;      // 浮标名称 (用于调试和显示)
    int number;        // 浮标主编号 (用于排序)
    int subNumber = 0; // 子编号，如 "113-1" 的 1；无子编号时为 0

    // 重载 < 运算符，用于 std::sort 按编号和经度排序
    bool operator<(const Buoy& other) const {
        if (number != other.number) {
            return number < other.number;
        }
        if (subNumber != other.subNumber) {
            return subNumber < other.subNumber;
        }
        return pos.x() < other.pos.x();
    }
};

// 【新增】潮流观测点结构体 (用于 TS_FEB 表)
struct FEBPoint {
    int id;            // 数据库中的ID
    double x;           // mercator x
    double y;           // mercator y
    double floodSpeed;  // 涨潮最大流速 (m/s)
    double floodDir;    // 涨潮流向 (度, 真北方向)
    double ebbSpeed;    // 落潮最大流速 (m/s)
    double ebbDir;      // 落潮流向 (度, 真北方向)
    double refDepth;    // 该观测站所在位置的基准水深
    int type;           // 【新增】存储 CAT_TS: 1=涨潮站, 2=落潮站
};

#endif // TYPES_H