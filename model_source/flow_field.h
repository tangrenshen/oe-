#ifndef FLOW_FIELD_H
#define FLOW_FIELD_H

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QPolygonF>
#include <QPainterPath>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QMap>
#include <complex>
#include <functional>
#include <QRecursiveMutex>
#include <QImage>   // 🌟 核心 1：引入图像类，用于承载隐形掩码
#include "types.h"

// ================= 【新增】三维流速矢量结构体 =================
struct FlowVector3D {
    double u; // 东向速度 (m/s)
    double v; // 北向速度 (m/s)
    double w; // 垂向速度 (m/s, 向下为正，向上涌升为负)
};

// 全局仿真常量
const double T_TIDAL = 12.42 * 3600.0; // M2 分潮周期
const double T_SIMULATION_START = T_TIDAL / 4.0; // 仿真起始时间：涨潮1/4周期

// RBF 节点结构体
struct RBFNode {
    QPointF pos;
    double depth;
    std::complex<double> weight;
};

class FlowField : public QObject {
    Q_OBJECT
public:
    FlowField();

    // 【新增/恢复】 体育场形桥墩定义
    struct IdealizedPylon {
        double x;       // 中心 X
        double y;       // 中心 Y
        double R_head;  // 圆头半径
        double Offset;  // 圆心偏移量 (半长)
    };

    // --- 核心接口 ---
    // 【新增/恢复】 设置体育场形桥墩
    void setIdealizedPylons(const QVector<IdealizedPylon>& pylons);
    
    // 设置桥墩多边形 (直接使用海图或手动设置的 ChannelPoly)
    void setPylons(const QVector<ChannelPoly>& pylons);
    
    // 设置沙洲多边形 (用于流场计算)
    void setSandBars(const QVector<ChannelPoly>& sandBars);
    
    // 设置背景流参数
    void setBackgroundFlow(double U0, double flowAngleDeg);
    
    // 初始化全流域环境 (岸线、陆地、水深)
    void initGlobalFlow(const QVector<QPolygonF>& shorelines,
                         const QVector<ChannelPoly>& landPolygons,
                         const QVector<SoundingPoint>& soundings,
                         const QVector<ChannelPoly>& deparePolygons);

    // 设置潮汐观测站
    void setTidalSources(const QVector<FEBPoint>& sources);
    
    // 【新增】从数据库加载场源并自动计算曲率流向
    void loadAnchorsFromDb(QSqlDatabase &db);
    
    // 🌟 新增：航道切线流向自适应校准
    void autoAlignAnchorDirections(const QVector<QPointF>& referencePath);

    // 🌟 核心 2：向外暴露的极速探水雷达
    void buildRasterMask();
    bool isWaterArea(double x, double y) const;

    // ================= 【新增】三维高精度流场接口 =================
    FlowVector3D getFlowVector3DAt(double x, double y, double z, double t);

    // 计算某点流速 (核心计算函数)
    QPointF getFlowVectorAt(double x, double y, double t);
    QPointF computeAt(double x, double y, double t);

    // 获取某点水深
    double getDepthAt(double x, double y) const;

    // --- 录制与回放 ---
    void setRecordingMode(bool enable);
    void setPlaybackMode(bool enable);
    // 开启/关闭桥墩专注模式 (仅录制桥墩周围高精度数据)
    void setPylonFocusMode(bool enable) { m_pylonFocusMode = enable; }
    
    // 执行录制 (阻塞式/回调进度)
    void recordStartingFlowField(std::function<void(int)> progressCallback = nullptr);
    
    // 【新增】显式触发 RBF 权重矩阵求解 (用于系统初始化)
    void solveRBFWeights(double t);

    // 【顶托情景】径流受潮汐顶托的周期调制（情景参数化，非预报）：
    //   U_i(t) = U_i0 · [1 − β·cos(ω(t − τ_i))]，τ_i = (s_max − s_i) / c
    // s_i 为锚点沿航道中心线的里程（顺流增大），c 为潮波传播速度；t 与主程序仿真时间一致，
    // t = 0 时最下游锚点处顶托最强。β < 1 保证流向不反转。默认关闭，此时流场与原来完全一致
    struct BackwaterScenario {
        bool enabled;
        double amplitude;      // β，限定在 [0, 0.95]
        double period;         // 调制周期 (s)
        double celerity;       // 潮波传播速度 c (m/s)，浅水长波可取 sqrt(g·h)
        double updateInterval; // RBF 权重刷新间隔 (s，仿真时间)
        double phase;          // 相位偏置 (rad)：一次航行只占潮周期的约 0.2，
                               // 改变该相位即代表在不同潮时出航
    };
    void setBackwaterScenario(const BackwaterScenario& scenario);
    // 锚点在时刻 t 的调制系数（情景未启用时为 1）
    double backwaterFactor(const FEBPoint& pt, double t) const;

private:
    // --- 辅助结构体定义 ---    
    // 多边形SDF计算结果
    struct SDFResult {
        double distance; // 到多边形的距离
        QPointF normal;  // 法向量 (指向外部)
    };

    // --- 线程安全相关 ---    
    mutable QRecursiveMutex m_mutex; // 递归锁，保护核心数据

    // --- 内部算法函数 ---
    void buildDepthGrid();
    double getInterpolatedDepthOptimized(double x, double y) const;
    uint64_t getGridKey(double x, double y) const;

    void calculateDynamicRBFParam();
    // 按时刻 t 的锚点流速重解 RBF 权重（核矩阵不变）
    void updateRBFWeights(double t);
    QPointF rbfInterpolate(double x, double y) const;
    QPointF calculateTidalVector(const FEBPoint& pt, double t) const;
    double calculateAverageSourceSpeed(double t) const;
    
    double rbfKernel(double r) const;
    double calculateAnisotropicDist(double dx, double dy) const;

    // 几何辅助函数
    bool isInsidePylon(double x, double y) const;
    bool isInDepare(double x, double y) const;
    bool isNearPylon(double x, double y, double threshold) const;
    double getDistanceToNearestShore(double x, double y) const;
    double pointToSegmentDistance(const QPointF& p, const QPointF& a, const QPointF& b) const;
    
    // 【新增】计算点到多边形的最近距离 (用于几何排斥)
    double pointToPolygonDistance(const QPointF& p, const QPolygonF& poly) const;

    // ================= 【新增】地形梯度计算 (用于垂向涌升流) =================
    void computeDepthGradient(double x, double y, double& dHdx, double& dHdy) const;

    // 计算桥墩扰动 (新版几何排斥算法)
    QPointF computePylonPerturbation(double x, double y, double t, double referenceSpeed, double flowAngle) const;
    
    // 计算沙洲扰动 (SDF滑移流算法)
    QPointF computeSandBarPerturbation(double x, double y, double t, const QPointF& baseFlow) const;
    
    // 计算点到多边形的SDF
    SDFResult getPolygonSDF(const QPointF& p, const ChannelPoly& poly) const;

    // --- 数据库相关 ---
    void initRecordTable();
    void initSnapshotTable();
    void saveToCache(double t, double x, double y, double u, double v);
    bool loadFromCache(double t, double x, double y, QPointF& outFlow);

private:
    // 基础参数
    double m_U0;
    double m_flowAngleRad;
    
    // 桥墩数据
    QVector<FlowField::IdealizedPylon> m_idealizedPylons; // 体育场形桥墩 (用于流场计算)
    QVector<ChannelPoly> m_pylons; // 多边形桥墩 (用于显示)

    // RBF 参数
    double m_rbfShapeParam;
    double m_rbfDepthWeight;
    bool m_rbfInitialized;
    double m_lastRBFUpdateTime;
    QVector<RBFNode> m_rbfNodes;
    QVector<FEBPoint> m_rbfStations;        // 参与插值的锚点
    QVector<QVector<double>> m_rbfMatrix;   // 含正则项的核矩阵，只随锚点变化

    // 顶托情景
    BackwaterScenario m_backwater = {false, 0.0, T_TIDAL, 12.0, 5.0, 0.0};
    QMap<int, double> m_anchorAlongChannel; // 锚点 id -> 沿中心线里程 (m)
    double m_maxAlongChannel = 0.0;

    // 环境数据
    QVector<QPolygonF> m_shorelines;
    QVector<QRectF> m_shorelineBounds;
    QVector<SoundingPoint> m_soundings;
    QVector<ChannelPoly> m_landPolygons;
    QVector<ChannelPoly> m_sandBars; // 沙洲多边形
    QVector<ChannelPoly> m_deparePolygons;
    QVector<QRectF> m_depareBounds;
    QVector<FEBPoint> m_febSources;

    // 水深网格索引
    QMap<uint64_t, QVector<int>> m_depthGrid;
    const double m_gridSize = 100.0;

    // 掩码
    QPainterPath m_waterDomainMask;

    // 🌟 核心 3：掩码专用的内部变量
    QImage m_landMask;
    double m_maskMinX;
    double m_maskMaxY;
    double m_maskRes; // 栅格分辨率 (1像素代表几米)

    // 录制状态
    bool m_isRecording;
    bool m_isPlayback;
    bool m_pylonFocusMode; // 专注模式开关
    
    QSqlDatabase m_recordDb;
    QSqlDatabase m_snapshotDb;
    
    // 常量
    const double m_St = 0.2; // 斯特劳哈尔数
};

#endif // FLOW_FIELD_H
