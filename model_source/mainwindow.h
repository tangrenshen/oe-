#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtWidgets/QMainWindow>
#include <new> // 新增头文件，用于placement new
#include <QPointF>
#include <QVector>
#include <QFile>
#include <QTextStream>
#include <QMap>
#include <QPair>
#include <QTimer>
#include <QPushButton> // 新增头文件
#include <QLabel> // 新增头文件
#include "qcustomplot.h"
#include <QFileDialog>
#include <QMessageBox>
#include "ship.h"
#include "huiyu.h"
#include "types.h" // 包含数据结构定义
#include "flow_field.h"
#include "baseline_controller.h"
#include <QCheckBox>
#include <QComboBox>
#include "trajectory_predictor.h"

// 全局日志计数器，用于控制所有日志的输出频率
extern int g_globalLogCounter;

namespace Ui {
class MainWindow;
}

extern void lonLat2Mercator(double lon, double lat, double &x, double &y);

extern void Mercator2lonLat(double x, double y, double &lon, double &lat);

extern void lonLat2UTM(double lon, double lat, double &x, double &y);

extern void UTM2LonLat(double x, double y, double &lon, double &lat);

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QPointF mercatorPoint(double lon, double lat);

    // 【新增】桥墩中心结构体，用于绘制桥墩标记
    struct PylonCenter {
        double x; double y; QString name;
    };

    // 辅助函数：计算两点间的航海方位角 (北=0, 顺时针)
    double calculateBearing(QPointF p1, QPointF p2);

    // 新增辅助函数：只计算LOS目标航向，不操舵
    double calculateLOSAimCourse(Ship &ship, bool isPrediction = false);
    
    // 新增辅助函数：执行PID控制
    void runPIDControl(Ship &ship, double targetHeading, float dt);

    // 【新增】公开环境查询接口，供预测器使用
    QPointF getFlowAt(double x, double y, double t) {
        return m_bridgeEnv.computeAt(x, y, t);
    }

    // 无界面批处理（命令行 --batch，见 main.cpp 与 batch_runner.cpp）：
    // 按配置运行一次仿真并导出 CSV，返回进程退出码
    struct BatchConfig {
        int group = 3;                     // 对照组 0~3
        QString outputDir;                 // 输出目录，空则为程序所在目录
        double maxSimSeconds = 7200.0;     // 最长仿真时间 (s)
        double backwaterAmplitude = 0.0;   // 顶托调制幅度 β，0 表示关闭
        double backwaterCelerity = 12.0;   // 潮波传播速度 (m/s)
        double backwaterPeriod = T_TIDAL;  // 调制周期 (s)
        double backwaterPhaseDeg = 0.0;    // 起始潮时（相位，度）
        bool shallowWater = true;          // 是否做浅水修正
        QString dumpLanePath;              // 非空则只导出计划航线坐标后退出，不做仿真
        QString dumpFlowPath;              // 非空则只导出重构流场网格后退出，供绘图用
        QString dumpCellsPath;             // 非空则只导出沿航线的八单元受流采样后退出
        int cellSplit = 4;                 // 单元采样的纵向等分数（默认 4，即模型实际使用的 4x2）
        int fsiSplit = 4;                  // 仿真中流固耦合的纵向等分数（默认 4 = 原 4x2 八单元）
        // 移植导数的不确定性：对 16 项横向/首摇导数各乘 (1 + frac·U(-1,1))，R0 不扰动（由自航点标定）
        double derivPerturb = 0.0;         // 扰动幅度，0 表示不扰动
        int derivSeed = 1;                 // 扰动随机种子
        double flowStep = 120.0;           // 流场导出网格步长 (m)；单元采样时为沿航线站距
        double flowTime = 0.0;             // 流场导出时刻 (s)
        // 本文算法（Group 4）的控制参数，默认值与代码中原有取值一致，便于批量整定
        double kp = 8.0;
        double ti = 50.0;
        double td = 2.0;
        double kff = 3.0e-6;
        double baseKp = -1.0;              // 对照组 1~3 的内环比例增益，<=0 表示沿用原值
        // 退化实验：前馈拿到的流场估计误差 + 偏差量测噪声。默认即完美信息，与原结果一致
        double ffGain = 1.0;               // 前馈流速幅值系数（0.7 / 1.3 = 估计偏小/偏大 30%）
        double ffBiasDeg = 0.0;            // 前馈流向偏差 (度)
        double ffLagSec = 0.0;             // 前馈流场估计滞后 (s)，按历史船位取流
        double xteNoiseM = 0.0;            // 横向偏差量测噪声标准差 (m)，一阶相关，时间常数 60 s
        int noiseSeed = 1;                 // 噪声种子，保证可复现
        // 命令转速：对照组原为 100、本文算法原为 94.5，两者不等会让本文算法用时多约 10%，
        // 对齐后重跑可回答“精度是不是靠开得慢换来的”
        double baseRpm = 100.0;            // 对照组 1~3 的命令转速
        double ownRpm = 94.5;              // 本文算法（组 4）的命令转速
    };
    int runBatch(const BatchConfig& config);

private:
    // 退化实验参数：批处理时由 runBatch 写入，界面运行保持完美信息
    double m_baseRpm = 100.0;
    double m_ownRpm = 94.5;
    double m_ffGain = 1.0;
    double m_ffBiasRad = 0.0;
    double m_ffLagSec = 0.0;
    double m_xteNoiseSigma = 0.0;
    double m_xteNoiseState = 0.0;
    struct PoseSample { double t; double x; double y; };
    QVector<PoseSample> m_poseHistory;   // 供前馈滞后取用的历史船位

    Ui::MainWindow *ui;
    QSqlDatabase m_db; // 数据库连接
    QMap<int, QVector<QVector<QPointF>>> m_waterwayPointsByGrup; // 按GRUP分组的航道点
    
    // 【新增】存储本船的历史轨迹点
    QVector<QPointF> m_ownShipHistory;
    QVector<DepthLine> m_depthLines;    // 水深线数据

    // 航道数据存储
    QCustomPlot* m_aisPlot;   // 航道 + 中心线可视化
    QTimer* m_timer; // 仿真定时器
    
    // 自动驾驶控制变量 ---
    float m_prevHeading_1;  // 上一步的航向 (k-1)
    float m_prevHeading_2;  // 上上步的航向 (k-2)
    float m_currentRudderRad; // 当前舵角 (弧度)，用于增量累加
    double m_intXTE; // ILOS积分项，用于消除稳态误差
    double m_filteredDrift = 0.0; // 滤波后的流场漂角，用于平滑前馈
    
    // 船舶运动和态势感知相关成员
    Ship m_ownShip; // 本船对象
    HuiYu m_huiyu; // 态势感知对象
    float m_currentRudderAngle; // 当前舵角（度）
    QVector<Ship> m_targetShips; // 目标船列表
    
    // 日志输出控制
    // int m_logCounter; // 日志输出计数器，用于控制30帧输出一次日志（已改为全局计数器）
    
    FlowField m_bridgeEnv; // 新增环境类实例
    double m_totalSimTime = 0.0; // 新增累计仿真时间
    bool m_isSimulating = false; // 仿真状态标志
    TrajectoryPredictor* m_trajectoryPredictor; // 新增轨迹预测器实例
    bool m_futureRiskWarning = false; // 预测系统风险警告标志

    // 对照组控制器
    BaselineController* m_baselineController;

    // 本文算法的控制参数（可由批处理命令行覆盖，见 BatchConfig）
    double m_gainKp = 8.0;
    double m_gainTi = 50.0;
    double m_gainTd = 2.0;
    double m_gainKff = 3.0e-6;

    // 批处理状态：批处理时不弹消息框、不重绘，录制文件写到 outputDir()
    bool m_batchMode = false;
    bool m_batchArrived = false;
    QString m_outputDir;
    QString outputDir() const;
    
    // UI 开关指针
    QComboBox* m_cmbControlMode;

    void initDatabase(); // 初始化数据库连接
    void loadChannelData(); // 加载航道数据
    void drawWaterways(); // 绘制航道
    void drawBackgroundLayers(); // 【新增】专用背景绘制函数

    void onTimerTick();
    void loadDepthData();
    void drawOwnShip(); // 绘制本船
    
    // WKT解析辅助函数
    bool parseWktPoint(const QString &wkt, double &lon, double &lat);
    QVector<QPointF> parseWktPoints(const QString &wkt);          // 仅外环
    QVector<QVector<QPointF>> parseWktRings(const QString &wkt);   // 全部环，rings[0] 为外环

// Buoy结构体已在types.h中定义

private:
    // 航迹保持相关函数
    QVector<QPointF> calculateCenterLine(const QVector<QPointF>& polyPoints); // 提取多边形的中心线
    double pointToLineDistance(const QPointF& point, const QPointF& lineStart, const QPointF& lineEnd); // 计算点到线段的距离
    double calculateSignedDistance(QPointF p, QPointF p1, QPointF p2); // 计算点到线段的有向距离 (Cross Track Error)
    
    // 【新增】生成平滑曲线插值点 (Catmull-Rom Spline)
    // points: 原始浮标点 (已排序)
    // segments: 每两个点之间插入多少个点 (例如 20)
    QVector<QPointF> generateSmoothCurve(const QVector<Buoy>& buoys, int segments);
    
    // 【新增】曲线重采样函数 (将任意曲线转化为等间距的固定数量点)
    QVector<QPointF> resamplePolyline(const QVector<QPointF>& points, int targetCount);
    
    // 【新增】强制锁定坐标轴的1:1物理比例
    void enforceAspectRatio();
    
    // 航道数据存储
    // 【新增】桥墩数据 (PYLONS)
    QVector<ChannelPoly> m_pylonsPolygons;

    // 【新增】疏浚航道数据 (DRGARE) - 替代原来的 TSSLPT
    QVector<ChannelPoly> m_dredgedPolygons;

    // 【保留】陆地 (LNDARE)
    QVector<ChannelPoly> m_landPolygons;        // 存储 LNDARE (陆地) - Group 1
    QVector<ChannelPoly> m_sandBars;            // 存储 LNDARE 中的沙洲 (名称包含"Sha"的陆地) - Group 1

    // 【新增】岸线 (COALNE)
    QVector<QPolygonF> m_shorelinePolygons;     // 存储 COALNE (岸线) - 用于流场边界条件

    // 【保留】桥梁 (BRIDGE)
    QVector<ChannelPoly> m_bridgePolygons;    // BRIDGE (桥梁) - 苏通大桥的关键数据

    // 【保留】水深区 (DEPARE) - 水域背景
    QVector<ChannelPoly> m_depthPolygons;     // DEPARE (水深区/水域)
    
    // 【新增】功能区域数据容器
    QVector<ChannelPoly> m_anchoragePolygons;   // 锚地 (ACHARE)
    QVector<ChannelPoly> m_obstructionPolygons; // 障碍物 (OBSTRN)
    QVector<ChannelPoly> m_wreckPolygons;       // 沉船 (WRECKS)

    // 【新增】水深散点数据
    QVector<SoundingPoint> m_soundings; // SOUNDG数据，存储水深散点

    // 【删除】以下变量已注释掉
    // QVector<ChannelPoly> m_channelPolygons; // 旧 TSSLPT
    // QVector<ChannelPoly> m_fairwayPolygons; // 旧 FAIRWY
    // QVector<SeparationLine> m_separationLines; // 旧 TSELNE
    
    // 【新增】虚拟航道边界点
    // 【新增】存储原始浮标数据
    QVector<Buoy> m_allBuoys;
    
    // 【新增】存储潮流观测站数据
    QVector<FEBPoint> m_febPoints;
    
    // 【保留】存储计算后的边界线（用于画虚线和填充）
    QVector<QPointF> m_virtualRedBoundary;   // 红色浮标连线
    QVector<QPointF> m_virtualGreenBoundary; // 绿色浮标连线
    
    // 【新增】存储计算后的中心线
    QVector<QPointF> m_virtualCenterLine;
    
    // 【新增】存储细分后的计划航线
    QVector<QPointF> m_laneUpstream;   // 上行航线 (往西/北侧)
    QVector<QPointF> m_laneDownstream; // 下行航线 (往东/南侧)
    
    // 【新增】存储文字标签的指针，用于后续控制显隐
    QCPItemText* m_bridgeLabel = nullptr;   // 桥梁名称标签
    QList<QCPItemText*> m_buoyLabels;       // 所有浮标编号标签
    QList<QCPItemText*> m_febLabels;        // 潮流观测站标签
    
    // 【新增】界面控件与标签容器
    QPushButton *m_btnRun;            // "开始仿真"按钮
    QPushButton *m_btnToggleFlow;     // 【新增】流场数据导出按钮
    QPushButton *m_btnRecordFlow;     // 【新增】录制全流域流场按钮
    QPushButton *m_btnRecordValidation; // 【新增】验证录制按钮
    
    // CSV 录制专用变量
    QFile m_csvFile;
    QTextStream m_csvStream;
    bool m_isRecordingCSV = false; // 录制开关
    QCheckBox *m_chkPylonMode;        // 【新增】桥墩紊流专注模式复选框
    QList<QCPItemText*> m_depthLabels;// 管理水深数值标签，方便刷新时清理
    
    // 【新增】仪表盘控件
    QLabel *m_lblSpeed;   // 航速
    QLabel *m_lblHeading; // 航向
    QLabel *m_lblRudder;  // 舵角
    QLabel *m_lblStatus;  // 状态 (巡航/避碰)
    // 【新增】环境数据显示控件
    QLabel *m_lblDepthVal; // 水深数值
    QLabel *m_lblFlowSpdVal; // 流速数值
    QLabel *m_lblFlowDirVal; // 流向数值
    // 【新增】仿真时间显示
    QLabel *m_lblSimTime; // 仿真时间
    
    // 【新增】桥区水域定义
    QVector<Buoy> m_suqiaoBuoys;    // 存储所有 "Suqiao" 前缀的浮标
    QPolygonF m_bridgeZonePolygon;  // 由浮标围成的桥区多边形区域
    
    
private slots:
    // 【新增】处理视图缩放的槽函数
    void updateLabelsVisibility();
    
    // 【新增】开始仿真槽函数
    void onBtnRunClicked();
    
    // 【新增】流场数据导出槽函数
    void onBtnToggleFlowClicked();
    
    // 【新增】录制全流域流场槽函数
    void onBtnRecordFlowClicked();
    
    // 【新增】 录制按钮点击槽函数
    void onBtnRecordValidationClicked();
};

#endif // MAINWINDOW_H
