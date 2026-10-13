#include <new> // 添加<new>头文件，用于placement new
#include <QDebug>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QProgressDialog>
#include <QtConcurrent>
#include "mainwindow.h"
#include <random>
#include "ui_mainwindow.h"
#include "ship.h"
#include <QCoreApplication>
#include <math.h>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QSqlError>
#include <QStringList>
#include <QMap>
#include <QPair>
#include <algorithm>
#include <QRegularExpression>
#include "qcustomplot.h"
#include "flow_field.h"
#pragma execution_character_set("utf-8")

// 定义地球半径（米）
const double R = 6378137.0;

// 退化实验用的噪声源。放在这里而不是成员里，是为了不把 <random> 塞进头文件；
// 种子由 seedSimNoise 设置，保证同一条命令行可复现
static std::mt19937 g_noiseGen(1);
static std::normal_distribution<double> g_norm(0.0, 1.0);
void seedSimNoise(unsigned seed) { g_noiseGen.seed(seed); }

// 墨卡托是保角投影：同一点上 1 墨卡托米只相当于 cos(lat) 真实米。船舶运动学按物理米积分，
// 若把速度直接积到墨卡托坐标上，航道几何等于被放大 1/cos(lat) 倍（北纬 32.2 度为 1.18 倍），
// 航程、弯道半径、横向偏差随之全部偏大 18%。这里在投影出口处统一乘参考纬度的比例因子，
// 使全工程的 X/Y 都是真实米。比例因子 MERC_SCALE 定义在 types.h，全工程共用一份。

// 定义全局日志计数器
int g_globalLogCounter = 0;

extern void Mercator2lonLat(double x, double y, double &lon, double &lat);
extern void lonLat2UTM(double lon, double lat, double &x, double &y);
extern void UTM2LonLat(double x, double y, double &lon, double &lat);
extern void lonLat2Mercator(double lon, double lat, double &x, double &y);

// 将S-57颜色代码转换为Qt颜色
QColor s57ToQtColor(int code) {
    switch(code) {
        case 1: return Qt::white;
        case 2: return Qt::black;
        case 3: return Qt::red;
        case 4: return Qt::green;
        case 5: return Qt::blue;
        case 6: return Qt::yellow;
        case 7: return Qt::gray;
        case 11: return QColor(255, 165, 0); // Orange
        case 12: return Qt::magenta;
        default: return Qt::gray; // 默认颜色
    }
}

// 从经纬度转换为墨卡托坐标
inline void lonLat2Mercator(double lon, double lat, double &x, double &y)
{
    // 限制经度在 [-180, 180] 范围内
    lon = qBound(-180.0, lon, 180.0);

    // 限制纬度在 [-89.9, 89.9] 范围内（避免极点计算问题）
    lat = qBound(-89.9, lat, 89.9);

    // 墨卡托投影转换公式
    x = MERC_SCALE * R * lon * M_PI / 180.0;
    y = MERC_SCALE * R * log(tan(M_PI / 4.0 + lat * M_PI / 360.0));
}

// 从墨卡托坐标转换为经纬度
void Mercator2lonLat(double x, double y, double &lon, double &lat)
{
    const double mx = x / MERC_SCALE, my = y / MERC_SCALE;
    lon = mx * 180.0 / (R * M_PI);
    lat = (2.0 * atan(exp(my / R)) - M_PI / 2.0) * 180.0 / M_PI;
}

// 从经纬度转换为UTM坐标
void lonLat2UTM(double lon, double lat, double &x, double &y)
{
    // UTM投影参数
    const double a = 6378137.0; // 地球半长轴
    const double f = 1.0 / 298.257223563; // 扁率
    const double k0 = 0.9996; // 比例因子
    const double e = sqrt(f * (2 - f)); // 第一偏心率
    const double e2 = e * e; // 第一偏心率平方
    const double e4 = e2 * e2;
    const double e6 = e4 * e2;
    const double e1sq = e2 / (1 - e2); // e'^2

    // 确保纬度在有效范围内
    lat = qBound(-89.9, lat, 89.9);
    lon = qBound(-180.0, lon, 180.0);

    // 计算UTM带号
    int zone = floor((lon + 180.0) / 6.0) + 1;

    // 中央经线
    double lon0 = (zone - 1) * 6.0 - 180.0 + 3.0;

    double latRad = lat * M_PI / 180.0;
    double lonRad = lon * M_PI / 180.0;
    double lon0Rad = lon0 * M_PI / 180.0;

    // 计算自然纬度
    double N = a / sqrt(1 - e2 * sin(latRad) * sin(latRad));
    double T = tan(latRad) * tan(latRad);
    double C = e1sq * cos(latRad) * cos(latRad);
    double A = cos(latRad) * (lonRad - lon0Rad);

    // 计算M (经线曲率半径)
    double M = a * ((1 - e2/4 - 3*e4/64 - 5*e6/256) * latRad
                   - (3*e2/8 + 3*e4/32 + 45*e6/1024) * sin(2*latRad)
                   + (15*e4/256 + 45*e6/1024) * sin(4*latRad)
                   - (35*e6/3072) * sin(6*latRad));

    // 计算UTM东坐标
    x = k0 * N * (A + (1 - T + C) * pow(A, 3)/6 + (5 - 18*T + T*T + 72*C - 58*e1sq) * pow(A, 5)/120) + 500000.0;

    // 计算UTM北坐标
    y = k0 * (M + N * tan(latRad) * (pow(A, 2)/2 + (5 - T + 9*C + 4*C*C) * pow(A, 4)/24 + (61 - 58*T + T*T + 600*C - 330*e1sq) * pow(A, 6)/720));

    // 南纬区域调整
    if (lat < 0.0) {
        y += 10000000.0;
    }
}

// 从UTM坐标转换为经纬度
void UTM2LonLat(double x, double y, double &lon, double &lat)
{
    // UTM投影参数
    const double a = 6378137.0;
    const double f = 1/298.257223563;
    const double k0 = 0.9996;
    const double e = sqrt(f*(2-f));
    const double e1sq = (f*(2-f))/pow(1-f,2);
    const double e2 = e*e;
    const double e4 = e2*e2;
    const double e6 = e4*e2;
    const double e8 = e6*e2;

    // 移除UTM东坐标偏移
    x -= 500000.0;

    // 移除UTM北坐标偏移（南纬情况）
    bool isSouthHemisphere = (y >= 10000000.0);
    if (isSouthHemisphere) {
        y -= 10000000.0;
    }

    // 计算UTM带号
    int zone = 51; // 默认值，根据实际情况调整
    double lon0 = (zone - 1)*6 - 180 + 3;

    double M = y / k0;
    double mu = M / (a*(1 - e2/4 - 3*e4/64 - 5*e6/256 - 35*e8/32768));

    double e1 = (1 - sqrt(1-e2))/(1 + sqrt(1-e2));
    double J1 = (3*e1/2 - 27*e1*e1*e1/32);
    double J2 = (21*e1*e1/16 - 55*e1*e1*e1*e1/32);
    double J3 = (151*e1*e1*e1/96);
    double J4 = (1097*e1*e1*e1*e1/512);

    double phi1Rad = mu + J1*sin(2*mu) + J2*sin(4*mu) + J3*sin(6*mu) + J4*sin(8*mu);
    double phi1 = phi1Rad * 180.0/M_PI;

    double N1 = a / sqrt(1-e2*sin(phi1Rad)*sin(phi1Rad));
    double T1 = tan(phi1Rad)*tan(phi1Rad);
    double C1 = e1sq*cos(phi1Rad)*cos(phi1Rad);
    double R1 = a*(1-e2) / pow(1-e2*sin(phi1Rad)*sin(phi1Rad), 1.5);
    double D = x / (N1*k0);

    double phiRad = phi1Rad - (N1*tan(phi1Rad)/R1) * (D*D/2 - (5 + 3*T1 + 10*C1 - 4*C1*C1 - 9*e1sq)*D*D*D*D/24 + (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*e1sq - 3*C1*C1)*D*D*D*D*D*D/720);
    lat = phiRad * 180.0/M_PI;

    double lonRad = lon0*M_PI/180.0 + (D - (1 + 2*T1 + C1)*D*D*D/6 + (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*e1sq + 24*T1*T1)*D*D*D*D*D/120) / cos(phi1Rad);
    lon = lonRad * 180.0/M_PI;

    // 确保经度在 [-180, 180] 范围内
    if (lon > 180.0) lon -= 360.0;
    if (lon < -180.0) lon += 360.0;

    // 恢复南纬标记
    if (isSouthHemisphere && lat > 0.0) {
        lat = -lat;
    }
}

// 将经纬度转换为墨卡托坐标点
QPointF MainWindow::mercatorPoint(double lon, double lat) {
    double x, y;
    lonLat2Mercator(lon, lat, x, y);
    return QPointF(x, y);
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 【修改 1】更新软件标题
    this->setWindowTitle(QString("船舶数字孪生自主航行系统"));

    QWidget *centralWidget = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // =========================================================
    // 【新增】 1. 顶部仪表盘布局
    // =========================================================
    QHBoxLayout *dashboardLayout = new QHBoxLayout();
    dashboardLayout->setContentsMargins(10, 10, 10, 0);

    // 样式表：黑底绿字，像雷达屏幕
    QString labelStyle = "QLabel { background-color: #222; color: #00FF00; font: bold 14px 'Consolas'; padding: 5px; border: 1px solid #555; border-radius: 4px; }";

    // 初始化标签
    m_lblSpeed = new QLabel("对水 0.0 | 对地 0.0 kn");
    m_lblSpeed->setStyleSheet(labelStyle);
    m_lblSpeed->setFixedWidth(120);

    m_lblHeading = new QLabel("HDG: 000.0°");
    m_lblHeading->setStyleSheet(labelStyle);
    m_lblHeading->setFixedWidth(120);

    m_lblRudder = new QLabel("Rudder: 00.0°");
    m_lblRudder->setStyleSheet(labelStyle);
    m_lblRudder->setFixedWidth(120);

    // 【新增】环境数据标签
    m_lblDepthVal = new QLabel("Depth: 0.0 m");
    m_lblDepthVal->setStyleSheet(labelStyle);
    m_lblDepthVal->setFixedWidth(120);

    m_lblFlowSpdVal = new QLabel("Flow: 0.0 kn");
    m_lblFlowSpdVal->setStyleSheet(labelStyle);
    m_lblFlowSpdVal->setFixedWidth(120);

    m_lblFlowDirVal = new QLabel("Dir: 000°");
    m_lblFlowDirVal->setStyleSheet(labelStyle);
    m_lblFlowDirVal->setFixedWidth(120);

    // 【新增】仿真时间标签
    m_lblSimTime = new QLabel("TIME: 0 s");
    m_lblSimTime->setStyleSheet(labelStyle);
    m_lblSimTime->setFixedWidth(120);

    m_lblStatus = new QLabel("MODE: STANDBY");
    m_lblStatus->setStyleSheet("QLabel { background-color: #222; color: white; font: bold 14px 'Arial'; padding: 5px; border-radius: 4px; }");
    m_lblStatus->setAlignment(Qt::AlignCenter);

    // 添加到布局
    dashboardLayout->addWidget(m_lblSpeed);
    dashboardLayout->addWidget(m_lblHeading);
    dashboardLayout->addWidget(m_lblRudder);
    dashboardLayout->addWidget(m_lblDepthVal);
    dashboardLayout->addWidget(m_lblFlowSpdVal);
    dashboardLayout->addWidget(m_lblFlowDirVal);
    dashboardLayout->addWidget(m_lblSimTime); // 放在流向和状态栏之间
    dashboardLayout->addWidget(m_lblStatus, 1); // 状态栏占据剩余空间

    // 将仪表盘放入主布局的最上方
    mainLayout->addLayout(dashboardLayout);

    // =========================================================
    // 2. 控制按钮栏
    // =========================================================
    QHBoxLayout *toolLayout = new QHBoxLayout();
    m_btnRun = new QPushButton("开始仿真 (Run)", centralWidget);
    m_btnRun->setStyleSheet("font-weight: bold; font-size: 14px; height: 30px; background-color: #4CAF50; color: white;");
    connect(m_btnRun, &QPushButton::clicked, this, &MainWindow::onBtnRunClicked);
    toolLayout->addWidget(m_btnRun);

    m_btnToggleFlow = new QPushButton("流场数据导出", centralWidget);
    m_btnToggleFlow->setStyleSheet("font-weight: bold; font-size: 14px; height: 30px; background-color: #2196F3; color: white;");
    connect(m_btnToggleFlow, &QPushButton::clicked, this, &MainWindow::onBtnToggleFlowClicked);
    toolLayout->addWidget(m_btnToggleFlow);

    m_btnRecordFlow = new QPushButton("录制全流域流场", centralWidget);
    m_btnRecordFlow->setStyleSheet("font-weight: bold; font-size: 14px; height: 30px; background-color: #9C27B0; color: white;");
    connect(m_btnRecordFlow, &QPushButton::clicked, this, &MainWindow::onBtnRecordFlowClicked);
    toolLayout->addWidget(m_btnRecordFlow);

    // =========================================================
    // 【新增】 创建"蟹行验证录制"按钮
    // =========================================================
    m_isRecordingCSV = false; // 默认关闭

    m_btnRecordValidation = new QPushButton("录制验证数据 (Off)", centralWidget);
    // 默认灰色样式
    m_btnRecordValidation->setStyleSheet("font-weight: bold; font-size: 14px; height: 30px; background-color: #757575; color: white;");
    connect(m_btnRecordValidation, &QPushButton::clicked, this, &MainWindow::onBtnRecordValidationClicked);
    toolLayout->addWidget(m_btnRecordValidation);

    // 【新增】添加桥墩紊流专注模式复选框
    m_chkPylonMode = new QCheckBox("桥墩紊流专注模式", centralWidget);
    m_chkPylonMode->setToolTip("开启后仅录制桥区1km范围内50m高精数据，跳过全局录制");
    toolLayout->addWidget(m_chkPylonMode);

    toolLayout->addStretch(); // 弹簧，把按钮顶在左边

    mainLayout->addLayout(toolLayout);

    // 2. 初始化绘图窗口
    m_aisPlot = new QCustomPlot(centralWidget);
    m_aisPlot->xAxis->setLabel("经度 (Mercator X)");
    m_aisPlot->yAxis->setLabel("纬度 (Mercator Y)");
    mainLayout->addWidget(m_aisPlot, 1); // 海图占据主要空间

    this->setCentralWidget(centralWidget);

    // 初始化成员
    m_ownShip = Ship();
    m_currentRudderAngle = 0.0;

    // 初始化控制变量
    m_prevHeading_1 = 0.0;
    m_prevHeading_2 = 0.0;
    m_currentRudderRad = 0.0;
    m_intXTE = 0.0;
    // m_logCounter = 0; // 不再使用本地计数器，改用全局计数器

    m_trajectoryPredictor = new TrajectoryPredictor(this, this);
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::onTimerTick);
    m_timer->setInterval(30);

    // 初始化基准控制器
    m_baselineController = new BaselineController();

    // 动态创建基准模式开关
    m_cmbControlMode = new QComboBox(this);
    // =========================================================
    // 🌟 专为 SCI 论文定制的 4 组消融实验菜单
    // =========================================================
    m_cmbControlMode->addItem("0: Group 1 (基准组: 纯LOS + 传统带积分PID)");
    m_cmbControlMode->addItem("1: Group 2 (痛点组: 常规ILOS + 传统带积分PID)");
    m_cmbControlMode->addItem("2: Group 3 (前馈组: 纯LOS + 极简PD + 8单元FSI前馈)");
    m_cmbControlMode->addItem("3: Group 4 (本文完整算法: 强化ILOS + 极简PD + FSI前馈)");

    m_cmbControlMode->setStyleSheet("color: red; font-weight: bold; font-size: 14px; padding: 2px;");
    toolLayout->addWidget(m_cmbControlMode);

    initDatabase();

    // ============================================================
    // 【重点】只构建静态环境，不生成目标船，不启动定时器
    // ============================================================
    loadChannelData();
    loadDepthData(); // 加载水深数据
    drawWaterways();     // 绘制静态环境


    // 交互设置
    m_aisPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    m_aisPlot->setNotAntialiasedElements(QCP::aeNone);

    // 连接信号
    connect(m_aisPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(updateLabelsVisibility()));
    connect(m_aisPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), this, SLOT(updateLabelsVisibility()));

    // 【初始化本船位置】(静态展示用)
    // 先把本船放在起点，让用户能看到它，但它不动
    // 主尺度由 Ship 构造函数按 MMG 船型设定，这里不再覆盖
    m_ownShip.u = 0.0;
    m_ownShip.v = 0.0; // 初始静止
    m_ownShip.r = 0.0;

    // 动态获取上行航道作为静态展示起点（顺延10个点到深水区）
    if (!m_laneUpstream.isEmpty() && m_laneUpstream.size() > 10) {
        QPointF startPoint = m_laneUpstream[0];
        m_ownShip.X = startPoint.x();
        m_ownShip.Y = startPoint.y();

        // 计算初始航向：指向上游方向
        double dx = m_laneUpstream[1].x() - m_laneUpstream[0].x();
        double dy = m_laneUpstream[1].y() - m_laneUpstream[0].y();
        m_ownShip.C = atan2(dx, dy);
        qDebug() << "船舶静态展示位置已设置为上行航道起点:" << m_ownShip.X << m_ownShip.Y;
    } else {
        // 如果航道加载失败的防崩溃兜底
        m_ownShip.X = 0;
        m_ownShip.Y = 0;
        m_ownShip.C = 0;
        qDebug() << "警告: 上行航道未加载，使用默认位置";
    }

    m_ownShip.S = 12.0; // 初始航速 12 节

    // 设置初始视角（查看 3000m 范围）
    double viewRange = 3000.0;
    m_aisPlot->xAxis->setRange(m_ownShip.X - viewRange, m_ownShip.X + viewRange);
    m_aisPlot->yAxis->setRange(m_ownShip.Y - viewRange, m_ownShip.Y + viewRange);

    // 绘制一次静态画面
    drawOwnShip();

    // 注释掉这一行，防止软件一启动就读取旧数据库
    // m_bridgeEnv.setPlaybackMode(true);
}

MainWindow::~MainWindow()
{
    // 关闭数据库连接
    if (m_db.isOpen()) {
        m_db.close();
    }
    delete m_trajectoryPredictor;
    delete m_baselineController;
    delete ui;
}

void MainWindow::initDatabase()
{
    m_db = QSqlDatabase::addDatabase("QSQLITE");

    QString dbPath = "E:/Desk/department/shuziluansheng/shuziluansheng/xjp/daima/Yangtze_Mile235_270.db";

    m_db.setDatabaseName(dbPath);

    if (!m_db.open()) {
        return;
    }

}

// 辅助函数：从WKT字符串解析出点坐标
// 辅助函数：从WKT字符串解析出点坐标 (修复版)
// 把 WKT 的每个括号组解析成一个环：POLYGON ((外环...), (内环...)) 得到两个环，rings[0] 为外环。
// 原实现把括号统统替换成空格，多环几何的外环与内环被拼成一条自相交折线。长江岸陆是一个
// 86 km x 19 km 的整体多边形、河道即其内环，拼平后扣除水域掩码时把航道中段也当成陆地，
// computeAt 在该处返回 0 流速，蟹行角前馈瞬间失去输入，即全程最大横向偏差的成因。
QVector<QVector<QPointF>> MainWindow::parseWktRings(const QString &wkt) {
    QVector<QVector<QPointF>> rings;
    if (wkt.isEmpty()) return rings;

    int pos = 0;
    while (true) {
        const int open = wkt.indexOf('(', pos);
        if (open < 0) break;
        const int close = wkt.indexOf(')', open + 1);
        if (close < 0) break;

        // 只有不含嵌套左括号的括号组才是一个环，外层的 "((" 直接跳过
        const int nextOpen = wkt.indexOf('(', open + 1);
        if (nextOpen >= 0 && nextOpen < close) {
            pos = open + 1;
            continue;
        }

        const QString body = wkt.mid(open + 1, close - open - 1);
        QVector<QPointF> ring;
        const QStringList pairs = body.split(',', Qt::SkipEmptyParts);
        for (const QString &pair : pairs) {
            // WKT 的一对坐标是 "经度 纬度"，SkipEmptyParts 顺带吃掉多余空格
            const QStringList coords = pair.trimmed().split(' ', Qt::SkipEmptyParts);
            if (coords.size() < 2) continue;
            bool okLon = false, okLat = false;
            const double lon = coords[0].toDouble(&okLon);
            const double lat = coords[1].toDouble(&okLat);
            if (okLon && okLat) ring.append(this->mercatorPoint(lon, lat));
        }
        if (ring.size() > 1) rings.append(ring);
        pos = close + 1;
    }
    return rings;
}

// 兼容原接口：只返回外环。线状要素本来只有一个环，行为不变。
QVector<QPointF> MainWindow::parseWktPoints(const QString &wkt) {
    const QVector<QVector<QPointF>> rings = parseWktRings(wkt);
    return rings.isEmpty() ? QVector<QPointF>() : rings.first();
}

// 辅助函数：解析WKT点字符串，返回经纬度
bool MainWindow::parseWktPoint(const QString &wkt, double &lon, double &lat) {
    QString cleaned = wkt.trimmed();

    // 1. 暴力移除 "POINT" 前缀 (忽略大小写)
    // 这样 "POINT (121...)" 变成 " (121...)"
    cleaned.remove("POINT", Qt::CaseInsensitive);

    // 2. 暴力移除所有括号 '(' 和 ')'
    // 这样 " (121...)" 变成 "  121... "
    cleaned = cleaned.remove('(').remove(')');

    // 3. 去头尾空格，变成 "121.38 31.61"
    cleaned = cleaned.trimmed();

    // 4. 分割经纬度
    QStringList parts = cleaned.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
    if (parts.size() >= 2) {
        bool ok1, ok2;
        lon = parts[0].toDouble(&ok1);
        lat = parts[1].toDouble(&ok2);

        // 只要两个数都能转成功，就认为解析成功
        if (ok1 && ok2) {
            return true;
        }
    }

    return false;
}

// 【新增函数】提取多边形的中心线
// 算法逻辑：假设多边形是一个长条形，找到距离最远的两对端点，将长边视为左右边界
QVector<QPointF> MainWindow::calculateCenterLine(const QVector<QPointF>& polyPoints)
{
    QVector<QPointF> centerLine;
    if (polyPoints.size() < 4) return centerLine;

    // 这里使用一个针对航道的简化逻辑：
    // 假设多边形点的顺序是顺时针或逆时针连续的
    // 我们取前半段点集和后半段点集（倒序）的平均值

    int n = polyPoints.size();
    int half = n / 2;

    // 双指针法，从“头”和“尾”向中间逼近
    // 适用于 S57 这种通常首尾相接定义的带状多边形
    for (int i = 0; i < half; ++i) {
        QPointF p1 = polyPoints[i];
        QPointF p2 = polyPoints[n - 1 - i]; // 对面的点

        // 计算中点
        double mx = (p1.x() + p2.x()) / 2.0;
        double my = (p1.y() + p2.y()) / 2.0;

        centerLine.append(QPointF(mx, my));
    }

    return centerLine;
}

// 【新增函数】计算点到线段的距离
double MainWindow::pointToLineDistance(const QPointF& point, const QPointF& lineStart, const QPointF& lineEnd)
{
    // 计算线段向量
    double dx = lineEnd.x() - lineStart.x();
    double dy = lineEnd.y() - lineStart.y();

    // 计算点到线段起点的向量
    double px = point.x() - lineStart.x();
    double py = point.y() - lineStart.y();

    // 计算线段长度的平方
    double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared < 1e-9) {
        // 线段长度接近0，返回点到起点的距离
        return sqrt(px * px + py * py);
    }

    // 计算投影参数t
    double t = (px * dx + py * dy) / lengthSquared;

    // 将t限制在[0, 1]范围内
    t = qBound(0.0, t, 1.0);

    // 计算投影点
    double projX = lineStart.x() + t * dx;
    double projY = lineStart.y() + t * dy;

    // 计算点到投影点的距离
    double distX = point.x() - projX;
    double distY = point.y() - projY;

    return sqrt(distX * distX + distY * distY);
}

// 辅助函数：计算带符号的偏航距离 (Cross Track Error)
// 修复后的 calculateSignedDistance - 已反转XTE符号以符合物理事实
double MainWindow::calculateSignedDistance(QPointF p, QPointF p1, QPointF p2) {
    double dx_line = p2.x() - p1.x();
    double dy_line = p2.y() - p1.y();
    double dx_point = p.x() - p1.x();
    double dy_point = p.y() - p1.y();

    // 二维叉乘: A_x*B_y - A_y*B_x
    // 数学坐标系中:
    // > 0 表示点在向量左侧
    // < 0 表示点在向量右侧
    double cross_product = dx_line * dy_point - dy_line * dx_point;

    double line_len = sqrt(dx_line * dx_line + dy_line * dy_line);
    double distance = 0.0;
    if (line_len > 1e-6) {
        distance = fabs(cross_product) / line_len; // 先取绝对值距离
    }

    // 【核心修复】反转XTE符号，确保符合物理事实
    // LOS 公式是 atan(-error / Delta)
    // 现在的逻辑：
    // 1. 如果 cross_product > 0 (物理上在左侧) -> 返回负值
    //    这样 LOS 公式 atan(-XTE/Delta) 会得到正的修正角 (右转)
    // 2. 如果 cross_product <= 0 (物理上在右侧) -> 返回正值
    //    这样 LOS 公式 atan(-XTE/Delta) 会得到负的修正角 (左转)

    if (cross_product > 0) {
        // 物理上在左侧 -> 返回负值
        return -distance;
    } else {
        // 物理上在右侧 -> 返回正值
        return distance;
    }
}

// 注：Buoy结构体已在types.h中定义

void MainWindow::loadChannelData()
{
    // 1. 清空所有容器
    m_pylonsPolygons.clear();
    m_dredgedPolygons.clear();
    m_landPolygons.clear();
    m_bridgePolygons.clear();
    m_depthPolygons.clear();

    // 【新增】清空新容器
    m_anchoragePolygons.clear();
    m_obstructionPolygons.clear();
    m_wreckPolygons.clear();

    // 清空连线数据
    m_virtualRedBoundary.clear();
    m_virtualGreenBoundary.clear();

    // 清空散点数据
    m_allBuoys.clear();

    // 初始化左右岸浮标容器
    QVector<Buoy> leftBankBuoys;
    QVector<Buoy> rightBankBuoys;

    // 【新增】初始化潮流观测点容器
    QVector<FEBPoint> finalFebPoints;

    QSqlQuery query(m_db);

    // =========================================================
    // 1. 加载 DEPARE (水深区) - 读取 DRVAL1 用于分层设色
    // =========================================================
    // 假设数据库中有 DRVAL1 字段 (S-57标准字段)
    QString sqlDep = "SELECT RCID, DRVAL1, geometry_wkt FROM DEPARE WHERE geometry_type = 'POLYGON'";

    // 容错：如果表中没有 DRVAL1，降级为只查 geometry
    bool hasDepth = true;
    if (!query.exec(sqlDep)) {
        sqlDep = "SELECT RCID, 0, geometry_wkt FROM DEPARE WHERE geometry_type = 'POLYGON'";
        hasDepth = false;
        if (!query.exec(sqlDep)) {
        }
    }

    while (query.next()) {
        ChannelPoly poly;
        poly.id = query.value(0).toInt();

        // 读取水深 (如果数据库里是字符串类型，用 toDouble；如果是数字，直接转)
        if (hasDepth) {
            poly.depth = query.value(1).toDouble();
        } else {
            poly.depth = 5.0; // 默认给个中间值
        }

        QString wkt = query.value(2).toString();
        poly.rings = parseWktRings(wkt);
        poly.points = poly.rings.isEmpty() ? QVector<QPointF>() : poly.rings.first();

        if (!poly.points.isEmpty()) m_depthPolygons.append(poly);
    }


    // =========================================================
    // 2. 加载 LNDARE (陆地)
    // =========================================================
    QString sqlLand = "SELECT RCID, geometry_wkt, OBJNAM FROM LNDARE WHERE geometry_type = 'POLYGON'";
    if (query.exec(sqlLand)) {
        while (query.next()) {
            ChannelPoly poly;
            poly.id = query.value(0).toInt();
            poly.rings = parseWktRings(query.value(1).toString());
            poly.points = poly.rings.isEmpty() ? QVector<QPointF>() : poly.rings.first();
            QString objnam = query.value(2).toString();

            if (!poly.points.isEmpty()) {
                m_landPolygons.append(poly);

                // 检查是否是沙洲 (OBJNAM包含"Sha")
                if (objnam.contains("Sha", Qt::CaseInsensitive)) {
                    m_sandBars.append(poly);
                }
            }
        }
    }

    // =========================================================
    // 【新增】加载 COALNE (岸线) - 用于流场边界条件
    // =========================================================
    QString sqlCoalne = "SELECT RCID, geometry_wkt FROM COALNE";
    if (query.exec(sqlCoalne)) {
        while (query.next()) {
            QString wkt = query.value(1).toString();
            QPolygonF poly = parseWktPoints(wkt);

            if (!poly.isEmpty()) {
                m_shorelinePolygons.append(poly);
            }
        }
        }

    // =========================================================
    // 读取所有潮流观测站，并根据CAT_TS分类
    // =========================================================
    QString sqlFeb = "SELECT id, CAT_TS, CURVEL, ORIENT, geometry_wkt FROM TS_FEB";
    QVector<FEBPoint> febPoints;

    if (query.exec(sqlFeb)) {
        int count = 0;
        while (query.next()) {
            int id = query.value(0).toInt();
            int catTs = query.value(1).toInt();

            double speedRaw = query.value(2).toDouble();
            double speed = speedRaw / 100.0;

            double orient = query.value(3).toDouble();
            QString wkt = query.value(4).toString();

            QString stationType = (catTs == 1) ? "涨潮站" : (catTs == 2) ? "落潮站" : "未分类站";
            if (count < 5) {
            }

            double lon, lat;
            if (parseWktPoint(wkt, lon, lat)) {
                QPointF pos = mercatorPoint(lon, lat);

                FEBPoint newPt;
                newPt.id = id;
                newPt.x = pos.x();
                newPt.y = pos.y();
                newPt.floodSpeed = speed;
                newPt.floodDir = orient;
                newPt.ebbSpeed = speed;
                newPt.ebbDir = orient;
                newPt.type = catTs;  // 保存CAT_TS值

                febPoints.append(newPt);
                count++;
            }
        }

        finalFebPoints = febPoints;
        m_febPoints = finalFebPoints;
    } else {
    }

    // =========================================================
    // 【新增】加载 ACHARE (锚地)
    // =========================================================
    QString sqlAchare = "SELECT RCID, geometry_wkt FROM ACHARE";
    if (query.exec(sqlAchare)) {
        while (query.next()) {
            ChannelPoly poly;
            poly.id = query.value(0).toInt();
            poly.points = parseWktPoints(query.value(1).toString());
            // 如果解析出点，存入
            if (!poly.points.isEmpty()) m_anchoragePolygons.append(poly);
        }
    }
    // =========================================================
    // 【新增】加载 OBSTRN (障碍物) & WRECKS (沉船)
    // =========================================================
    // 障碍物和沉船可能是点(POINT)也可能是面(POLYGON)
    // 我们的 parseWktPoints 能处理这两种情况

    // 加载障碍物
    QString sqlObs = "SELECT RCID, geometry_wkt FROM OBSTRN";
    if (query.exec(sqlObs)) {
        while (query.next()) {
            ChannelPoly poly;
            poly.id = query.value(0).toInt();
            poly.points = parseWktPoints(query.value(1).toString());
            if (!poly.points.isEmpty()) m_obstructionPolygons.append(poly);
        }
    }
    // 加载沉船
    QString sqlWrecks = "SELECT RCID, geometry_wkt FROM WRECKS";
    if (query.exec(sqlWrecks)) {
        while (query.next()) {
            ChannelPoly poly;
            poly.id = query.value(0).toInt();
            poly.points = parseWktPoints(query.value(1).toString());
            if (!poly.points.isEmpty()) m_wreckPolygons.append(poly);
        }
    }


    QString sqlDrg = "SELECT RCID, geometry_wkt FROM TSSLPT WHERE geometry_type = 'POLYGON'";
    if (query.exec(sqlDrg)) {
        while (query.next()) {
            ChannelPoly poly;
            poly.id = query.value(0).toInt();
            poly.points = parseWktPoints(query.value(1).toString());

            // 计算中心线 (如果需要航迹保持功能)
            if (!poly.points.isEmpty()) {
                poly.centerLine = calculateCenterLine(poly.points);
                m_dredgedPolygons.append(poly);
            }
        }
    }

    // =========================================================
    // 4. 加载 BRIDGE (桥梁主体)
    // =========================================================
    // 添加objnam字段以显示桥梁名称
    QString sqlBridge = "SELECT RCID, geometry_wkt, objnam FROM BRIDGE WHERE geometry_type = 'POLYGON'";
    if (query.exec(sqlBridge)) {
        while (query.next()) {
            ChannelPoly poly;
            poly.id = query.value(0).toInt();
            poly.points = parseWktPoints(query.value(1).toString());
            poly.name = query.value(2).toString();
            if (!poly.points.isEmpty()) m_bridgePolygons.append(poly);
        }
    }

    // =========================================================
    // 5. PYLONS (桥墩/塔架) - 从数据库加载
    // =========================================================
    // 桥墩数据现在从数据库动态加载，不再硬编码

    // =========================================================
    // 【终极航道生成】111-132区间卡点 + 按编号平滑插值连线
    // =========================================================

    // 1. 查询数据
    QString sqlBuoys = "SELECT CATLAM, COLOUR, OBJNAM, geometry_wkt FROM BOYLAT";
    if (!query.exec(sqlBuoys)) return;

    // 同时捕获主编号与子编号："(113-1)" -> 113 与 1。只取主编号会让 113 与 113-1 无法区分
    QRegularExpression rxName("(\\d+)(?:-(\\d+))?");

    int totalCount = 0, validCount = 0;

    while (query.next()) {
        totalCount++;
        int catlam = query.value(0).toInt();
        QString colorStr = query.value(1).toString();
        QString name = query.value(2).toString().trimmed();
        QString wkt = query.value(3).toString();

        double lon, lat;
        if (parseWktPoint(wkt, lon, lat)) {
            double x, y;
            lonLat2Mercator(lon, lat, x, y);

            // =================================================
            // 【提取编号与范围过滤】
            // =================================================
            QRegularExpressionMatch match = rxName.match(name);
            if (match.hasMatch()) {
                int buoyNum = match.captured(1).toInt();
                int buoySub = match.captured(2).toInt(); // 无子编号时为 0

                if (buoyNum < 111 || buoyNum > 132) continue;

                // =================================================
                // 【颜色解析与不对称截断】 (3=红, 4=绿, 2=黑转绿)
                // =================================================
                colorStr = colorStr.remove('[').remove(']').remove('\'').remove('"');
                int colorCode = colorStr.split(',').first().toInt();

                if (colorCode == 2) colorCode = 4;
                if (colorCode != 3 && colorCode != 4) continue;

                // 左侧红浮(3)只到131，右侧黑/绿浮(4)到132
                if (colorCode == 3 && buoyNum > 131) continue;
                if (colorCode == 4 && buoyNum > 132) continue;

                // --- 保存数据 ---
                Buoy b;
                b.pos = QPointF(x, y);
                b.catlam = catlam;
                b.name = name;
                b.number = buoyNum;
                b.subNumber = buoySub;
                b.colorCode = colorCode;

                m_allBuoys.append(b);
                validCount++;

                // 【恢复分组】将干净的数据分别放入左右岸容器
                if (b.colorCode == 3) {
                    leftBankBuoys.append(b);
                } else if (b.colorCode == 4) {
                    rightBankBuoys.append(b);
                }
            }
        }
    }

    // 【硬编码补全】手动添加 125-130 之间缺失的红浮 (左岸/南岸)
    // 这些浮标在S-57数据中缺失，需要手动补全以形成完整的航道边界
    // 坐标从用户提供的海图截图中精确读取
    QVector<QPair<QString, QPointF>> missingRedBuoys = {
        {"125-1", QPointF(119.076567, 32.233933)},  // 32°14.036'N, 119°4.594'E
        {"128", QPointF(119.058100, 32.210117)},  // 32°12.607'N, 119°3.486'E
        {"129", QPointF(119.049617, 32.201850)},  // 32°12.111'N, 119°2.977'E
        {"129-1", QPointF(119.032917, 32.189133)},// 32°11.348'N, 119°1.975'E
        {"132", QPointF(118.9876, 32.1718)}// 32°11.348'N, 119°1.975'E
    };

    for (const auto& pair : missingRedBuoys) {
        double x, y;
        lonLat2Mercator(pair.second.x(), pair.second.y(), x, y);

        Buoy b;
        b.pos = QPointF(x, y);
        b.name = pair.first;
        const QStringList nameParts = b.name.split('-');
        b.number = nameParts.first().toInt();
        b.subNumber = nameParts.size() > 1 ? nameParts.at(1).toInt() : 0;
        b.colorCode = 3; // 红色

        m_allBuoys.append(b);
        leftBankBuoys.append(b); // 压入左岸容器参与连线计算
    }

    // =========================================================
    // 【核心修复：按编号排序，无视尹公洲U型大弯道的地理错乱】
    // =========================================================
    // 主编号相同时（如 113 与 113-1）必须再比子编号：只比主编号时两者顺序不确定，
    // 会让插值出的边界线在该处折返，形成半径只有几十米的假弯道
    auto sortBuoysByNum = [](const Buoy& a, const Buoy& b) {
        if (a.number != b.number) return a.number < b.number;
        if (a.subNumber != b.subNumber) return a.subNumber < b.subNumber;
        return a.pos.x() > b.pos.x(); // 编号完全相同时按上行方向（经度递减）排列
    };
    std::sort(leftBankBuoys.begin(), leftBankBuoys.end(), sortBuoysByNum);
    std::sort(rightBankBuoys.begin(), rightBankBuoys.end(), sortBuoysByNum);

    // =========================================================
    // 【生成高精度航道边界与中心线】
    // =========================================================

    // 1. 生成平滑边界 (Catmull-Rom 插值，每两个浮标间插值20个点)
    m_virtualRedBoundary = generateSmoothCurve(leftBankBuoys, 20);
    m_virtualGreenBoundary = generateSmoothCurve(rightBankBuoys, 20);

    // 2. 清空旧航线
    m_virtualCenterLine.clear();
    m_laneUpstream.clear();
    m_laneDownstream.clear();

    // 3. 重采样并求取中点，生成绝对居中的航迹带
    if (!m_virtualRedBoundary.isEmpty() && !m_virtualGreenBoundary.isEmpty()) {
        int sampleCount = 1000;
        QVector<QPointF> resampledRed = resamplePolyline(m_virtualRedBoundary, sampleCount);
        QVector<QPointF> resampledGreen = resamplePolyline(m_virtualGreenBoundary, sampleCount);

        for (int i = 0; i < sampleCount; ++i) {
            double rx = resampledRed[i].x(); double ry = resampledRed[i].y();
            double gx = resampledGreen[i].x(); double gy = resampledGreen[i].y();

            // 计算中心线 (分隔带)
            double cx = (rx + gx) / 2.0;
            double cy = (ry + gy) / 2.0;
            m_virtualCenterLine.append(QPointF(cx, cy));

            // 计算上行航线 (偏向右岸/北岸)
            double upX = (cx + gx) / 2.0;
            double upY = (cy + gy) / 2.0;
            m_laneUpstream.append(QPointF(upX, upY));

            // 计算下行航线 (偏向左岸/南岸)
            double downX = (cx + rx) / 2.0;
            double downY = (cy + ry) / 2.0;
            m_laneDownstream.append(QPointF(downX, downY));
        }

        // 翻转数组：让航线的点序符合船舶顺流行驶的方向（从西向东，或者视你的需求而定）
        std::reverse(m_virtualCenterLine.begin(), m_virtualCenterLine.end());

        // ❌ 上行航道保持原生的"由东向西(逆流)"顺序，不翻转！
        // std::reverse(m_laneUpstream.begin(), m_laneUpstream.end());

        std::reverse(m_laneDownstream.begin(), m_laneDownstream.end());

        // 将计算好的优质路径传给底层基准控制器，准备进行AILOS跟踪
        // 【注意】主测上行，所以传给上行航道！
        m_baselineController->setPath(m_laneUpstream);

        // -----------------------------------------------------------------
        // 【新增】将生成的上行航道点 (m_laneUpstream) 导出为 CSV 文件
        // -----------------------------------------------------------------
        QString laneCsvPath = QCoreApplication::applicationDirPath() + "/UpstreamLane.csv";
        QFile laneCsvFile(laneCsvPath);
        if (laneCsvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            QTextStream out(&laneCsvFile);
            out << "lon,lat,x_mercator,y_mercator\n"; // 表头

            for (const QPointF& pt : m_laneUpstream) {
                double lon, lat;
                // m_laneUpstream 存的是墨卡托坐标，将其转回经纬度以便与 ShipLog 对齐
                Mercator2lonLat(pt.x(), pt.y(), lon, lat);

                out << QString::number(lon, 'f', 6) << ","
                    << QString::number(lat, 'f', 6) << ","
                    << QString::number(pt.x(), 'f', 2) << ","
                    << QString::number(pt.y(), 'f', 2) << "\n";
            }
            laneCsvFile.close();
            qDebug() << "✅ 上行航道基准点已成功导出到: " << laneCsvPath;
        }
        // -----------------------------------------------------------------
    }

    // 【新增】加载水深线数据
    loadDepthData();

    // 【新增】加载SOUNDG数据（水深散点）
    m_soundings.clear();
    QSqlQuery soundQuery("SELECT geometry_wkt, DEPTH FROM SOUNDG");
    while (soundQuery.next()) {
        double lon, lat;
        if (parseWktPoint(soundQuery.value(0).toString(), lon, lat)) {
            QPointF p = mercatorPoint(lon, lat);
            m_soundings.append(SoundingPoint{p.x(), p.y(), soundQuery.value(1).toDouble()});
        }
    }

    // 【新增】初始化全流域流场参数（合并初始化，避免重复计算）
    qDebug() << "=== 准备调用 initGlobalFlow ===";
    qDebug() << "岸线数量:" << m_shorelinePolygons.size();
    qDebug() << "陆地多边形数量:" << m_landPolygons.size();
    qDebug() << "水深点数量:" << m_soundings.size();
    qDebug() << "水深多边形数量:" << m_depthPolygons.size();

    if (!m_shorelinePolygons.isEmpty() && !m_landPolygons.isEmpty()) {
        qDebug() << "--- 开始调用 initGlobalFlow ---";
        m_bridgeEnv.initGlobalFlow(m_shorelinePolygons, m_landPolygons, m_soundings, m_depthPolygons);
        qDebug() << "--- initGlobalFlow 调用完成 ---";
    }

    qDebug() << "=== initGlobalFlow 阶段完成 ===";

    // 【新增】动态获取所有观测站的基准水深
    for (int i = 0; i < finalFebPoints.size(); ++i) {
        double stationH = m_bridgeEnv.getDepthAt(finalFebPoints[i].x, finalFebPoints[i].y);
        finalFebPoints[i].refDepth = (stationH > 1.0) ? stationH : 25.0;
    }

    // 【新增】设置潮汐动力源观测点
    if (!finalFebPoints.isEmpty()) {
        m_bridgeEnv.setTidalSources(finalFebPoints);
    }

    // 【新增】同步桥墩位置到环境场
    m_bridgeEnv.setPylons(m_pylonsPolygons);

    // =========================================================
    // 【条件配置】只有在使用苏通大桥数据库时才启用体育场模型
    // 密西西比河数据库使用虚拟流场锚点，不需要体育场配置
    // =========================================================
    // 设置流场参数（通用配置，不依赖特定水域）
    // =========================================================
    // 默认不使用体育场模型，桥墩数据从数据库加载
    QVector<FlowField::IdealizedPylon> emptyPylons;
    m_bridgeEnv.setIdealizedPylons(emptyPylons);

    // 将筛选出的沙洲数据传递给流场模块
    m_bridgeEnv.setSandBars(m_sandBars);

    // =========================================================
    // 【数据库驱动流场】从 VIRTUAL_FLOW_ANCHORS 表加载场源
    // =========================================================
    m_bridgeEnv.loadAnchorsFromDb(m_db);

    // 🌟🌟🌟 新增连招 1：用顺流的中心线校准所有锚点的流向，专治"长江倒流"！
    if (!m_virtualCenterLine.isEmpty()) {
        m_bridgeEnv.autoAlignAnchorDirections(m_virtualCenterLine);
    }

    // 🌟🌟🌟 新增连招 2：背景兜底流也顺便改为向东 (90度)
    qDebug() << "探针 2: 准备调用 setBackgroundFlow";
    m_bridgeEnv.setBackgroundFlow(2.0, 90.0);
    qDebug() << "探针 3: setBackgroundFlow 完成";

    // 🌟🌟🌟 新增连招 3：流向和参数全部齐备后，一锤定音解算静态全局 RBF 矩阵！
    qDebug() << "探针 4: 正在求解全局 RBF 稳态流场权重矩阵...";
    m_bridgeEnv.solveRBFWeights(0.0); // 传入 0 即可，因为已经是静态场

    qDebug() << "✅ 全流域高精度稳态流场初始化完毕！";
}

void MainWindow::drawWaterways()
{
    qDebug() << "探针 4: 进入 drawWaterways";
    qDebug() << "  - m_aisPlot:" << m_aisPlot;

    if (!m_aisPlot) {
        qDebug() << "  ! m_aisPlot 为空！";
        return;
    }

    qDebug() << "  - m_aisPlot->xAxis:" << m_aisPlot->xAxis;

    // =========================================================
    // 【修复】阻塞信号，防止在清理过程中触发 updateLabelsVisibility 导致野指针访问
    // =========================================================
    m_aisPlot->xAxis->blockSignals(true);
    qDebug() << "探针 4a: xAxis->blockSignals 完成";
    m_aisPlot->yAxis->blockSignals(true);
    qDebug() << "探针 4b: yAxis->blockSignals 完成";

    // 清除旧图层
    m_aisPlot->clearPlottables();
    qDebug() << "探针 4c: clearPlottables 完成";

    // 【新增】清空旧的浮标标签，防止内存泄漏和重影
    // QCustomPlot::removeItem 会同时释放对象，不能再 delete
    for (auto label : m_buoyLabels) {
        m_aisPlot->removeItem(label);
    }
    m_buoyLabels.clear();
    qDebug() << "探针 4d: 浮标标签清理完成";

    // 【新增】清空旧的观测站标签
    qDebug() << "探针 4e: 准备清理观测站标签";
    for (auto label : m_febLabels) {
        m_aisPlot->removeItem(label);
    }
    m_febLabels.clear();
    qDebug() << "探针 4f: 观测站标签清理完成";

    // 【新增】清理水深数值标签
    qDebug() << "探针 4g: 准备清理水深标签";
    for (auto label : m_depthLabels) {
        m_aisPlot->removeItem(label);
    }
    m_depthLabels.clear();
    qDebug() << "探针 4h: 水深标签清理完成";

    // 如果没有任何数据，直接返回
    qDebug() << "探针 4i: 检查数据是否为空";
    if (m_landPolygons.isEmpty() && m_depthPolygons.isEmpty() && m_dredgedPolygons.isEmpty() && m_bridgePolygons.isEmpty() && m_pylonsPolygons.isEmpty()) {
        qDebug() << "探针 4j: 数据为空，直接返回";
        return;
    }

    // =========================================================
    // 【核心改进】按 S-57 标准的 GRUP 顺序绘制 (从小到大)
    // =========================================================

    // 调用背景层绘制函数
    qDebug() << "探针 4k: 准备调用 drawBackgroundLayers";
    drawBackgroundLayers();
    qDebug() << "探针 4l: drawBackgroundLayers 完成";

    // =========================================================
    // 【新增】绘制虚拟航道 (根据浮标拟合)
    // =========================================================
    if (!m_virtualRedBoundary.isEmpty() && !m_virtualGreenBoundary.isEmpty()) {

        // --- A. 绘制航道填充区域 (可选) ---
        // 将红线和反向的绿线拼接，形成一个闭合多边形
        QVector<double> polyX, polyY;

        // 加入红线点
        for(const auto& p : m_virtualRedBoundary) { polyX.append(p.x()); polyY.append(p.y()); }

        // 加入绿线点 (倒序，以便首尾相接)
        for(int i = m_virtualGreenBoundary.size()-1; i >= 0; --i) {
            polyX.append(m_virtualGreenBoundary[i].x());
            polyY.append(m_virtualGreenBoundary[i].y());
        }
        // 闭合
        if(!m_virtualRedBoundary.isEmpty()) {
            polyX.append(m_virtualRedBoundary.first().x());
            polyY.append(m_virtualRedBoundary.first().y());
        }

        QCPCurve *channelFill = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        channelFill->setData(polyX, polyY);
        channelFill->setPen(Qt::NoPen);
        // 稍微加深一点填充色，增加质感
        channelFill->setBrush(QColor(138, 43, 226, 40)); // 蓝紫色半透明
        channelFill->setSelectable(QCP::stNone);

        // --- B. 绘制边界线 ---

        // 1. 红色边界 (左岸)
        QCPCurve *redCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> rx, ry;
        for(const auto& p : m_virtualRedBoundary) { rx.append(p.x()); ry.append(p.y()); }
        redCurve->setData(rx, ry);

        // 【修改】加粗至 4.0，使用实线或长虚线使其更明显
        QPen redPen(Qt::red);
        redPen.setWidthF(4.0);
        redPen.setStyle(Qt::DashLine); // 或者 Qt::SolidLine 实线
        redCurve->setPen(redPen);
        redCurve->setBrush(Qt::NoBrush);

        // 2. 绿色边界 (右岸)
        QCPCurve *greenCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> gx, gy;
        for(const auto& p : m_virtualGreenBoundary) { gx.append(p.x()); gy.append(p.y()); }
        greenCurve->setData(gx, gy);

        // 【修改】加粗至 4.0
        QPen greenPen(Qt::darkGreen); // 使用深绿，在亮背景下更清楚
        greenPen.setWidthF(4.0);
        greenPen.setStyle(Qt::DashLine);
        greenCurve->setPen(greenPen);
        greenCurve->setBrush(Qt::NoBrush);

        // =========================================================
        // 【新增】绘制三条航线
        // =========================================================

        // 3. 中心线 (分隔带) - 中等粗细 (2.5)
        if (!m_virtualCenterLine.isEmpty()) {
            QCPCurve *centerCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
            QVector<double> x, y;
            for (const auto &pt : m_virtualCenterLine) { x.append(pt.x()); y.append(pt.y()); }
            centerCurve->setData(x, y);

            QPen centerPen(Qt::yellow);
            centerPen.setWidthF(2.5); // 介于 1.5 和 4.0 之间
            centerPen.setStyle(Qt::DashLine);
            centerCurve->setPen(centerPen);
            centerCurve->setName("Separation Line");
        }

        // 4. 上行航线 - 较细 (1.5)
        if (!m_laneUpstream.isEmpty()) {
            QCPCurve *upCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
            QVector<double> x, y;
            for (const auto &pt : m_laneUpstream) { x.append(pt.x()); y.append(pt.y()); }
            upCurve->setData(x, y);
            upCurve->setPen(QPen(Qt::cyan, 1.5, Qt::DashDotLine));
            upCurve->setName("Upstream Lane");
        }

        // 5. 下行航线 - 较细 (1.5)
        if (!m_laneDownstream.isEmpty()) {
            QCPCurve *downCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
            QVector<double> x, y;
            for (const auto &pt : m_laneDownstream) { x.append(pt.x()); y.append(pt.y()); }
            downCurve->setData(x, y);
            downCurve->setPen(QPen(QColor(255, 165, 0), 1.5, Qt::DashDotLine));
            downCurve->setName("Downstream Lane");
        }
    }

    // =========================================================
    // 【新增】绘制浮标实体 (Scatter Points)
    //    这里绘制真实的浮标位置，覆盖在线条之上
    // =========================================================
    if (!m_allBuoys.isEmpty()) {
        // 创建 Graph 绘制红绿浮标点
        QCPGraph *redBuoys = m_aisPlot->addGraph();
        redBuoys->setLineStyle(QCPGraph::lsNone);
        // 加大一点浮标尺寸 (9 -> 10)
        redBuoys->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare, QPen(Qt::black, 0.5), QBrush(Qt::red), 10));

        QCPGraph *greenBuoys = m_aisPlot->addGraph();
        greenBuoys->setLineStyle(QCPGraph::lsNone);
        greenBuoys->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssTriangle, QPen(Qt::black, 0.5), QBrush(QColor(0, 200, 0)), 11));

        QVector<double> rx, ry, gx, gy;

        for (const auto& b : m_allBuoys) {
            // 根据颜色代码分配图层
            // 3 = Red, 4 = Green
            // 左右通航浮 (如 3,4,3) split后是 3，也会画成红色，符合主色调
            if (b.colorCode == 3) {
                rx.append(b.pos.x());
                ry.append(b.pos.y());
            } else if (b.colorCode == 4) {
                gx.append(b.pos.x());
                gy.append(b.pos.y());
            }

            // 【修改】添加浮标编号标签并存入列表
            QCPItemText *label = new QCPItemText(m_aisPlot);
            label->position->setCoords(b.pos.x(), b.pos.y());
            label->setText(QString::number(b.number));

            // 样式优化：无底色，小字体
            label->setFont(QFont("Arial", 9)); // 字号固定
            label->setBrush(Qt::NoBrush);      // 透明背景
            label->setPen(Qt::NoPen);          // 无边框

            // 颜色逻辑
            if (b.colorCode == 3) { // 左/红
                label->setPositionAlignment(Qt::AlignRight | Qt::AlignVCenter);
                label->setPadding(QMargins(0, 0, 5, 0));
                label->setColor(Qt::darkRed); // 深红，比正红更易读
            } else { // 右/绿
                label->setPositionAlignment(Qt::AlignLeft | Qt::AlignVCenter);
                label->setPadding(QMargins(5, 0, 0, 0));
                label->setColor(Qt::darkGreen); // 深绿
            }

            // 性能优化：设为可裁剪，移出屏幕就不画
            label->setClipToAxisRect(true);

            // 将标签加入列表管理
            m_buoyLabels.append(label);
        }

        redBuoys->setData(rx, ry);
        greenBuoys->setData(gx, gy);
    }

    // =========================================================
    // 【新增】绘制潮流观测站 (TS_FEB) - 可视化区分涨/落潮
    // =========================================================
    if (!m_febPoints.isEmpty()) {
        // 1. 准备分类数据容器
        QVector<double> floodX, floodY; // 涨潮站 (CAT_TS=1)
        QVector<double> ebbX, ebbY;     // 落潮站 (CAT_TS=2)
        QVector<double> otherX, otherY; // 未分类

        // 2. 遍历分类
        for (const auto& feb : m_febPoints) {
            if (feb.type == 1) {
                floodX.append(feb.x);
                floodY.append(feb.y);
            } else if (feb.type == 2) {
                ebbX.append(feb.x);
                ebbY.append(feb.y);
            } else {
                otherX.append(feb.x);
                otherY.append(feb.y);
            }
        }

        // 3. 绘制涨潮站 (Flood) - 红色向上三角 ▲
        if (!floodX.isEmpty()) {
            QCPGraph *floodGraph = m_aisPlot->addGraph();
            floodGraph->setLineStyle(QCPGraph::lsNone);
            // 样式：红色填充，黑色边框，向上三角
            floodGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssTriangle,
                                        QPen(Qt::black, 0.5), QBrush(QColor(255, 0, 0, 200)), 14));
            floodGraph->setData(floodX, floodY);
            floodGraph->setName("Flood Stations (涨潮)");
        }

        // 4. 绘制落潮站 (Ebb) - 绿色倒三角 ▼
        if (!ebbX.isEmpty()) {
            QCPGraph *ebbGraph = m_aisPlot->addGraph();
            ebbGraph->setLineStyle(QCPGraph::lsNone);
            // 样式：绿色填充，黑色边框，倒三角
            ebbGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssTriangleInverted,
                                        QPen(Qt::black, 0.5), QBrush(QColor(0, 200, 0, 200)), 14));
            ebbGraph->setData(ebbX, ebbY);
            ebbGraph->setName("Ebb Stations (落潮)");
        }

        // 5. 绘制其他站点 - 灰色圆形 ●
        if (!otherX.isEmpty()) {
            QCPGraph *otherGraph = m_aisPlot->addGraph();
            otherGraph->setLineStyle(QCPGraph::lsNone);
            otherGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle,
                                        QPen(Qt::black, 0.5), QBrush(Qt::gray), 10));
            otherGraph->setData(otherX, otherY);
        }

        // 6. 添加增强版文字标签 (ID + 类型)
        for (const auto& feb : m_febPoints) {
            QCPItemText *label = new QCPItemText(m_aisPlot);
            label->position->setCoords(feb.x, feb.y);

            QString typeStr;
            QColor txtColor;

            // 根据类型设置文本内容和颜色
            if (feb.type == 1) {
                typeStr = "涨潮";
                txtColor = Qt::red;
                // 涨潮标签放在图标上方
                label->setPositionAlignment(Qt::AlignBottom | Qt::AlignHCenter);
                label->setPadding(QMargins(0, 8, 0, 0));
            } else if (feb.type == 2) {
                typeStr = "落潮";
                txtColor = Qt::darkGreen;
                // 落潮标签放在图标下方
                label->setPositionAlignment(Qt::AlignTop | Qt::AlignHCenter);
                label->setPadding(QMargins(0, 0, 0, 8));
            } else {
                typeStr = "";
                txtColor = Qt::black;
                label->setPositionAlignment(Qt::AlignBottom | Qt::AlignHCenter);
                label->setPadding(QMargins(0, 5, 0, 0));
            }

            // 设置标签文本：第一行ID，第二行类型
            label->setText(QString("ID:%1\n%2").arg(feb.id).arg(typeStr));

            // 字体样式优化
            label->setFont(QFont("Microsoft YaHei", 9, QFont::Bold));
            label->setColor(txtColor);

            // 添加半透明背景框，防止看不清
            label->setBrush(QBrush(QColor(255, 255, 255, 180)));
            label->setPen(Qt::NoPen);

            // 性能优化：移出屏幕不绘制
            label->setClipToAxisRect(true);

            // 加入列表管理（用于后续清除）
            m_febLabels.append(label);
        }

        qDebug() << "✅ 观测站可视化更新完毕: 涨潮站" << floodX.size() << "个, 落潮站" << ebbX.size() << "个";
    }

    // =========================================================
    // 【核心修复】自动计算数据的边界范围 (Bounding Box) - 包含所有数据类型
    // =========================================================
    double minX = 1e9, maxX = -1e9;
    double minY = 1e9, maxY = -1e9;
    bool hasData = false;

    // 遍历 LNDARE 更新边界 (Group 1)
    for (const auto &poly : m_landPolygons) {
        for (const auto &pt : poly.points) {
            if (pt.x() < minX) minX = pt.x();
            if (pt.x() > maxX) maxX = pt.x();
            if (pt.y() < minY) minY = pt.y();
            if (pt.y() > maxY) maxY = pt.y();
            hasData = true;
        }
    }

    // 遍历 DRGARE (疏浚航道) 更新边界
    for (const auto &poly : m_dredgedPolygons) {
        for (const auto &pt : poly.points) {
            if (pt.x() < minX) minX = pt.x();
            if (pt.x() > maxX) maxX = pt.x();
            if (pt.y() < minY) minY = pt.y();
            if (pt.y() > maxY) maxY = pt.y();
            hasData = true;
        }
    }

    // 遍历 BRIDGE 更新边界
    for (const auto &poly : m_bridgePolygons) {
        for (const auto &pt : poly.points) {
            if (pt.x() < minX) minX = pt.x();
            if (pt.x() > maxX) maxX = pt.x();
            if (pt.y() < minY) minY = pt.y();
            if (pt.y() > maxY) maxY = pt.y();
            hasData = true;
        }
    }

    // 遍历 PYLONS 更新边界
    for (const auto &poly : m_pylonsPolygons) {
        for (const auto &pt : poly.points) {
            if (pt.x() < minX) minX = pt.x();
            if (pt.x() > maxX) maxX = pt.x();
            if (pt.y() < minY) minY = pt.y();
            if (pt.y() > maxY) maxY = pt.y();
            hasData = true;
        }
    }

    if (hasData) {
        // 计算中心点和宽高
        double width = maxX - minX;
        double height = maxY - minY;

        // 增加 10% 的边距，避免图形顶格
        double marginX = (width > 0) ? width * 0.1 : 1000;
        double marginY = (height > 0) ? height * 0.1 : 1000;

        // 设置坐标轴范围
        m_aisPlot->xAxis->setRange(minX - marginX, maxX + marginX);
        m_aisPlot->yAxis->setRange(minY - marginY, maxY + marginY);

        qDebug() << "自动聚焦成功: Range X[" << minX << maxX << "] Y[" << minY << maxY << "]";
    } else {
        // 只有真的没有数据时，才使用默认值 (密西西比河 Mile 099)
        double defaultLon = -90.0182;
        double defaultLat = 29.9515;
        QPointF center = mercatorPoint(defaultLon, defaultLat);
        m_aisPlot->xAxis->setRange(center.x() - 5000, center.x() + 5000);
        m_aisPlot->yAxis->setRange(center.y() - 5000, center.y() + 5000);
        qDebug() << "无有效数据，使用默认视角 (密西西比河 Mile 099)";
    }

    // 【重要】绘图结束后手动调用一次可见性更新，确保初始状态正确
    updateLabelsVisibility();

    // 【新增】确保坐标轴正确缩放以显示所有数据
    m_aisPlot->rescaleAxes();

    // 【新增】在全流域加载后，强制矫正比例
    enforceAspectRatio();

    m_aisPlot->replot();
}

// 辅助函数：解析WKT点字符串，如"POINT(121.5 31.2)"，返回经纬度
QPair<double, double> parseWktPointToPair(const QString &wkt)
{
    // 移除POINT前缀和括号
    QString cleaned = wkt.trimmed();
    if (cleaned.startsWith("POINT(", Qt::CaseInsensitive)) {
        cleaned = cleaned.mid(6);
        if (cleaned.endsWith(")")) {
            cleaned = cleaned.left(cleaned.length() - 1);
        }
    }

    // 分割经纬度
    QStringList parts = cleaned.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
    if (parts.size() >= 2) {
        bool ok1, ok2;
        double lon = parts[0].toDouble(&ok1);
        double lat = parts[1].toDouble(&ok2);
        if (ok1 && ok2) {
            return QPair<double, double>(lon, lat);
        }
    }

    return QPair<double, double>(0.0, 0.0); // 默认值
}

void MainWindow::loadDepthData() {
    // 加载水深线数据
    m_depthLines.clear(); // 清空旧数据

     // 从数据库DEPCNT表加载水深线数据
     QSqlQuery query;

     // 查询DEPCNT表的必要字段：geometry_wkt（几何数据）和VALDCO（水深值）
     QString sql = "SELECT geometry_wkt, VALDCO FROM DEPCNT WHERE geometry_wkt IS NOT NULL AND VALDCO IS NOT NULL";

     if (!query.exec(sql)) {
         return;
     }

     while (query.next()) {
         QString geometryWkt = query.value(0).toString();
         double depthValue = query.value(1).toDouble();

         // 解析WKT几何数据为点序列
         QVector<QPointF> points = parseWktPoints(geometryWkt);

         if (points.isEmpty()) continue; // 跳过无效几何数据

         // 创建DepthLine对象并添加到向量中
         DepthLine depthLine;
         depthLine.points = points;
         depthLine.value = depthValue;

         m_depthLines.append(depthLine);
     }
}

// 【新增函数】专门绘制背景层：水域 -> 陆地 -> 桥梁
void MainWindow::drawBackgroundLayers()
{
    qDebug() << "探针 4k1: 进入 drawBackgroundLayers";
    qDebug() << "  - m_aisPlot:" << m_aisPlot;
    qDebug() << "  - m_aisPlot->xAxis:" << m_aisPlot->xAxis;
    qDebug() << "  - m_aisPlot->yAxis:" << m_aisPlot->yAxis;

    // 1. 遍历所有背景图层数据，按顺序绘制
    // 绘制顺序: DEPARE (水) -> LNDARE (陆地) -> DRGARE (航道) -> PYLONS (桥墩) -> BRIDGE (桥面)

    // =========================================================
    // DEPARE (水深区) - 分层设色 (S-52 风格)
    // =========================================================
    // 定义颜色常量
    QColor colorDeep(240, 248, 255);    // > 10m: 爱丽丝蓝 (接近白)
    QColor colorMedium(173, 216, 230);  // 5-10m: 浅蓝
    QColor colorShallow(135, 206, 250); // 2-5m:  天蓝 (稍深)
    QColor colorVeryShallow(70, 130, 180); // 0-2m: 钢蓝 (深)

    qDebug() << "探针 4k2: 准备绘制水深多边形，数量:" << m_depthPolygons.size();
    int depthCount = 0;
    for (const auto &poly : m_depthPolygons) {
        if (poly.points.isEmpty()) continue;

        qDebug() << "  - 绘制水深多边形 #" << depthCount << ", 点数:" << poly.points.size();
        QCPCurve *depthCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;
        for (const auto &pt : poly.points) { x.append(pt.x()); y.append(pt.y()); }
        if (!poly.points.isEmpty()) { x.append(poly.points.first().x()); y.append(poly.points.first().y()); }

        depthCurve->setData(x, y);
        depthCurve->setPen(Qt::NoPen); // 不要边框，让色块自然拼接

        // 【核心：根据深度选颜色】
        QColor brushColor;
        double d = poly.depth;

        if (d >= 10.0) {
            brushColor = colorDeep;
        } else if (d >= 5.0) {
            brushColor = colorMedium;
        } else if (d >= 2.0) {
            brushColor = colorShallow;
        } else {
            brushColor = colorVeryShallow; // 极浅水域或干出滩
        }

        // 设置一点透明度，让底图更有质感
        brushColor.setAlpha(200);
        depthCurve->setBrush(QBrush(brushColor));
        depthCurve->setSelectable(QCP::stNone);
        depthCount++;
    }
    qDebug() << "探针 4k3: 水深多边形绘制完成，共绘制" << depthCount << "个";

    // =========================================================
    // LNDARE (陆地) - 浅金菊黄，棕色边框
    // =========================================================
    qDebug() << "探针 4k4: 准备绘制陆地多边形，数量:" << m_landPolygons.size();
    int landCount = 0;
    for (const auto &poly : m_landPolygons) {
        if (poly.points.isEmpty()) continue;

        qDebug() << "  - 绘制陆地多边形 #" << landCount << ", 点数:" << poly.points.size();
        QCPCurve *landCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;
        for (const auto &pt : poly.points) {
            x.append(pt.x());
            y.append(pt.y());
        }
        // 闭合多边形
        if (!poly.points.isEmpty()) {
            x.append(poly.points.first().x());
            y.append(poly.points.first().y());
        }

        landCurve->setData(x, y);

        // 样式：浅金菊黄填充，棕色边框
        landCurve->setPen(QPen(QColor(160, 82, 45), 1)); // 棕色边框
        landCurve->setBrush(QBrush(QColor(255, 248, 220, 150))); // 浅金菊黄，透明度150
        landCount++;
    }
    qDebug() << "探针 4k5: 陆地多边形绘制完成，共绘制" << landCount << "个";

    // =========================================================
    // 【新增】绘制 ACHARE (锚地) - 黄色虚线框
    // =========================================================
    qDebug() << "探针 4k6: 准备绘制锚地多边形，数量:" << m_anchoragePolygons.size();
    int anchorageCount = 0;
    for (const auto &poly : m_anchoragePolygons) {
        if (poly.points.isEmpty()) continue;

        qDebug() << "  - 绘制锚地多边形 #" << anchorageCount << ", 点数:" << poly.points.size();
        QCPCurve *curve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;

        // 锚地通常是面，直接连线
        for (const auto &pt : poly.points) { x.append(pt.x()); y.append(pt.y()); }
        // 闭合
        if (poly.points.size() > 2) {
            x.append(poly.points.first().x()); y.append(poly.points.first().y());
        }

        curve->setData(x, y);

        // 样式：黄色虚线，无填充(或极淡填充)
        QPen pen(Qt::yellow);
        pen.setWidth(2);
        pen.setStyle(Qt::DashLine);
        curve->setPen(pen);
        curve->setBrush(Qt::NoBrush); // 空心框
        // curve->setBrush(QColor(255, 255, 0, 20)); // 或者极淡黄色填充
        anchorageCount++;
    }
    qDebug() << "探针 4k7: 锚地多边形绘制完成，共绘制" << anchorageCount << "个";

    // =========================================================
    // 【新增】绘制 OBSTRN / WRECKS (危险区) - 红色虚线框
    // =========================================================
    // 合并两个列表一起画，样式一样
    QVector<ChannelPoly> allDangers = m_obstructionPolygons + m_wreckPolygons;
    qDebug() << "探针 4k8: 准备绘制危险区多边形，数量:" << allDangers.size();
    int dangerCount = 0;

    for (const auto &poly : allDangers) {
        if (poly.points.isEmpty()) continue;

        qDebug() << "  - 绘制危险区多边形 #" << dangerCount << ", 点数:" << poly.points.size();
        QCPCurve *curve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;

        // 【关键处理】如果是单点(POINT)，画一个小方框
        if (poly.points.size() == 1) {
            double cx = poly.points.first().x();
            double cy = poly.points.first().y();
            double size = 0.0001; // 方框大小 (约10米)

            // 生成矩形四个角
            x << cx - size << cx + size << cx + size << cx - size << cx - size;
            y << cy - size << cy - size << cy + size << cy + size << cy - size;
        }
        else {
            // 如果是多边形，照常绘制
            for (const auto &pt : poly.points) { x.append(pt.x()); y.append(pt.y()); }
            x.append(poly.points.first().x()); y.append(poly.points.first().y()); // 闭合
        }

        curve->setData(x, y);

        // 样式：红色虚线
        QPen pen(Qt::red);
        pen.setWidth(2);
        pen.setStyle(Qt::DashLine);
        curve->setPen(pen);
        curve->setBrush(Qt::NoBrush); // 空心框
        dangerCount++;
    }
    qDebug() << "探针 4k9: 危险区多边形绘制完成，共绘制" << dangerCount << "个";

    // =========================================================
    // DRGARE (疏浚区/核心航道) - 深蓝色填充，深蓝色边框
    // =========================================================
    qDebug() << "探针 4k10: 准备绘制疏浚区多边形，数量:" << m_dredgedPolygons.size();
    int dredgedCount = 0;
    for (const auto &poly : m_dredgedPolygons) {
        if (poly.points.isEmpty()) continue;

        qDebug() << "  - 绘制疏浚区多边形 #" << dredgedCount << ", 点数:" << poly.points.size();
        QCPCurve *drgCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;
        for (const auto &pt : poly.points) {
            x.append(pt.x());
            y.append(pt.y());
        }
        // 闭合多边形
        if (!poly.points.isEmpty()) {
            x.append(poly.points.first().x());
            y.append(poly.points.first().y());
        }

        drgCurve->setData(x, y);

        // 样式：深蓝色填充，深蓝色边框
        drgCurve->setPen(QPen(QColor(0, 0, 139), 2)); // 深蓝色边框
        drgCurve->setBrush(QBrush(QColor(0, 0, 255, 50))); // 深蓝色填充，透明度50

        // 绘制计划航线 (中心线 - 黄色虚线)
        if (!poly.centerLine.isEmpty()) {
            qDebug() << "    - 绘制中心线，点数:" << poly.centerLine.size();
            QCPCurve *centerCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
            QVector<double> cx, cy;
            for (const auto &pt : poly.centerLine) {
                cx.append(pt.x());
                cy.append(pt.y());
            }
            centerCurve->setData(cx, cy);

            QPen centerPen;
            centerPen.setColor(Qt::yellow);
            centerPen.setWidth(2);
            centerPen.setStyle(Qt::DashLine);
            centerCurve->setPen(centerPen);
        }
        dredgedCount++;
    }
    qDebug() << "探针 4k11: 疏浚区多边形绘制完成，共绘制" << dredgedCount << "个";

    // =========================================================
    // PYLONS (桥墩/塔架) - 从数据库加载并绘制
    // =========================================================
    // 如果数据库中没有桥墩数据，则不绘制

    // =========================================================
    // 【新增】绘制 DEPCNT (水深线) + 数值标注
    // =========================================================
    qDebug() << "探针 4k13: 准备绘制水深线，数量:" << m_depthLines.size();
    int depthLineCount = 0;
    // 遍历所有水深线数据
    for (const auto &line : m_depthLines) {
        if (line.points.size() < 2) continue;

        // 1. 绘制线条
        QCPCurve *contour = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;
        for (const auto &pt : line.points) { x.append(pt.x()); y.append(pt.y()); }
        contour->setData(x, y);

        // 样式：浅水深色实线，深水浅色虚线
        QPen pen(Qt::gray);
        pen.setWidthF(1.0);
        if (line.value <= 5.0) { // 5米等深线及以下
            pen.setColor(QColor(100, 100, 100)); // 深灰
            pen.setStyle(Qt::SolidLine);
        } else {
            pen.setColor(QColor(160, 160, 160)); // 浅灰
            pen.setStyle(Qt::DashLine);
        }
        contour->setPen(pen);
        contour->setBrush(Qt::NoBrush);
        contour->setSelectable(QCP::stNone);

        // 2. 添加数值标签 (海图风格：小字，随线分布)
        // 为了性能，我们不给每个点加，只在线条的“中点”加一个标签
        int midIdx = line.points.size() / 2;
        QPointF midPt = line.points[midIdx];

        QCPItemText *depthLabel = new QCPItemText(m_aisPlot);
        depthLabel->position->setType(QCPItemPosition::ptPlotCoords);
        depthLabel->position->setCoords(midPt.x(), midPt.y());

        // 设置文本 (保留一位小数，如果是整数则不显示小数)
        QString text = QString::number(line.value, 'f', (int(line.value) == line.value ? 0 : 1));
        depthLabel->setText(text);

        // 海图风格字体：极小，斜体，灰色
        depthLabel->setFont(QFont("Arial", 7, QFont::Normal, true));
        depthLabel->setColor(Qt::black); // 或深灰色
        depthLabel->setBrush(Qt::NoBrush); // 无背景
        depthLabel->setPen(Qt::NoPen);     // 无边框

        // 优化：裁剪掉视口外的标签
        depthLabel->setClipToAxisRect(true);

        m_depthLabels.append(depthLabel); // 加入列表管理
        depthLineCount++;
    }
    qDebug() << "探针 4k14: 水深线绘制完成，共绘制" << depthLineCount << "条";

    // =========================================================
    // BRIDGE (桥梁主体) - 深灰色，边框+填充
    // =========================================================
    qDebug() << "探针 4k15: 准备绘制桥梁多边形，数量:" << m_bridgePolygons.size();
    int bridgeCount = 0;
    for (const auto &poly : m_bridgePolygons) {
        if (poly.points.isEmpty()) continue;

        qDebug() << "  - 绘制桥梁多边形 #" << bridgeCount << ", 点数:" << poly.points.size();
        QCPCurve *bridgeCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> x, y;
        for (const auto &pt : poly.points) {
            x.append(pt.x());
            y.append(pt.y());
        }
        // 闭合多边形
        if (!poly.points.isEmpty()) {
            x.append(poly.points.first().x());
            y.append(poly.points.first().y());
        }

        bridgeCurve->setData(x, y);

        // 样式：深灰色填充和边框
        bridgeCurve->setPen(QPen(QColor(64, 64, 64), 2)); // 深灰色边框
        bridgeCurve->setBrush(QBrush(QColor(128, 128, 128, 200))); // 灰色填充，透明度200

        // 如果有桥梁名称，在桥梁中心位置显示名称
        if (!poly.name.isEmpty()) {
            qDebug() << "    - 桥梁名称:" << poly.name;
            // 计算多边形的中心点（简单平均）
            double centerX = 0, centerY = 0;
            for (const auto &pt : poly.points) {
                centerX += pt.x();
                centerY += pt.y();
            }
            centerX /= poly.points.size();
            centerY /= poly.points.size();

            // 如果之前已经创建过，先删除旧的，防止重影
            qDebug() << "    - 检查是否需要删除旧的桥梁标签";
            if (m_bridgeLabel) {
                qDebug() << "    - 删除旧的桥梁标签";
                // 必须经 removeItem 删除：它会把标签移出绘图控件的列表并释放对象。
                // 直接 delete 会在列表里留下悬空指针，QCustomPlot 析构时再次释放导致崩溃
                m_aisPlot->removeItem(m_bridgeLabel);
                m_bridgeLabel = nullptr;
                qDebug() << "    - 旧标签删除成功";
            }

            qDebug() << "    - 创建新的桥梁标签对象";
            m_bridgeLabel = new QCPItemText(m_aisPlot);
            qDebug() << "    - 桥梁标签对象创建成功";
            m_bridgeLabel->setText(poly.name); // 使用数据库中的桥梁名称
            qDebug() << "    - 设置文本成功";
            m_bridgeLabel->position->setType(QCPItemPosition::ptPlotCoords);
            qDebug() << "    - 设置位置类型成功";
            m_bridgeLabel->position->setCoords(centerX, centerY);
            qDebug() << "    - 设置坐标成功";

            // 【样式优化】
            // 1. 去掉黑底
            m_bridgeLabel->setBrush(Qt::NoBrush); // 透明背景
            qDebug() << "    - 设置画刷成功";
            m_bridgeLabel->setPen(Qt::NoPen);     // 无边框
            qDebug() << "    - 设置画笔成功";

            // 2. 固定字号 (setPointSize 是物理像素大小，不随缩放改变)
            QFont font("Arial", 12, QFont::Bold);
            m_bridgeLabel->setFont(font);
            qDebug() << "    - 设置字体成功";

            // 3. 颜色设为深色以对比水面
            m_bridgeLabel->setColor(Qt::black);
            qDebug() << "    - 设置颜色成功";
            m_bridgeLabel->setPositionAlignment(Qt::AlignCenter);
            qDebug() << "    - 设置对齐方式成功";
        }
        bridgeCount++;
    }
    qDebug() << "探针 4k16: 桥梁多边形绘制完成，共绘制" << bridgeCount << "个";

    // =========================================================
    // 【新增】绘制桥区控制区域 (Bridge Zone)
    // =========================================================
    if (!m_bridgeZonePolygon.isEmpty()) {
        QCPCurve *zoneCurve = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        QVector<double> zx, zy;
        for (const auto& pt : m_bridgeZonePolygon) {
            zx.append(pt.x());
            zy.append(pt.y());
        }
        // 闭合多边形
        if (!m_bridgeZonePolygon.isEmpty()) {
            zx.append(m_bridgeZonePolygon.first().x());
            zy.append(m_bridgeZonePolygon.first().y());
        }
        zoneCurve->setData(zx, zy);

        // 样式：橙色虚线边框，半透明填充
        QPen pen;
        pen.setColor(QColor(255, 165, 0)); // 橙色
        pen.setWidth(2);
        pen.setStyle(Qt::DashDotLine);
        zoneCurve->setPen(pen);
        zoneCurve->setBrush(QBrush(QColor(255, 165, 0, 30))); // 透明橙色
        zoneCurve->setName("Bridge Control Zone");
    }
}



// ---------- 绘制本船 ----------
// 确保这个函数定义在 drawOwnShip 之前
QPointF rotatePoint(QPointF point, double headingRad) {
    double cosH = cos(headingRad);
    double sinH = sin(headingRad);
    double xNew = point.x() * cosH + point.y() * sinH;
    double yNew = -point.x() * sinH + point.y() * cosH;
    return QPointF(xNew, yNew);
}

void MainWindow::drawOwnShip()
{
    // 1. 清理旧图形
    // 注意：要清理的目标船名字前缀是 TargetShip
    for (int i = m_aisPlot->plottableCount() - 1; i >= 0; --i) {
        QString name = m_aisPlot->plottable(i)->name();
        if (name == "OwnShipBody" || name == "OwnShipHeading" ||
            name == "OwnShipTrail" || name == "OwnShipDomain" ||
            name.startsWith("TargetShip")) {
            m_aisPlot->removePlottable(i);
        }
    }

    // 辅助 Lambda: 旋转坐标
    auto rotate = [](QPointF p, double angle) {
        return QPointF(p.x() * cos(angle) + p.y() * sin(angle),
                       -p.x() * sin(angle) + p.y() * cos(angle));
    };

    // =========================================================
    // 2. 绘制本船 (红色，带轨迹，带领域)
    // =========================================================
    double x = m_ownShip.X;
    double y = m_ownShip.Y;
    double hRad = m_ownShip.C;
    double L = m_ownShip.L;
    double B = m_ownShip.B;

    // 2.1 绘制历史轨迹
    if (!m_ownShipHistory.isEmpty()) {
        QCPCurve *trail = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        trail->setName("OwnShipTrail");

        QVector<double> tx, ty;
        for(const auto& p : m_ownShipHistory) {
            tx.append(p.x());
            ty.append(p.y());
        }
        tx.append(x);
        ty.append(y);

        trail->setData(tx, ty);
        QPen trailPen(Qt::darkGray);
        trailPen.setWidthF(1.5);
        trail->setPen(trailPen);
    }

    // 2.2 绘制船舶领域
    QCPCurve *domain = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
    domain->setName("OwnShipDomain");

    double axisLong = 1.5 * L;
    double axisShort = 0.25 * L;

    QVector<QPointF> domainPts;
    for (int i = 0; i <= 360; i += 5) {
        double rad = i * M_PI / 180.0;
        double lx = axisShort * cos(rad);
        double ly = axisLong * sin(rad);
        domainPts.append(QPointF(lx, ly));
    }

    QVector<double> dx, dy;
    for (const auto& p : domainPts) {
        QPointF rp = rotatePoint(p, hRad);
        dx.append(x + rp.x());
        dy.append(y + rp.y());
    }

    domain->setData(dx, dy);
    QPen domainPen(Qt::blue);
    domainPen.setWidth(1);
    domainPen.setStyle(Qt::DashLine);
    domain->setPen(domainPen);
    domain->setBrush(QBrush(QColor(0, 0, 255, 30)));

    // 2.3 绘制船体
    QCPCurve *shipBody = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
    shipBody->setName("OwnShipBody");

    QVector<QPointF> pts;
    pts << QPointF(0, L/2.0) << QPointF(B/2.0, L/3.0) << QPointF(B/2.0, -L/2.0)
        << QPointF(-B/2.0, -L/2.0) << QPointF(-B/2.0, L/3.0) << QPointF(0, L/2.0);

    QVector<double> bx, by;
    for (const auto& p : pts) {
        QPointF rp = rotatePoint(p, hRad);
        bx.append(x + rp.x());
        by.append(y + rp.y());
    }
    shipBody->setData(bx, by);
    shipBody->setPen(QPen(Qt::black, 2));
    shipBody->setBrush(QBrush(QColor(255, 0, 0, 200)));

    // 2.4 绘制航向线
    QCPCurve *headingLine = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
    headingLine->setName("OwnShipHeading");

    QVector<QPointF> hpts;
    hpts << QPointF(0, 0) << QPointF(0, L * 1.0);

    QVector<double> hx, hy;
    for (const auto& p : hpts) {
        QPointF rp = rotatePoint(p, hRad);
        hx.append(x + rp.x());
        hy.append(y + rp.y());
    }
    headingLine->setData(hx, hy);
    headingLine->setPen(QPen(Qt::yellow, 3));

    // =========================================================
    // 3. 绘制目标船 (青色，只画船体，不画轨迹)
    // =========================================================
    int targetIndex = 0;
    for (const Ship& target : m_targetShips) {
        double tx = target.X;
        double ty = target.Y;
        double thRad = target.C;
        double tL = target.L;
        double tB = target.B;

        // 3.1 绘制目标船船体
        QCPCurve *targetBody = new QCPCurve(m_aisPlot->xAxis, m_aisPlot->yAxis);
        targetBody->setName(QString("TargetShip_%1").arg(targetIndex++));

        QVector<QPointF> tPts;
        tPts << QPointF(0, tL/2.0) << QPointF(tB/2.0, tL/3.0) << QPointF(tB/2.0, -tL/2.0)
             << QPointF(-tB/2.0, -tL/2.0) << QPointF(-tB/2.0, tL/3.0) << QPointF(0, tL/2.0);

        QVector<double> tbx, tby;
        for (const auto& p : tPts) {
            QPointF rp = rotatePoint(p, thRad);
            tbx.append(tx + rp.x());
            tby.append(ty + rp.y());
        }
        targetBody->setData(tbx, tby);
        targetBody->setPen(QPen(Qt::black, 2));
        targetBody->setBrush(QBrush(QColor(0, 255, 255, 200))); // 青色填充
    }

    // =========================================================
    // 4. 视角锁定
    // =========================================================
    double rangeX = m_aisPlot->xAxis->range().size();
    double rangeY = m_aisPlot->yAxis->range().size();
    if (rangeX < 100) rangeX = 4000;
    if (rangeY < 100) rangeY = 4000;

    m_aisPlot->xAxis->setRange(x - rangeX/2, x + rangeX/2);
    m_aisPlot->yAxis->setRange(y - rangeY/2, y + rangeY/2);

    // 【新增】在每帧镜头移动后，强制矫正比例
    enforceAspectRatio();

    m_aisPlot->replot(QCustomPlot::rpQueuedReplot);
}

void MainWindow::onTimerTick()
{
    float dt = 0.1f;
    int steps = 5;
    g_globalLogCounter++;

    // 🌟 修改 2：定义当前激活的航道为下行航道！
    // 🌟 彻底修改：定义当前激活的航道为上行航道！
    const QVector<QPointF>& activeLane = m_laneUpstream;

    if (activeLane.size() < 10) return;

    // =========================================================
    // 【新增】 自动到达终点检测与录制停止机制
    // =========================================================
    // 提取航线的最后一个点作为物理终点
    QPointF destination = activeLane.last();
    double distSqToDest = std::pow(m_ownShip.X - destination.x(), 2) + std::pow(m_ownShip.Y - destination.y(), 2);

    // 设定到达阈值：距离终点小于 200 米 (约一倍船长) 认为到达
    if (distSqToDest < (200.0 * 200.0)) {
        qDebug() << "🏁 本船已抵达终点，正在自动停止仿真与录制...";

        // 1. 停止物理仿真循环
        m_timer->stop();
        m_btnRun->setText("开始仿真 (Run)");

        // 2. 更新仪表盘状态栏
        m_lblStatus->setText("🏁 ARRIVED: SIMULATION FINISHED");
        m_lblStatus->setStyleSheet("background-color: #9C27B0; color: white; font: bold 14px 'Arial'; padding: 5px;");

        // 3. 自动停止 CSV 数据录制 (复用按钮的逻辑)
        if (m_isRecordingCSV) {
            // 直接调用录制按钮的槽函数，它会自动关闭文件、恢复按钮颜色并弹窗提示保存路径
            onBtnRecordValidationClicked();
        }

        // 4. 弹出仿真完成提示框（批处理时只记录状态）
        m_batchArrived = true;
        if (!m_batchMode) {
            QMessageBox::information(this, "航行结束", "本船已成功抵达航道终点！\n\n物理仿真与数据录制均已自动停止。");
        }

        return; // 彻底退出当前的 Tick，不再执行后续移动
    }
    // =========================================================

    // =========================================================
    // 【修改】1. 航迹预测调度 (每 1秒/33帧 执行一次) - 降低CPU负担
    // =========================================================
    /* 注释掉这整个块，避免时间预测与空间预瞄发生冲突！
    if (g_globalLogCounter % 33 == 0) {
        // 预测未来 4 分钟 (240秒)
        double maxFutureXTE = m_trajectoryPredictor->predict(m_ownShip, 240.0);
        bool riskDetected = (maxFutureXTE > 5.0);
        if (riskDetected != m_futureRiskWarning) {
            m_futureRiskWarning = riskDetected;
        }
    }
    */

    // 【核心修复】：强制关闭时间维度预测警告，纯靠空间曲率预瞄和底层高敏PID来抗饱和！
    m_futureRiskWarning = false;

    // =========================================================
    // 2. 物理仿真循环
    // =========================================================
    int turnOS = 0; float aimNp = 0;
    for (int k = 0; k < steps; ++k) {
        m_totalSimTime += dt;

        // ====================================================================
        // --- A. 桥区检测 (已关闭，当前水域无复杂桥区限速需求) ---
        // ====================================================================
        bool isInBridgeArea = false;

        /* 🌟 将这整段遍历测距的代码注释掉，彻底屏蔽桥区降速 🌟
        QPointF shipPos(m_ownShip.X, m_ownShip.Y);
        for (const auto& bridge : m_bridgePolygons) {
            if (bridge.points.isEmpty()) continue;

            double bx = 0, by = 0;
            for (const auto& pt : bridge.points) {
                bx += pt.x();
                by += pt.y();
            }
            bx /= bridge.points.size();
            by /= bridge.points.size();

            double dx = shipPos.x() - bx;
            double dy = shipPos.y() - by;
            double distanceToBridge = std::sqrt(dx * dx + dy * dy);

            if (distanceToBridge < 1500.0) {
                isInBridgeArea = true;
                break;
            }
        }
        */

        if (isInBridgeArea != m_ownShip.isInsideBridgeArea) {
            m_ownShip.isInsideBridgeArea = isInBridgeArea;
        }
        // ====================================================================

        // --- B. 本船控制 (四组消融实验) ---
        // 🔀 获取当前的实验对照模式 (0, 1, 2, 3)
        int controlMode = m_cmbControlMode->currentIndex();

        // =========================================================
        // 🌟【核心闭环】构建 2D 流场感知接口，并喂给 8单元船体
        // =========================================================
        auto get_water_2d_interface = [&](double x, double y, double& uw, double& vw) {
             QPointF flowVec = m_bridgeEnv.computeAt(x, y, m_totalSimTime);
             uw = flowVec.x();
             vw = flowVec.y();
        };
        m_ownShip.calculate_fluid_interaction(get_water_2d_interface);

        // 获取当前环境流场 (供 UI 使用)
        QPointF flowVec = m_bridgeEnv.computeAt(m_ownShip.X, m_ownShip.Y, m_totalSimTime);
        EnvData env;
        env.currentSpeed = sqrt(flowVec.x()*flowVec.x() + flowVec.y()*flowVec.y());
        env.currentDir = atan2(flowVec.x(), flowVec.y());
        env.waterDepth = m_bridgeEnv.getDepthAt(m_ownShip.X, m_ownShip.Y);
        env.windSpeed = 5.0;
        env.windDir = 0.0;

        // 🔴 如果是前三组消融实验 (Group 1, 2, 3)
        if (controlMode == 0 || controlMode == 1 || controlMode == 2) {

            // 传统实验组强制不遵守桥区规则 (全程保持恒定转速，默认 100 RPM)
            float commandRpm = static_cast<float>(m_baseRpm);
            m_ownShip.np = commandRpm / 60.0;

            // 调用重构后的 BaselineController，传入实验组号 (controlMode + 1 即 1, 2, 3)
            double rudderCmd = m_baselineController->execute(m_ownShip, dt, controlMode + 1);

            m_ownShip.delta = rudderCmd;

            // 计算一次虚假的 LOS，仅仅是为了让系统算出 currentXTE 供 CSV 录制，不用于打舵
            calculateLOSAimCourse(m_ownShip, false);

            // 执行运动
            m_ownShip.bianxiang_move(m_ownShip.delta, dt, env, commandRpm);
            m_currentRudderAngle = m_ownShip.delta;

        } else {
            // 🟢 如果是模式 3 (本文实验组 Group 4: 全功能 Proposed)

            // 1. 计算目标航向 (包含 曲率-偏差双自适应, 强化 Lyapunov ILOS)
            double finalAimC = calculateLOSAimCourse(m_ownShip, false);

            // 2. 避碰介入 (保持原有逻辑)
            QVector<ChannelPoly> staticObstacles = m_obstructionPolygons + m_wreckPolygons;
            float avoidAngleRad = m_huiyu.analysis(m_ownShip, m_targetShips, staticObstacles, turnOS, aimNp, finalAimC);
            if (turnOS != 0) {
                finalAimC = m_ownShip.C + avoidAngleRad;
            }

            // 3. 执行本文的极简定常 PD + FSI 低通滤波前馈
            runPIDControl(m_ownShip, finalAimC, dt);

            // 4. 【核心合规与控速逻辑】：对齐对地航速
            float normalSpeedRpm = static_cast<float>(m_ownRpm); // 默认 94.5，可用 --own-rpm 与对照组对齐
            float currentBaseRpm = normalSpeedRpm;

            if (aimNp > (normalSpeedRpm / 60.0)) {
                aimNp = normalSpeedRpm / 60.0;
            }
            float commandRpm = (aimNp > 0.1) ? aimNp * 60.0 : currentBaseRpm;

            // 执行运动
            m_ownShip.bianxiang_move(m_ownShip.delta, dt, env, commandRpm);
            m_currentRudderAngle = m_ownShip.delta;
        }

        // --- C. 目标船控制 (保持不变) ---
        for (Ship &target : m_targetShips) {
            double targetAimC = calculateLOSAimCourse(target, false); // 也可以复用
            runPIDControl(target, targetAimC, dt);

            // 简化的目标船环境...
            QPointF tFlow = m_bridgeEnv.computeAt(target.X, target.Y, m_totalSimTime);
            EnvData tEnv;
            tEnv.currentSpeed = sqrt(tFlow.x()*tFlow.x() + tFlow.y()*tFlow.y());
            tEnv.currentDir = atan2(tFlow.x(), tFlow.y());
            tEnv.waterDepth = 20.0;
            target.bianxiang_move(target.delta, dt, tEnv, 85.0);
        }

        // --- D. 轨迹记录 ---
        static QPointF lastPos;
        if (lastPos.isNull() || (pow(m_ownShip.X - lastPos.x(), 2) + pow(m_ownShip.Y - lastPos.y(), 2) > 100)) {
            m_ownShipHistory.append(QPointF(m_ownShip.X, m_ownShip.Y));
            lastPos = QPointF(m_ownShip.X, m_ownShip.Y);
            if (m_ownShipHistory.size() > 5000) m_ownShipHistory.removeFirst();
        }
    }

    // 【新增】 更新仪表盘数据
    // =========================================================
    // 1. 速度 (m/s 转 节)
    // 对水航速来自船体纵荡；对地航速还要叠加流场，顶流上行两者相差 2~3 kn，
    // 原先只显示 v/0.5144 却标成 SOG，等于把对水航速当成了对地航速。
    double stwKnots = m_ownShip.v / 0.5144;
    QPointF uiFlow = m_bridgeEnv.computeAt(m_ownShip.X, m_ownShip.Y, m_totalSimTime);
    double gvx = m_ownShip.v * sin(m_ownShip.C) + m_ownShip.u * cos(m_ownShip.C) + uiFlow.x();
    double gvy = m_ownShip.v * cos(m_ownShip.C) - m_ownShip.u * sin(m_ownShip.C) + uiFlow.y();
    double sogKnots = sqrt(gvx * gvx + gvy * gvy) / 0.5144;
    m_lblSpeed->setText(QString("对水 %1 | 对地 %2 kn").arg(stwKnots, 0, 'f', 1).arg(sogKnots, 0, 'f', 1));

    // 2. 航向 (弧度 转 度，0~360)
    double headingDeg = m_ownShip.C * 180.0 / M_PI;
    while (headingDeg < 0) headingDeg += 360.0;
    while (headingDeg >= 360) headingDeg -= 360.0;
    m_lblHeading->setText(QString("HDG: %1°").arg(headingDeg, 0, 'f', 1));

    // 3. 舵角 (度)
    m_lblRudder->setText(QString("Rudder: %1°").arg(m_ownShip.delta, 0, 'f', 1));

    // 4. 状态显示 (使用主仿真循环中的结果，避免重复计算)
    // turnOS 和 aimNp 已经在主仿真循环中计算过了

    // 5. 仿真时间显示
    // 直接以秒为单位显示仿真时间
    m_lblSimTime->setText(QString("TIME: %1 s").arg(static_cast<int>(m_totalSimTime)));

    // 首先检查是否在桥区水域
    if (m_ownShip.isInsideBridgeArea) {
        m_lblStatus->setText("🚢 IN BRIDGE AREA: LIMIT SPEED");
        m_lblStatus->setStyleSheet("background-color: #FFA500; color: white; font: bold 14px 'Arial'; padding: 5px;");
    } else if (turnOS != 0) {
        m_lblStatus->setText("⚠️ WARNING: COLLISION AVOIDANCE ACTIVE");
        m_lblStatus->setStyleSheet("background-color: #CC0000; color: white; font: bold 14px 'Arial'; padding: 5px;");
    } else {
        m_lblStatus->setText("✅ AUTO PILOT: TRACK KEEPING");
        m_lblStatus->setStyleSheet("background-color: #006600; color: white; font: bold 14px 'Arial'; padding: 5px;");
    }

    // ============================================================
    // 【新增功能】 实时环境数据显示 (水深 & 流速流向)
    // ============================================================

    // 1. 获取本船当前的墨卡托坐标
    double shipX = m_ownShip.X;
    double shipY = m_ownShip.Y;

    // ----------------------
    // A. 水深显示逻辑
    // ----------------------
    // 调用刚才写的接口
    double currentDepth = m_bridgeEnv.getDepthAt(shipX, shipY);

    // 更新 UI 文本
    m_lblDepthVal->setText(QString::number(currentDepth, 'f', 1) + " m");

    // 浅水报警功能：如果水深 < 8米，变成红色警示
    if (currentDepth < 8.0) {
        m_lblDepthVal->setStyleSheet("QLabel { background-color: #222; color: red; font: bold 16px 'Consolas'; padding: 5px; border: 1px solid #555; border-radius: 4px; }");
    } else {
        // 恢复正常青色
        m_lblDepthVal->setStyleSheet("QLabel { background-color: #222; color: cyan; font: bold 14px 'Consolas'; padding: 5px; border: 1px solid #555; border-radius: 4px; }");
    }

    // ----------------------
    // B. 流速流向显示逻辑
    // ----------------------
    // 获取当前仿真时间 t
    double t_now = m_totalSimTime;

    // 计算该点的流场矢量
    QPointF flowVec = m_bridgeEnv.getFlowVectorAt(shipX, shipY, t_now);

    // 1. 计算流速 (模长) -> 转换为节 (1 m/s = 1.94384 kn)
    double flowSpeedMs = std::sqrt(flowVec.x() * flowVec.x() + flowVec.y() * flowVec.y());
    double flowSpeedKn = flowSpeedMs * 1.94384;

    // 2. 计算流向 (角度)
    // atan2 返回的是数学角度 (弧度，逆时针，0度指向右/东)
    double flowAngleRad = std::atan2(flowVec.y(), flowVec.x());
    double flowAngleDeg = flowAngleRad * 180.0 / M_PI;

    // 转换为航海角度 (顺时针，0度指向上/北)
    // 公式: NavAngle = 90 - MathAngle
    double navFlowDir = 90.0 - flowAngleDeg;

    // 归一化到 0~360 度
    while (navFlowDir < 0) navFlowDir += 360.0;
    while (navFlowDir >= 360.0) navFlowDir -= 360.0;

    // 更新 UI
    m_lblFlowSpdVal->setText(QString::number(flowSpeedKn, 'f', 1) + " kn");
    // %1 表示参数1，3表示总宽3位，'f'浮点，0表示保留0位小数，'0'表示前补零 (例如 045°)
    m_lblFlowDirVal->setText(QString("%1°").arg(navFlowDir, 3, 'f', 0, '0'));

    // ============================================================
    // 【结束】
    // ============================================================

    // =========================================================
    // 【CSV 录制逻辑】 (仅修改这里，复用 UI 的计算结果)
    // =========================================================
    if (m_isRecordingCSV && m_csvFile.isOpen()) {

        // 1. 墨卡托坐标转经纬度 (WGS84)
        double lon = 0.0, lat = 0.0;
        Mercator2lonLat(m_ownShip.X, m_ownShip.Y, lon, lat); // 内部已含 MERC_SCALE

        // 【核心新增：计算本船当前受流】
        // 调用底层的 computeAt 获取本船坐标在这一瞬间的绝对流场力
        QPointF shipFlow = m_bridgeEnv.computeAt(m_ownShip.X, m_ownShip.Y, m_totalSimTime);

        // 计算遭遇流速 (节)
        double encountered_spd_kn = std::sqrt(shipFlow.x()*shipFlow.x() + shipFlow.y()*shipFlow.y()) * 1.94384;

        // 计算遭遇流向 (度，航海标准 0~360)
        double encountered_dir_deg = 90.0 - (std::atan2(shipFlow.y(), shipFlow.x()) * 180.0 / M_PI);
        while(encountered_dir_deg < 0) encountered_dir_deg += 360.0;
        while(encountered_dir_deg >= 360.0) encountered_dir_deg -= 360.0;

        // 2. 获取和 UI 一致的基础数据
        // 注意：这里我们直接用 m_ownShip 的原始数据重算一遍，确保和 UI 逻辑一致
        // 这样就不需要去改上面的代码提取变量了，最安全

        // (1) 速度：对水（船体纵荡）与对地（叠加流场）分列，勿再混用
        double rec_stw_kn = m_ownShip.v / 0.5144;
        const double rec_gvx = m_ownShip.v * sin(m_ownShip.C) + m_ownShip.u * cos(m_ownShip.C) + shipFlow.x();
        const double rec_gvy = m_ownShip.v * cos(m_ownShip.C) - m_ownShip.u * sin(m_ownShip.C) + shipFlow.y();
        double rec_sog_kn = std::sqrt(rec_gvx * rec_gvx + rec_gvy * rec_gvy) / 0.5144;

        // (2) 船首向 (0~360度)
        double rec_hdg_deg = m_ownShip.C * 180.0 / M_PI;
        while (rec_hdg_deg < 0) rec_hdg_deg += 360.0;
        while (rec_hdg_deg >= 360) rec_hdg_deg -= 360.0;

        // (3) 舵角
        double rec_rudder = m_ownShip.delta;

        // 3. 计算论文需要的额外数据 (XTE, COG, Crab)

        // (4) 横向偏差 (XTE)
        double rec_xte = m_ownShip.currentXTE;

        // (5) 航迹向 (COG) - 利用位移差分计算
        static QPointF rec_lastP(m_ownShip.X, m_ownShip.Y);
        double dx = m_ownShip.X - rec_lastP.x();
        double dy = m_ownShip.Y - rec_lastP.y();
        double rec_cog_deg = rec_hdg_deg; // 默认等于船头

        // 只有当位移足够大时才更新 COG
        if (dx*dx + dy*dy > 1e-8) {
             double rad = atan2(dx, dy);
             if(rad < 0) rad += 2*M_PI;
             rec_cog_deg = rad * 180.0 / M_PI;
        }
        rec_lastP = QPointF(m_ownShip.X, m_ownShip.Y); // 更新位置

        // (6) 蟹行角
        double rec_crab_deg = rec_hdg_deg - rec_cog_deg;
        while(rec_crab_deg > 180) rec_crab_deg -= 360;
        while(rec_crab_deg < -180) rec_crab_deg += 360;

        // =========================================================
        // 【修改】二维平面流固耦合：不再有横摇和纵摇力矩
        // =========================================================
        double rec_roll_moment = 0.0;  // 横摇力矩设为0（二维）
        double rec_pitch_moment = 0.0; // 纵摇力矩设为0（二维）

        // 4. 写入 CSV
        // 格式: time, lon, lat, sog, hdg, rudder, xte, cog, crab, flow_spd, flow_dir, roll_moment, pitch_moment
        m_csvStream << QString::number(m_totalSimTime, 'f', 2) << ","
                    << QString::number(lon, 'f', 6) << ","    // 经度 (保留6位小数，约0.1米精度)
                    << QString::number(lat, 'f', 6) << ","    // 纬度
                    << QString::number(rec_stw_kn, 'f', 2) << ","
                    << QString::number(rec_sog_kn, 'f', 2) << ","
                    << QString::number(rec_hdg_deg, 'f', 2) << ","
                    << QString::number(rec_rudder, 'f', 2) << ","
                    << QString::number(rec_xte, 'f', 4) << ","
                    << QString::number(rec_cog_deg, 'f', 2) << ","
                    << QString::number(rec_crab_deg, 'f', 2) << ","
                    << QString::number(encountered_spd_kn, 'f', 2) << ","  // 新增受流流速
                    << QString::number(encountered_dir_deg, 'f', 1) << ","
                    << QString::number(rec_roll_moment, 'f', 2) << ","   // 写入横摇
                    << QString::number(rec_pitch_moment, 'f', 2) << "\n"; // 写入纵摇

        m_csvStream.flush(); // 强制刷盘

        // =========================================================
        // 【修改】同步动态写入对应的 FSI_Elements_Log (8单元受力数据)
        // =========================================================
        int mode = m_cmbControlMode->currentIndex();
        QString suffix;
        if (mode == 0)      suffix = "Group1_Baseline";
        else if (mode == 1) suffix = "Group2_ConvILOS";
        else if (mode == 2) suffix = "Group3_Feedforward";
        else                suffix = "Group4_Proposed";

        QString fsiFileName = outputDir() + QString("/FSI_%1.csv").arg(suffix);

        QFile fsiFile(fsiFileName);
        if (fsiFile.open(QIODevice::Append | QIODevice::Text)) {
            QTextStream fsiOut(&fsiFile);
            fsiOut << QString::number(m_totalSimTime, 'f', 2);
            for (int i = 0; i < 8; ++i) {
                fsiOut << "," << QString::number(m_ownShip.m_fsi_log[i].dX, 'f', 6)
                       << "," << QString::number(m_ownShip.m_fsi_log[i].dY, 'f', 6);
            }
            fsiOut << "\n";
            fsiFile.close();
        }

        // 调试日志 (每秒打印一次)
        static int logCnt = 0;
        if (logCnt++ % 30 == 0) {
            qDebug() << "🔥 [CSV录制] T=" << m_totalSimTime
                     << " HDG=" << rec_hdg_deg
                     << " COG=" << rec_cog_deg
                     << " XTE=" << rec_xte
                     << " Rudder=" << rec_rudder
                     << " Crab=" << rec_crab_deg
                     << " FlowSpd=" << encountered_spd_kn
                     << " FlowDir=" << encountered_dir_deg;
        }
    }

    if (!m_batchMode) {
        drawOwnShip(); // 刷新界面
        updateLabelsVisibility();
    }
}


// ---------- 蟹行验证录制按钮槽函数 ----------
void MainWindow::onBtnRecordValidationClicked()
{
    m_isRecordingCSV = !m_isRecordingCSV; // 切换录制状态

    if (m_isRecordingCSV) {
        // === 🔴 开始录制 ===
        QString appPath = outputDir();

        // =========================================================
        // 🌟 核心新增：根据当前下拉菜单选项，动态生成完美的 CSV 文件名
        // =========================================================
        int mode = m_cmbControlMode->currentIndex();
        QString suffix;
        if (mode == 0)      suffix = "Group1_Baseline";
        else if (mode == 1) suffix = "Group2_ConvILOS";
        else if (mode == 2) suffix = "Group3_Feedforward";
        else                suffix = "Group4_Proposed";

        QString shipLogPath = appPath + QString("/Log_%1.csv").arg(suffix);
        QString fsiLogPath = appPath + QString("/FSI_%1.csv").arg(suffix);

        qDebug() << "📁 动态录制路径已设为: " << shipLogPath;

        // 删掉旧文件 (如果有同名遗留)
        QFile::remove(appPath + "/Flow_Eulerian_Validation.csv");
        QFile::remove(shipLogPath);
        QFile::remove(fsiLogPath);

        m_csvFile.setFileName(shipLogPath);

        // 覆盖模式打开文件
        if (m_csvFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            m_csvStream.setDevice(&m_csvFile);

            // 写入表头
            m_csvStream << "sim_time,lon,lat,stw_kn,sog_kn,heading_deg,rudder_deg,xte_m,cog_deg,crab_angle_deg,flow_spd_kn,flow_dir_deg,roll_moment,pitch_moment\n";
            m_csvStream.flush();
            qDebug() << "✅ 主日志创建成功: " << shipLogPath;

            // 初始化对应的 FSI_Elements_Log.csv 文件并写入表头
            QFile fsiFile(fsiLogPath);
            if (fsiFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
                QTextStream fsiOut(&fsiFile);
                fsiOut << "sim_time";
                for (int i = 0; i < 8; ++i) {
                    fsiOut << QString(",E%1_dX,E%1_dY").arg(i);
                }
                fsiOut << "\n";
                fsiFile.close();
            }

            m_btnRecordValidation->setText(QString("🔴 正在录制 [%1]...").arg(suffix));
            m_btnRecordValidation->setStyleSheet("background-color: #E91E63; color: white; font-weight: bold;");
        } else {
            if (!m_batchMode) {
                QMessageBox::critical(this, "录制失败", "无法创建日志文件！");
            } else {
                qDebug() << "❌ 无法创建日志文件:" << shipLogPath;
            }
            m_isRecordingCSV = false;
        }

    } else {
        // === ⚫ 停止录制 ===
        if (m_csvFile.isOpen()) {
            m_csvStream.flush();
            m_csvFile.close();
            qDebug() << "💾 CSV已保存关闭。";
        }
        m_btnRecordValidation->setText("录制验证数据 (Off)");
        m_btnRecordValidation->setStyleSheet("background-color: #757575; color: white;");

        // 检查文件是否实际创建
        int mode = m_cmbControlMode->currentIndex();
        QString suffix;
        if (mode == 0)      suffix = "Group1_Baseline";
        else if (mode == 1) suffix = "Group2_ConvILOS";
        else if (mode == 2) suffix = "Group3_Feedforward";
        else                suffix = "Group4_Proposed";

        QString appPath = outputDir();
        QFileInfo shipLogInfo(appPath + QString("/Log_%1.csv").arg(suffix));
        QFileInfo fsiLogInfo(appPath + QString("/FSI_%1.csv").arg(suffix));

        QString msg = QString("录制完成！\n\n") +
                      QString("Log_%1.csv: %2\n").arg(suffix).arg(shipLogInfo.exists() ? QString("存在 (%1 字节)").arg(shipLogInfo.size()) : "不存在") +
                      QString("FSI_%1.csv: %2").arg(suffix).arg(fsiLogInfo.exists() ? QString("存在 (%1 字节)").arg(fsiLogInfo.size()) : "不存在");
        if (!m_batchMode) {
            QMessageBox::information(this, "录制完成", msg);
        } else {
            qDebug().noquote() << msg;
        }
    }
}

// ---------- 开始仿真槽函数 ----------
void MainWindow::onBtnRunClicked()
{
    if (m_timer->isActive()) return;
    qDebug() << "=== 仿真启动（动态航道起点模式） ===";

    // 设置为3/4潮汐周期
    m_totalSimTime = T_TIDAL * 3.0 / 4.0; // 落潮最低潮
    m_ownShipHistory.clear();
    m_targetShips.clear(); // 清空旧数据

    // 【修复 1】重置流场滤波累积量，防止继承上次的残留值
    m_filteredDrift = 0.0;
    // 【修复 2】重置PID积分项，防止积分饱和残留
    m_ownShip.pid_integratedErr = 0.0;
    m_ownShip.pid_prevError     = 0.0;
    m_ownShip.m_intXTE          = 0.0;
    // 【修复 3】重置LOS搜索状态，防止上次仿真的记忆污染本次首帧
    m_ownShip.m_lastClosestIdx  = -1;
    m_ownShip.m_lastXTE_valid   = false;
    m_ownShip.currentXTE        = 0.0;
    m_ownShip.measuredXTE       = 0.0;
    // 退化实验的状态也要复位，否则连跑多次会带上一轮的噪声与历史船位
    m_xteNoiseState             = 0.0;
    m_poseHistory.clear();

    // =========================================================
    // 🌟 核心修改 1：切换为上行航道 (Upstream Lane)
    // 上行航道的索引 0 节点在地理上正是下行航道的终端水域。
    // =========================================================
    const QVector<QPointF>& activeLane = m_laneUpstream;

    // 动态定位：将本船出生点安全设置在上行航道的初始深水区（顺延 50 个点）
    if (!activeLane.isEmpty()) {
        // 🌟 修改点 1：将顺延的 50 改为 0，让船舶直接在起点诞生！
        int spawnIdx = qMin(0, activeLane.size() - 2);
        if (spawnIdx < 0) spawnIdx = 0;

        QPointF startPoint = activeLane[spawnIdx];
        m_ownShip.X = startPoint.x();
        m_ownShip.Y = startPoint.y();

        // 🌟 核心修改 2：计算上行航道初始段的切线方向，作为逆流北上的绝对船首向
        if (activeLane.size() > spawnIdx + 1) {
            m_ownShip.C = calculateBearing(activeLane[spawnIdx], activeLane[spawnIdx + 1]);
        }

        qDebug() << "🚢 船舶已成功诞生于【上行逆流航道】的初始端水域！";
        qDebug() << "📍 墨卡托坐标：[" << startPoint.x() << "," << startPoint.y() << "]";
        qDebug() << "🧭 初始逆流相切航向 (Rad):" << m_ownShip.C;
    } else {
        qDebug() << "⚠️ 警告：未找到有效航道，请检查浮标提取逻辑。";
        double simStartLon = 119.3850;
        double simStartLat = 32.2150;
        double simStartX, simStartY;
        lonLat2Mercator(simStartLon, simStartLat, simStartX, simStartY);
        m_ownShip.X = simStartX;
        m_ownShip.Y = simStartY;
        m_ownShip.C = 105.0 * M_PI / 180.0;
    }

    // =========================================================
    // 🌟 核心修改 3：物理状态严格重构，消除逆流开局动量冲突
    // =========================================================
    m_ownShip.u = 0.0;           // 横向侧滑初速度绝对归零

    // 逆流前进动力配置：设定初始纵向推进速度为 13.0 节 (转换为 m/s)
    // 使得合速度矢量在诞生的第一帧就绝对重合于上行航道切线方向
    m_ownShip.v = 13.0 * 0.5144;
    m_ownShip.r = 0.0;           // 回转角速度归零
    m_ownShip.delta = 0.0;       // 实际打舵角度初始化归零

    // 将初始推力螺旋桨转速设定为健康的 105 RPM（转为每秒圈数）
    // 确保本船在出生的瞬间，舵叶表面就有强大的滑流（Slipstream）冲刷，提供充足的始发舵效
    m_ownShip.np = 105.0 / 60.0;

    m_timer->start();
    m_btnRun->setText("仿真中...");
}


// drawBuoys函数已移除，浮标绘制功能已整合到drawWaterways中

// 【新增算法】Catmull-Rom 样条插值
QVector<QPointF> MainWindow::generateSmoothCurve(const QVector<Buoy>& buoys, int segments)
{
    QVector<QPointF> result;
    if (buoys.size() < 2) return result;

    // 提取坐标点
    QVector<QPointF> points;
    for (const auto& b : buoys) {
        points.append(b.pos);
    }

    // 为了让首尾也能插值，我们在两端重复第一个和最后一个点
    // 原始: P0, P1, ..., Pn
    // 扩展: P0, P0, P1, ..., Pn, Pn
    QVector<QPointF> extPoints;
    extPoints.append(points.first()); // 补头
    extPoints.append(points);
    extPoints.append(points.last());  // 补尾

    // 遍历每一段 (P_i 到 P_i+1)
    // 实际点在 extPoints 中的索引是从 1 到 size-2
    for (int i = 1; i < extPoints.size() - 2; ++i) {
        QPointF p0 = extPoints[i - 1];
        QPointF p1 = extPoints[i];     // 当前段起点
        QPointF p2 = extPoints[i + 1]; // 当前段终点
        QPointF p3 = extPoints[i + 2];

        // 在 p1 和 p2 之间插入 segments 个点
        for (int t = 0; t < segments; ++t) {
            double t_val = (double)t / segments;
            double t2 = t_val * t_val;
            double t3 = t2 * t_val;

            // Catmull-Rom 公式
            double f0 = -0.5 * t3 + t2 - 0.5 * t_val;
            double f1 =  1.5 * t3 - 2.5 * t2 + 1.0;
            double f2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t_val;
            double f3 =  0.5 * t3 - 0.5 * t2;

            double x = p0.x() * f0 + p1.x() * f1 + p2.x() * f2 + p3.x() * f3;
            double y = p0.y() * f0 + p1.y() * f1 + p2.y() * f2 + p3.y() * f3;

            result.append(QPointF(x, y));
        }
    }

    // 别忘了把最后一个点加进去
    result.append(points.last());

    return result;
}

// 【新增算法】曲线重采样：将曲线重新划分为 targetCount 个等距点
// 这能让左右岸的点一一对应，从而计算中心线
QVector<QPointF> MainWindow::resamplePolyline(const QVector<QPointF>& points, int targetCount)
{
    QVector<QPointF> result;
    if (points.size() < 2 || targetCount < 2) return points;

    // 1. 计算曲线总长度
    QVector<double> dists; // 累积距离
    dists.append(0);
    double totalLength = 0;

    for (int i = 0; i < points.size() - 1; ++i) {
        double dx = points[i+1].x() - points[i].x();
        double dy = points[i+1].y() - points[i].y();
        double d = sqrt(dx*dx + dy*dy);
        totalLength += d;
        dists.append(totalLength);
    }

    // 2. 按固定步长插值
    double step = totalLength / (targetCount - 1);
    result.append(points.first()); // 第一个点

    int currentIdx = 0;
    for (int i = 1; i < targetCount - 1; ++i) {
        double targetDist = i * step;

        // 寻找 targetDist 落在哪个线段内 (currentIdx -> currentIdx+1)
        while (currentIdx < dists.size() - 1 && dists[currentIdx+1] < targetDist) {
            currentIdx++;
        }

        // 线性插值
        // 此时 targetDist 介于 dists[currentIdx] 和 dists[currentIdx+1] 之间
        double segmentStartDist = dists[currentIdx];
        double segmentEndDist = dists[currentIdx+1];
        double segmentLen = segmentEndDist - segmentStartDist;

        if (segmentLen > 1e-9) {
            double t = (targetDist - segmentStartDist) / segmentLen;
            QPointF p1 = points[currentIdx];
            QPointF p2 = points[currentIdx+1];
            double x = p1.x() + (p2.x() - p1.x()) * t;
            double y = p1.y() + (p2.y() - p1.y()) * t;
            result.append(QPointF(x, y));
        } else {
            result.append(points[currentIdx]);
        }
    }

    result.append(points.last()); // 最后一个点
    return result;
}

// 强制锁定坐标轴的 1:1 物理比例
void MainWindow::enforceAspectRatio()
{
    if (!m_aisPlot || m_aisPlot->height() == 0) return;

    // 1. 获取物理窗口的像素宽高比
    double widgetRatio = (double)m_aisPlot->width() / m_aisPlot->height();

    // 2. 获取当前代码设定的数据逻辑宽高比
    double xCenter = m_aisPlot->xAxis->range().center();
    double yCenter = m_aisPlot->yAxis->range().center();
    double xSize = m_aisPlot->xAxis->range().size();
    double ySize = m_aisPlot->yAxis->range().size();

    if (ySize == 0) return;
    double dataRatio = xSize / ySize;

    // 3. 谁短补谁，强制把数据范围扩充至与物理窗口比例一致
    if (widgetRatio > dataRatio) {
        // 窗口比数据更宽：放大 X 轴的数据范围来填满多余的水平像素
        double newXSize = ySize * widgetRatio;
        m_aisPlot->xAxis->setRange(xCenter - newXSize / 2.0, xCenter + newXSize / 2.0);
    } else if (widgetRatio < dataRatio) {
        // 窗口比数据更高：放大 Y 轴的数据范围来填满多余的垂直像素
        double newYSize = xSize / widgetRatio;
        m_aisPlot->yAxis->setRange(yCenter - newYSize / 2.0, yCenter + newYSize / 2.0);
    }
}

// mainwindow.cpp

void MainWindow::updateLabelsVisibility()
{
    // 获取当前 X 轴显示的范围宽度 (单位：米)
    double visibleRange = m_aisPlot->xAxis->range().size();

    // 【阈值设置】
    // 当视野范围小于 25000 米 (25km) 时，显示桥梁文字
    // 当视野范围小于 12000 米 (12km) 时，显示浮标编号 (浮标更密，需要拉得更近才显示)
    bool showBridge = (visibleRange < 25000.0);
    bool showBuoys = (visibleRange < 12000.0);

    // 1. 控制桥梁标签
    if (m_bridgeLabel) {
        m_bridgeLabel->setVisible(showBridge);
    }

    // 2. 控制浮标标签
    for (auto label : m_buoyLabels) {
        if (label) {
            label->setVisible(showBuoys);
        }
    }

    // 注意：这里不需要调用 replot，因为 rangeChanged 信号本身就是在重绘过程中触发的
    // 或者 Qt 的事件循环会自动处理下一次绘制
}

// 辅助函数：计算方位角 (0~360度, 北为0, 顺时针)
// 新增辅助函数：执行PID控制
void MainWindow::runPIDControl(Ship &ship, double targetHeading, float dt)
{
    // 1. 计算航向误差
    double err_k = targetHeading - ship.C;
    while (err_k > M_PI) err_k -= 2 * M_PI;
    while (err_k < -M_PI) err_k += 2 * M_PI;

    // =============================================================
    // 🌟 中庸而沉稳的 PID 参数 (找回内环弱积分)
    // =============================================================
    // 取自成员变量，批处理可用 --kp/--ti/--td 覆盖（默认值与原有取值一致）
    float Kp = static_cast<float>(m_gainKp); // 适中的比例：比3.5硬，比25.0稳
    float Ti = static_cast<float>(m_gainTi); // 积分时间：拉长积分时间，让它只负责缓慢消除静态误差
    float Td = static_cast<float>(m_gainTd); // 微分阻尼

    // 2. 带有严格死区的弱积分项（专治前馈不精准导致的稳态误差！）
    double err_deg = err_k * 180.0 / M_PI;
    if (fabs(err_deg) < 5.0) {
        // 只有当航向误差小于 5 度时，才允许积分去微调
        ship.pid_integratedErr += err_k * dt;
        // 【关键限制】积分项最多只能提供约 ±2° 的补偿舵角，绝不抢权！
        ship.pid_integratedErr = qBound(-0.3, ship.pid_integratedErr, 0.3);
    } else {
        ship.pid_integratedErr *= 0.95; // 误差大时迅速遗忘
    }

    // 3. 微分项 (强力滤波)
    double raw_derivative = (err_k - ship.pid_prevError) / dt;
    static double filtered_derivative = 0.0;
    filtered_derivative = 0.9 * filtered_derivative + 0.1 * raw_derivative;

    // 4. 计算 PID 反馈舵角
    double P_term = Kp * err_k;
    double I_term = (Kp / Ti) * ship.pid_integratedErr;
    double D_term = Kp * Td * filtered_derivative;

    // =========================================================
    // 🌟 核心杀手锏：FSI 低通滤波前馈
    // =========================================================
    double raw_yaw_moment = ship.current_flow_forces.N_flow;
    static double filtered_yaw_moment = 0.0;
    filtered_yaw_moment = 0.9 * filtered_yaw_moment + 0.1 * raw_yaw_moment;

    // 建议：可以先把 K_ff 稍微调小一点，防止过度补偿
    double K_ff = m_gainKff; // 批处理可用 --kff 覆盖
    double feedforward_rudder_deg = - (K_ff * filtered_yaw_moment);
    feedforward_rudder_deg = qBound(-12.0, feedforward_rudder_deg, 12.0);

    // =========================================================
    // 5. 最终舵角合成与机械限幅
    // =========================================================
    double rudderCommandDeg = (P_term + I_term + D_term) * 180.0 / M_PI + feedforward_rudder_deg;
    rudderCommandDeg = qBound(-35.0, rudderCommandDeg, 35.0);

    double allowedRate = 12.0; // 最大舵速 12度/秒
    double maxStepDelta = allowedRate * dt;
    double deltaRudder = rudderCommandDeg - ship.delta;
    deltaRudder = qBound(-maxStepDelta, deltaRudder, maxStepDelta);

    ship.delta += deltaRudder;
    ship.pid_prevError = err_k;
}

double MainWindow::calculateBearing(QPointF p1, QPointF p2)
{
    // 墨卡托坐标系中: Y轴向上(北), X轴向右(东)
    double dy = p2.y() - p1.y();
    double dx = p2.x() - p1.x();

    // atan2(x, y) 返回的是以Y轴为0，顺时针旋转的角度(弧度)
    // 这种用法正好符合航海习惯 (北=0, 东=90)
    double bearing = atan2(dx, dy);

    // 归一化到 0 ~ 2PI
    if (bearing < 0) bearing += 2 * M_PI;
    return bearing;
}

// 核心函数：纯粹的航线跟踪逻辑
// 新增辅助函数：计算带有ILOS和潮流补偿的LOS目标航向
// 核心函数：带有强防跳变机制的航线跟踪逻辑
double MainWindow::calculateLOSAimCourse(Ship &ship, bool isPrediction)
{
    const QVector<QPointF>& activeLane = m_laneUpstream;

    if (activeLane.size() < 10) return ship.C;
    QPointF shipPos(ship.X, ship.Y);

    // =================================================================
    // 🔍 第一阶段：超微距局部滑动窗口搜索 (从物理视野上彻底切断跳跃)
    // =================================================================
    double min_d_new = 1e18;
    int closestIdx = 0;
    bool doGlobalSearchFallback = false;

    if (ship.m_lastClosestIdx == -1) {
        doGlobalSearchFallback = true;
    } else {
        int searchStart = qMax(0, ship.m_lastClosestIdx - 5);
        int searchEnd = qMin((int)activeLane.size() - 2, ship.m_lastClosestIdx + 15);

        for (int i = searchStart; i <= searchEnd; ++i) {
            double d = qPow(shipPos.x() - activeLane[i].x(), 2) + qPow(shipPos.y() - activeLane[i].y(), 2);
            if (d < min_d_new) {
                min_d_new = d;
                closestIdx = i;
            }
        }

        double shipLen = (ship.L > 10.0) ? ship.L : 190.0;
        double suspiciousDistSq = qPow(8.0 * shipLen, 2);
        if (min_d_new > suspiciousDistSq) {
            doGlobalSearchFallback = true;
        }
    }

    if (doGlobalSearchFallback) {
        min_d_new = 1e18;
        for (int i = 0; i < activeLane.size() - 1; ++i) {
            double d = qPow(shipPos.x() - activeLane[i].x(), 2) + qPow(shipPos.y() - activeLane[i].y(), 2);
            if (d < min_d_new) {
                min_d_new = d;
                closestIdx = i;
            }
        }
    }

    ship.m_lastClosestIdx = closestIdx;

    // =================================================================
    // --- B. 计算xte (干干净净，再无干扰) ---
    // =================================================================
    double xte = calculateSignedDistance(shipPos, activeLane[closestIdx], activeLane[closestIdx+1]);

    if (!isPrediction) {
        ship.currentXTE = xte;          // 真值：只用于统计与日志，噪声不污染指标
        ship.m_lastXTE_valid = true;
        if (m_xteNoiseSigma > 0.0) {
            // 量测噪声取一阶相关（AR(1)，时间常数 60 s）。白噪声会被控制器直接平均掉、
            // 等于没加；GNSS/DGPS 的实际误差是慢漂移，相关噪声才有惩罚意义
            const double dt = 0.1, tau = 60.0;
            const double rho = std::exp(-dt / tau);
            m_xteNoiseState = rho * m_xteNoiseState
                            + std::sqrt(1.0 - rho * rho) * m_xteNoiseSigma * g_norm(g_noiseGen);
        }
    }
    xte += m_xteNoiseState;             // 控制器只看到带噪声的偏差
    if (!isPrediction) {
        // 对照组的 BaselineController 也从这里取量测值，保证四组看到的是同一个带噪声的偏差，
        // 否则加噪声只惩罚本文方法，对比就不公平了
        ship.measuredXTE = xte;
    }
    double currentPathAngle = calculateBearing(activeLane[closestIdx], activeLane[closestIdx+1]);

    // =========================================================
    // 【核心找回】：长弯道预瞄与前馈融合 (Anticipatory Curve Blending)
    // =========================================================
    double lookaheadDist = ship.L * 4.0;
    double accD = 0.0;
    int aimIdx = closestIdx;

    for (int i = closestIdx; i < activeLane.size() - 1; ++i) {
        double dx = activeLane[i].x() - activeLane[i+1].x();
        double dy = activeLane[i].y() - activeLane[i+1].y();
        accD += sqrt(dx*dx + dy*dy);
        if (accD >= lookaheadDist) {
            aimIdx = i;
            break;
        }
    }

    int aimIdxNext = qMin(aimIdx + 10, (int)activeLane.size() - 1);
    double futurePathAngle = calculateBearing(activeLane[aimIdx], activeLane[aimIdxNext]);

    double angleDiffCurve = futurePathAngle - currentPathAngle;
    while (angleDiffCurve > M_PI) angleDiffCurve -= 2 * M_PI;
    while (angleDiffCurve < -M_PI) angleDiffCurve += 2 * M_PI;

    const double kCurveBlendFactor = 0.0;
    currentPathAngle += kCurveBlendFactor * angleDiffCurve;

    while (currentPathAngle < 0) currentPathAngle += 2 * M_PI;
    while (currentPathAngle >= 2 * M_PI) currentPathAngle -= 2 * M_PI;

    // =========================================================
    // 前视距离计算 (LOS Delta)
    // =========================================================
    double maxCurvature = 0.0;
    double accumulatedDist = 0.0;
    double checkDistLimit = ship.L * 9.0;

    for (int k = closestIdx + 1; k < activeLane.size() - 1; ++k) {
        QPointF p1 = activeLane[k];
        QPointF p2 = activeLane[k+1];
        double segLen = sqrt(pow(p1.x()-p2.x(), 2) + pow(p1.y()-p2.y(), 2));

        if (segLen < 1.0) continue;
        accumulatedDist += segLen;
        if (accumulatedDist < 50.0) continue;
        if (accumulatedDist > checkDistLimit) break;

        double nextPathAngle = calculateBearing(p1, p2);
        double angleDiff = fabs(nextPathAngle - currentPathAngle);
        while (angleDiff > M_PI) angleDiff = 2*M_PI - angleDiff;

        double localCurvature = angleDiff / accumulatedDist;
        if (localCurvature > maxCurvature) maxCurvature = localCurvature;
    }

    double L = (ship.L > 10.0) ? ship.L : 190.0;
    double baseDelta = 5.0 * L;
    double minDelta = 2.0 * L;

    double curvatureFactor = 1.0 / (1.0 + 10.0 * maxCurvature * (3.0 * L));
    double Delta = baseDelta * curvatureFactor;
    Delta = qMax(Delta, minDelta);

    // =========================================================
    // 流场前馈：按船位处流速的航线法向分量反算所需蟹行角
    // =========================================================
    // 稳态下 LOS 只能提供 atan(-xte/Delta) 的航向偏置，横流需要的蟹行角必须由偏差“换取”，
    // 即 xte = Delta·tan(所需蟹行角)。Delta 为 5 倍船长时，12° 蟹行角对应约 170 m 稳态偏差。
    // 这里直接按流速三角形前馈该蟹行角，不再用偏差去换。
    double betaFlow = 0.0;
    {
        // 退化实验：前馈只拿到有误差的流场估计——按历史船位取流（滞后）、幅值乘系数、流向加偏差。
        // lag=0、gain=1、bias=0 时与完美信息完全等价。
        double sx = ship.X, sy = ship.Y, stime = m_totalSimTime;
        if (!isPrediction) {
            m_poseHistory.append(PoseSample{m_totalSimTime, ship.X, ship.Y});
            while (m_poseHistory.size() > 1 && m_poseHistory[1].t <= m_totalSimTime - m_ffLagSec) {
                m_poseHistory.removeFirst();
            }
        }
        if (!m_poseHistory.isEmpty()) {
            sx = m_poseHistory.first().x;
            sy = m_poseHistory.first().y;
            stime = m_poseHistory.first().t;
        }
        QPointF flowVec = m_bridgeEnv.computeAt(sx, sy, stime);
        if (m_ffGain != 1.0 || m_ffBiasRad != 0.0) {
            const double cb = cos(m_ffBiasRad), sb = sin(m_ffBiasRad);
            flowVec = QPointF(m_ffGain * (flowVec.x() * cb - flowVec.y() * sb),
                              m_ffGain * (flowVec.x() * sb + flowVec.y() * cb));
        }
        // 航线法向分量（右舷为正）
        double crossFlow = flowVec.x() * cos(currentPathAngle) - flowVec.y() * sin(currentPathAngle);
        double waterSpeed = qMax(1.0, sqrt((double)ship.u * ship.u + (double)ship.v * ship.v));
        betaFlow = asin(qBound(-0.5, -crossFlow / waterSpeed, 0.5)); // 限制在 ±30°
    }
    if (!isPrediction) {
        // 低通滤波，抑制流场局部起伏引起的舵角抖动
        m_filteredDrift = 0.9 * m_filteredDrift + 0.1 * betaFlow;
    }
    betaFlow = m_filteredDrift; // 预测时沿用当前滤波值，不改变滤波状态

    // =========================================================
    // 🔥 强化版 Lyapunov ILOS
    // =========================================================
    // 积分只负责补偿前馈的残差，增益与限幅按“折算到偏差上的补偿量”标定：
    // Ki_los × 限幅 = 0.1 × 400 = 40 m。原取值 0.01 × 600 = 6 m，相对百米级偏差等于没有积分
    double Ki_los = 0.1;

    if (!isPrediction) {
        if (fabs(xte) < (3.0 * Delta)) {
            double numerator = Delta * xte;
            double denominator = (Delta * Delta) + pow(xte + Ki_los * ship.m_intXTE, 2);

            if (fabs(xte) < 120.0) {
                ship.m_intXTE += (numerator / denominator) * 0.1;
            } else {
                qDebug() << "🛑 [XTE 过大] XTE=" << xte << "，拒绝更新积分防止污染！";
            }
        }

        ship.m_intXTE = qBound(-400.0, ship.m_intXTE, 400.0);
    }

    double beta_los = atan(-(xte + Ki_los * ship.m_intXTE) / Delta);

    double finalAimC = currentPathAngle + beta_los + betaFlow;

    if (finalAimC < 0) finalAimC += 2 * M_PI;
    if (finalAimC >= 2 * M_PI) finalAimC -= 2 * M_PI;

    return finalAimC;
}

void MainWindow::onBtnToggleFlowClicked()
{
    static int state = 0;

    if (state == 0) {
        m_bridgeEnv.setRecordingMode(true);
        m_btnToggleFlow->setText("正在录制流场 (点击停止)");
        m_btnToggleFlow->setStyleSheet("background-color: #FF0000; color: white;");
        state = 1;

        QMessageBox::information(this, "模式切换", "已开启【流场录制模式】。\n\n请点击\"开始仿真\"，系统将自动记录本船经过区域的流场数据。\n\n跑完一次后再次点击此按钮停止，下次仿真将自动使用记录的数据。");

    } else {
        m_bridgeEnv.setRecordingMode(false);
        m_btnToggleFlow->setText("流场已保存 (下一次自动回放)");
        m_btnToggleFlow->setStyleSheet("background-color: #2196F3; color: white;");
        state = 0;

        m_bridgeEnv.setPlaybackMode(true);

        QMessageBox::information(this, "录制完成", "流场数据已保存到 SecondFlow.db。\n\n下一次点击\"开始仿真\"时，系统将优先读取数据库中的流场数据（零计算负载）。");
    }
}

// 冗余函数已删除，改用 calculateLOSAimCourse 和 runPIDControl

void MainWindow::onBtnRecordFlowClicked() {
    if (m_isSimulating) {
        QMessageBox::warning(this, "警告", "仿真进行中，无法录制流场。\n\n请先暂停或停止仿真后再试。");
        return;
    }

    // 1. 获取 UI 复选框状态
    bool focusMode = m_chkPylonMode->isChecked();

    // 2. 设置流场对象状态
    m_bridgeEnv.setPylonFocusMode(focusMode);

    // 创建进度条对话框
    QString progressText = focusMode
        ? "正在进行高精度几何势场计算 (20m 网格)...\n这是一个高强度物理计算，请耐心等待 (约 1-3 分钟)。"
        : "正在进行全流域录制...";
    QProgressDialog progress(progressText, "取消", 0, 100, this);
    progress.setWindowTitle("流场录制中");
    progress.setWindowModality(Qt::WindowModal); // 模态，阻止用户点击其他地方
    progress.setMinimumDuration(0); // 立即显示
    progress.setValue(0);

    progress.setCancelButton(nullptr);
    progress.setWindowFlags(progress.windowFlags() & ~Qt::WindowCloseButtonHint);

    // 设置样式
    progress.setStyleSheet(
        "QProgressBar { border: 1px solid grey; border-radius: 5px; text-align: center; }"
        "QProgressBar::chunk { background-color: #9C27B0; width: 10px; margin: 0.5px; }"
    );

    // 【新增】: 真正的启动录制任务 (使用异步线程防止界面卡死)
    QtConcurrent::run([=, &progress]() {
        // 调用核心录制函数，并传入回调更新进度条
        m_bridgeEnv.recordStartingFlowField([&progress](int value) {
            // 在主线程更新 UI
            QMetaObject::invokeMethod(&progress, "setValue", Qt::QueuedConnection, Q_ARG(int, value));
        });

        // 录制完成，关闭进度条
        QMetaObject::invokeMethod(&progress, "close", Qt::QueuedConnection);
    });

    // 让进度条显示并运行
    progress.exec();
}
