#ifndef SHIP_H
#define SHIP_H
#include <QPointF>
#include <QVector>
#include <QtMath>
#include <functional> // 【新增】用于传递流场获取函数
#include "mmg_model.h"

// ================= 【修改】二维平面流固耦合受力结构体 =================
struct FlowForces {
    double X_flow; // 纵向附加扰动力 (Surge)
    double Y_flow; // 横向附加扰动力 (Sway)
    double N_flow; // 艏摇附加力矩 (Yaw)
};

// ================= 【修改】8单元受力日志结构体 =================
struct FSIElementData {
    double dX; // 单元纵向扰动力
    double dY; // 单元横向扰动力
};

// 定义环境数据结构，方便传递
struct EnvData {
    double windSpeed;       // 真风速 (m/s)
    double windDir;         // 真风向 (弧度, 北=0, 顺时针)
    double currentSpeed;    // 流速 (m/s)
    double currentDir;      // 流向 (弧度)
    double waterDepth;      // 实时水深 (m)
};

extern void  Mercator2lonLat(double x, double y, double &lon,double &lat);
extern void lonLat2Mercator(double lon, double lat, double &x,double &y);

enum MoveStatus
{
    Move=0,
    CollisionAvoidance,
    BackToPath_move,
    BackToPath_Turn
};

class ChannelInfomation
{
public:
    ChannelInfomation() {}
     ChannelInfomation(int _channelnum,bool _flag):channelnum(_channelnum),flag(_flag){}
    ~ChannelInfomation() {}
public:
    int channelnum;
    float channelcourse;
    bool flag;
};

class Ship
{
public:
    Ship();
    // 委托默认构造函数，保证船舶参数总是被初始化
    Ship(double _X, double _Y) : Ship() { X = _X; Y = _Y; }
    Ship(double _X,double _Y,float _S,float _C,float _L,ChannelInfomation _CI,int _PlannedNum) : Ship()
    { X = _X; Y = _Y; S = _S; C = _C; L = _L; CI = _CI; PlannedNum = _PlannedNum; }

    double X;
    double Y;
    float S;
    float C;
    float B;
    float L;
    float u;
    float v;
    float r;
    float C1, C2;
    ChannelInfomation CI;
    int PlannedNum;
    float delta;
    float np;
    bool isInsideBridgeArea; // 【新增】是否在桥区水域 (Suqiao Zone)
    bool isInChannel; // 【新增】是否在航道内

    // --- 核心物理参数 (新增) ---
    double m_mass;      // 质量 (kg)
    double m_mx, m_my;  // 附加质量
    double m_Izz, m_Jzz;// 转动惯量及附加惯量
    double m_draft;     // 吃水 (m)
    double m_Cb;        // 方形系数
    double m_airAreaT;  // 正投影受风面积 (m2)
    double m_airAreaL;  // 侧投影受风面积 (m2)

    // ================= 【修改】流场耦合受力变量与计算接口 =================
    FlowForces current_flow_forces = {0.0, 0.0, 0.0};

    // 【修改】8单元感知的等效平均流速
    double m_avg_flow_u;  // 随船坐标系下的纵向平均流速 (m/s)
    double m_avg_flow_v;  // 随船坐标系下的横向平均流速 (m/s)

    // 【修改】8单元受力日志数组 - 存放当前帧各单元的扰动力数据
    FSIElementData m_fsi_log[8];
    // 流固耦合的纵向等分数（横向固定 2）。默认 4 即原有的 4x2 = 8 单元，
    // 加大只用于离散化收敛性核算，见 --fsi-split
    int m_fsiSplit = 4;

    // 【修改】计算基于 8单元 的流固耦合受力 (法向/切向分解与等效差值法)
    // 传入参数降维：输入 大地坐标 x, y -> 返回 该点流速 u, v
    void calculate_fluid_interaction(std::function<void(double, double, double&, double&)> get_water_2d);

    // --- 原有简单模型 ---
    void baoxiang_move(float time);
    void bianxiang_move(float thea,float h, const EnvData& env, float targetRpm);
    void bianxiangANDbiansu_move(double thea,float h,float AimNp);



    // 辅助: 计算风力系数 (Isherwood 近似)
    void calculateWindCoeffs(double alphaR, double &Cx, double &Cy, double &Cn);

    // 辅助: 计算浅水修正系数 (Yoshimura)
    double calculateShallowEffect(double depth);

    // MMG 标准方法的水动力参数（见 mmg_model.h），在构造函数中初始化
    MMGParams m_mmg;
    bool m_shallowWaterEnabled = true; // 是否按当前位置水深做浅水修正
    // 把转速指令（按旧模型的转速-航速比例）换算为 MMG 模型的自航转速 (r/s)，按指令值缓存
    double rpsForLegacyRpm(double rpm);
    double m_rpsCacheRpm = -1.0;
    double m_rpsCacheValue = 0.0;

    // 【新增】每艘船独立的 PID 状态变量
    double pid_prevError = 0.0;      // 上一次的航向误差
    double pid_integratedErr = 0.0; // PID 积分项 (舵角补偿)
    double m_intXTE = 0.0;          // ILOS 算法的积分项 (偏航补偿)
    double currentXTE = 0.0;        // 当前横向偏差真值，每帧由 calculateLOSAimCourse 写入，只用于统计与日志
    double measuredXTE = 0.0;       // 控制器实际看到的偏差：真值 + 量测噪声（噪声为 0 时二者相同）
    int    m_lastClosestIdx = -1;   // LOS 局部窗口搜索：上一帧最近点索引记忆（-1=未初始化）
    bool   m_lastXTE_valid  = false;// XTE 连续性保护：标记上一帧 currentXTE 是否已有有效值

};

#endif // SHIP_H
