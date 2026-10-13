#ifndef HUIYU_H
#define HUIYU_H

#include "ship.h"
#include "types.h" // 确保包含 ChannelPoly 定义
#include <QVector>
#include <QPointF>

// 外部声明全局变量
extern QPointF beginpoint;
extern QVector<QPointF> route;
extern QVector<QVector<QPointF>> routearray;

// 会遇局面记录类
class Jumin
{
public :
    Jumin(){} 
    Jumin(int _state, int _CRITime, int _limitGxj, int  _Turn)
        : state(_state), CRITime(_CRITime), limitGxj(_limitGxj), Turn(_Turn){}
    ~Jumin(){} 

public :
    int  state;
    int  CRITime;
    int  limitGxj;
    int  Turn;
};

extern  QVector<Jumin> juminarray;

class HuiYu
{
public :
    HuiYu();

    // --- 核心决策函数 ---
    // 输入：本船、目标船列表、静态障碍物列表、意图航向(LOS)
    // 输出：TurnOS(转向标志), aimNp(建议转速), 返回值(建议改向角弧度)
    float analysis(const  Ship &ownship,
                   const  QVector<Ship> &targetship,
                   const QVector<ChannelPoly> &staticObstacles, // 新增：静态障碍物
                   int  &TurnOS,
                   float  &aimNp,
                   float intendedCourse) ;

    // --- 辅助计算函数 ---
    int meeting_state(const Ship &ownShip, const Ship &targetShip, int &CRITime, float &D, float intendedCourse) ;
    void calculate_XianJiao(const Ship &pO, const Ship &pT, float &D, float &Q1, float &Q2) ;
    // 修改：增加 proposedSpeed 参数，如果 proposedSpeed < 0，则默认使用 pO.S (当前速度)
    bool calculate_CRITime(const Ship &pO, const Ship &pT, int &CRITime, float intendedCourse, float proposedSpeed = -1.0f) ;
    float calculate_CRI_Value(int collisionTime); // 新增：计算危险度
    void calculate_channelinfomation(Ship &ownship);

    // --- 规则行为函数 ---
    void duiyu(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn) ;
    void crossingstandon(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn) ;
    void crossinggiveway(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn) ;
    void overtakestandon(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn) ;
    void overtakegiveway(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn) ;

    // --- 辅助函数 ---
    float normalizeAngle(float angle) const; // 角度归一化到 [0, 2π)
    bool isFollowingChannel(float shipCourse, float channelCourse) const; // 判断是否顺航道
    
    // 【新增】重置内部状态（在每次仿真开始时调用）
    void resetState() {
        m_activeOvertakingTargetIdx = -1;
        m_isOvertaking = false;
    }

private :
    // --- 新增：静态避碰检测 ---
    // 检查沿 checkCourse 航行 checkDist 距离内是否撞上 staticObstacles
    bool checkStaticCollision(const  Ship &ownship,
                              float  checkCourse,
                              float  checkDist,
                              const QVector<ChannelPoly> &obstacles) ;
                               
    // 【新增】追越状态锁
    int m_activeOvertakingTargetIdx = -1; // 当前锁定的追越目标索引
    bool m_isOvertaking = false;          // 是否处于追越动作中
};

#endif // HUIYU_H