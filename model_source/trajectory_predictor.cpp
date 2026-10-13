#include "trajectory_predictor.h"
#include "mainwindow.h"
#include <QtMath>
#include <QDebug>

TrajectoryPredictor::TrajectoryPredictor(MainWindow* ctx, QObject *parent) : QObject(parent), m_ctx(ctx)
{
}

double TrajectoryPredictor::predict(const Ship& currentShip, double predictTime)
{
    // 1. 克隆一艘"幽灵船"
    Ship ghost = currentShip;
    
    // 2. 初始化最大偏差记录
    double maxFutureXTE = 0.0;
    
    int steps = predictTime / m_simDt;

    // ========================================================
    // 【性能优化关键】将流场获取移到循环外部！
    // ========================================================
    // 不要在循环里重复计算流场，那太慢了。
    // 我们假设未来航程中的流场与当前位置相似（或者您可以每隔20步更新一次）
    QPointF flowVec = m_ctx->getFlowAt(ghost.X, ghost.Y, 0);
    
    EnvData env;
    env.currentSpeed = sqrt(flowVec.x()*flowVec.x() + flowVec.y()*flowVec.y());
    env.currentDir = atan2(flowVec.x(), flowVec.y());
    env.waterDepth = 20.0; // 默认深水，忽略浅水效应
    env.windSpeed = 0.0;
    env.windDir = 0.0;
    // ========================================================

    // 3. 开始快速推演循环
    for (int k = 0; k < steps; ++k) {
        // ========================================================
        // 【核心】直接调用 MainWindow 的算法！
        // ========================================================
        // 传入 true 表示这是预测模式，不要乱改全局变量
        double targetHeading = m_ctx->calculateLOSAimCourse(ghost, true);
        
        // PID 控制也直接复用！
        // runPIDControl 内部只修改 ship 对象的状态(ghost)，是安全的
        m_ctx->runPIDControl(ghost, targetHeading, m_simDt);
        
        // ========================================================
        
        // 记录最大偏差
        // 注意：calculateLOSAimCourse 已经计算并更新了 ghost.currentXTE
        if (fabs(ghost.currentXTE) > maxFutureXTE) {
            maxFutureXTE = fabs(ghost.currentXTE);
        }

        // 真实的运动学更新 (模拟物理移动)
        // 【性能优化】直接使用循环外获取的 env，不再调用 getFlowAt
        float rpm = 85.0; // 假设巡航转速
        ghost.bianxiang_move(ghost.delta, m_simDt, env, rpm);
    }

    return maxFutureXTE;
}
