#include "baseline_controller.h"
#include <QtMath>
#include <QDebug>
#include <cmath>
#include <QtGlobal>

BaselineController::BaselineController() {}

void BaselineController::setPath(const QVector<QPointF>& path) {
    m_path = path;
}

double BaselineController::calculateBearing(QPointF p1, QPointF p2)
{
    double dy = p2.y() - p1.y();
    double dx = p2.x() - p1.x();
    double bearing = atan2(dx, dy);
    if (bearing < 0) bearing += 2 * M_PI;
    return bearing;
}

// 主执行入口：通过 ablationGroup 参数区分前三组实验
double BaselineController::execute(Ship& ship, float dt, int ablationGroup)
{
    // 1. 计算 LOS 目标航向 (外环制导)
    double targetHeading = calculateLOSAimCourse(ship, dt, ablationGroup);

    // 2. 执行底层航向控制 (内环执行)
    return runPIDControl(ship, targetHeading, dt, ablationGroup);
}

// ==============================================================
// 外环制导逻辑：控制横向偏差与目标航向
// ==============================================================
double BaselineController::calculateLOSAimCourse(Ship& ship, float dt, int ablationGroup)
{
    if (m_path.size() < 2) return ship.C;
    QPointF shipPos(ship.X, ship.Y);

    // 1. 寻找最近点 (粗略计算，复用船体已有的 currentXTE)
    double minDistSq = 1e18;
    int closestIdx = 0;
    for (int i = 0; i < m_path.size() - 1; ++i) {
        double d = qPow(shipPos.x() - m_path[i].x(), 2) + qPow(shipPos.y() - m_path[i].y(), 2);
        if (d < minDistSq) { minDistSq = d; closestIdx = i; }
    }

    // 取控制器可见的量测值（= 真值 + 量测噪声）。统计仍用 ship.currentXTE 真值
    double xte = ship.measuredXTE;
    
    // 基础视距 (传统固定视距：4倍船长)
    double L = (ship.L > 10.0) ? ship.L : 190.0;
    double Delta = 4.0 * L;

    double beta_los = 0.0;

    // ==========================================
    // 🔴 实验组差异化逻辑：外环 LOS
    // ==========================================
    if (ablationGroup == 2) {
        // 【Group 2: 传统常规 ILOS】 (暴露行业痛点：双积分冲突与 S 型画龙)
        // 传统积分增益：按“折算到偏差上的补偿量”标定，Ki × 限幅 = 0.1 × 400 = 40 m。
        // 原取值 0.005 × 1000 只有 5 m，相对百米级稳态偏差等于没有积分，
        // 这一组也就无法体现常规 ILOS 本身的积分饱和问题
        double Ki_los = 0.1;
        ship.m_intXTE += xte * dt; // 没有任何防饱和死区，粗暴积分！
        ship.m_intXTE = qBound(-400.0, ship.m_intXTE, 400.0);
        
        beta_los = atan(-(xte + Ki_los * ship.m_intXTE) / Delta);
    } else {
        // 【Group 1 (基准) & Group 3 (仅前馈)】: 纯基础 LOS，无积分 (暴露稳态误差死穴)
        ship.m_intXTE = 0.0; // 彻底清空积分历史
        beta_los = atan(-xte / Delta);
    }

    // 2. 计算航道基础走向：按累计弧长取前看点。
    // 航线点距并非固定 20 m（实际约 59 m），按下标估算会让前看距离偏差数倍
    int aimIdx = closestIdx;
    double accD = 0.0;
    while (aimIdx < m_path.size() - 1 && accD < Delta) {
        accD += std::hypot(m_path[aimIdx + 1].x() - m_path[aimIdx].x(),
                           m_path[aimIdx + 1].y() - m_path[aimIdx].y());
        ++aimIdx;
    }
    
    double pathAngle = calculateBearing(m_path[closestIdx], m_path[aimIdx]);
    
    // 3. 合成目标航向
    double finalAimC = pathAngle + beta_los;
    while (finalAimC < 0) finalAimC += 2 * M_PI;
    while (finalAimC >= 2 * M_PI) finalAimC -= 2 * M_PI;

    return finalAimC;
}

// ==============================================================
// 内环控制逻辑：底层打舵与前馈补偿
// ==============================================================
double BaselineController::runPIDControl(Ship& ship, double targetHeading, float dt, int ablationGroup)
{
    // 1. 误差计算
    double err_k = targetHeading - ship.C;
    while (err_k > M_PI) err_k -= 2*M_PI;
    while (err_k < -M_PI) err_k += 2*M_PI;

    double err_deg = err_k * 180.0 / M_PI;

    // 2. 微分项滤波 (通用，防止所有组别发生基础的高频震荡)
    double raw_derivative = (err_k - ship.pid_prevError) / dt;
    static double filtered_derivative = 0.0;
    filtered_derivative = 0.9 * filtered_derivative + 0.1 * raw_derivative;

    double P_term = 0.0, I_term = 0.0, D_term = 0.0;
    double feedforward_rudder = 0.0;

    // ==========================================
    // 🔴 实验组差异化逻辑：内环 PID 与前馈
    // ==========================================
    if (ablationGroup == 1 || ablationGroup == 2) {
        // 【Group 1 & Group 2】: 传统带积分 PID + 无前馈
        float Kp = (m_kpOverride > 0.0) ? static_cast<float>(m_kpOverride) : 15.0f; // 传统方法为了抗流，通常比例给得较大
        float Ti = 45.0;
        float Td = 3.0;

        // 传统内环积分逻辑 (在 Group 2 中会与外环 ILOS 发生灾难性的双积分打架)
        if (fabs(err_deg) < 10.0) {
            ship.pid_integratedErr += err_k * dt;
            ship.pid_integratedErr = qBound(-2.0, ship.pid_integratedErr, 2.0);
        } else {
            ship.pid_integratedErr *= 0.99;
        }

        P_term = Kp * err_k;
        I_term = (Kp / Ti) * ship.pid_integratedErr;
        D_term = Kp * Td * filtered_derivative;
        feedforward_rudder = 0.0; // 彻底关闭前馈

    } else if (ablationGroup == 3) {
        // 【Group 3】: 极简定常 PD + 8单元FSI前馈 (证明降阶与解耦的价值)
        float Kp = (m_kpOverride > 0.0) ? static_cast<float>(m_kpOverride) : 8.0f;  // 得益于前馈，Kp 可以降得很温和
        float Td = 3.0;
        ship.pid_integratedErr = 0.0; // 彻底剔除内环积分！

        P_term = Kp * err_k;
        I_term = 0.0;
        D_term = Kp * Td * filtered_derivative;

        // 获取并滤波 8 单元 FSI 力矩
        double raw_yaw_moment = ship.current_flow_forces.N_flow;
        static double filtered_yaw_moment = 0.0;
        filtered_yaw_moment = 0.9 * filtered_yaw_moment + 0.1 * raw_yaw_moment;

        // 引入前馈补偿
        double K_ff = 5.0e-6;
        feedforward_rudder = - (K_ff * filtered_yaw_moment);
        feedforward_rudder = qBound(-15.0, feedforward_rudder, 15.0);
    }

    // 3. 舵角合成与限幅
    double rudderCommandDeg = (P_term + I_term + D_term) * 180.0 / M_PI + feedforward_rudder;
    rudderCommandDeg = qBound(-35.0, rudderCommandDeg, 35.0);

    // 4. 机械舵速约束
    double allowedRate = 12.0;
    double maxStepDelta = allowedRate * dt;
    double deltaRudder = rudderCommandDeg - ship.delta;
    deltaRudder = qBound(-maxStepDelta, deltaRudder, maxStepDelta);

    ship.pid_prevError = err_k;
    
    // 返回真实的绝对物理舵角
    return ship.delta + deltaRudder;
}