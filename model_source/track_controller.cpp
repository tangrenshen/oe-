#include "track_controller.h"

TrackController::TrackController() : m_currentRudderDeg(0.0) { reset(); }

void TrackController::reset() {
    m_integratedXTE = 0.0;
    m_prevError = 0.0;
    m_intError = 0.0;
}

double TrackController::update(const QPointF& shipPos, double shipHeadingRad, double shipSpeedMs,
                               const QVector<QPointF>& path, const QPointF& flowVec, double dt) {
    if (path.size() < 2) return 0.0;

    // 1. 寻找最近点
    double minDistSq = 1e18;
    int idx = 0;
    for (int i = 0; i < path.size() - 1; ++i) {
        double d = qPow(shipPos.x()-path[i].x(),2) + qPow(shipPos.y()-path[i].y(),2);
        if (d < minDistSq) { minDistSq = d; idx = i; }
    }

    // 2. 计算横向偏差 XTE
    // 修改 getCrossTrackError 逻辑：让右偏为正，左偏为负
    double e = getCrossTrackError(shipPos, path[idx], path[idx+1]);

    // 3. ILOS 积分逻辑 (抗饱和处理)
    // 只有在偏差较小时开启积分，防止大角度转弯时锁死
    if (qAbs(e) < Delta) {
        m_integratedXTE += (Delta * e / (qPow(Delta, 2) + qPow(e + Ki_los * m_integratedXTE, 2))) * dt;
    } else {
        m_integratedXTE *= 0.95; // 偏差过大时缓慢重置积分
    }
    m_integratedXTE = qBound(-200.0, m_integratedXTE, 200.0);

    // 4. 潮流漂角前馈补偿
    double flowSpd = qSqrt(flowVec.x()*flowVec.x() + flowVec.y()*flowVec.y());
    double flowDir = qAtan2(flowVec.x(), flowVec.y());
    double driftComp = 0.0;
    if (shipSpeedMs > 0.5) {
        // 计算流向与航道切线的夹角
        double pathAngle = calculateBearing(path[idx], path[idx+1]);
        double crossFlow = flowSpd * qSin(flowDir - pathAngle);
        // 修正抵角计算：arcsin(垂直流速 / 船速)
        driftComp = qAsin(qBound(-0.7, crossFlow / shipSpeedMs, 0.7));
    }

    // 5. 计算目标航向
    double pathAngle = calculateBearing(path[idx], path[idx+1]);
    // 关键修正：符号逻辑。现在 e > 0 (右偏) 会产生负的修正角 (左转)
    double beta_los = qAtan(-(e + Ki_los * m_integratedXTE) / Delta);
    double targetHeading = pathAngle + beta_los - driftComp;

    // 6. PID 控制计算
    double err = targetHeading - shipHeadingRad;
    while (err > M_PI) err -= 2*M_PI;
    while (err < -M_PI) err += 2*M_PI;

    m_intError += err * dt;
    m_intError = qBound(-0.5, m_intError, 0.5);

    double diff = (err - m_prevError) / dt;
    m_prevError = err;

    // 死区限制：如果误差小于0.5度，则不进行微分修正
    if (qAbs(err) < (0.5 * M_PI / 180.0)) {
        diff = 0;
    }

    // 物理量级调整：降低Kp/Kd以减少振荡，增加系统稳定性
    double outputRudder = 1.2 * err + 0.02 * m_intError + 1.5 * diff;
    
    m_currentRudderDeg = outputRudder * 180.0 / M_PI;
    return qBound(-35.0, m_currentRudderDeg, 35.0);
}

// 修正 XTE 计算逻辑：确保 右偏为正，左偏为负
double TrackController::getCrossTrackError(QPointF p, QPointF p1, QPointF p2) {
    double dx = p2.x() - p1.x();
    double dy = p2.y() - p1.y();
    // 反转叉乘顺序：(dy * dx_p) - (dx * dy_p)
    // 这样在 X-东/Y-北 坐标系下，点在线段右侧时结果为正
    double cross = dy * (p.x() - p1.x()) - dx * (p.y() - p1.y());
    double len = qSqrt(dx*dx + dy*dy);
    return (len > 1e-6) ? (cross / len) : 0.0;
}

double TrackController::calculateBearing(QPointF p1, QPointF p2) {
    return qAtan2(p2.x() - p1.x(), p2.y() - p1.y());
}
