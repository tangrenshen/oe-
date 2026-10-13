#ifndef BASELINE_CONTROLLER_H
#define BASELINE_CONTROLLER_H

#include "ship.h"
#include <QVector>
#include <QPointF>

// 消融实验控制器：封装前三组对照实验逻辑
// 通过 ablationGroup 参数区分不同实验组
class BaselineController
{
public:
    BaselineController();
    void setPath(const QVector<QPointF>& path);

    // 【修改点】：用 int ablationGroup 替代了原来的 bool 和 QPointF
    double execute(Ship& ship, float dt, int ablationGroup);

    // 覆盖对照组内环比例增益（批处理 --base-kp）。<= 0 表示沿用各组原有取值
    void setProportionalGain(double kp) { m_kpOverride = kp; }

private:
    double calculateBearing(QPointF p1, QPointF p2);

    // 【修改点】：传递实验组编号
    double calculateLOSAimCourse(Ship& ship, float dt, int ablationGroup);
    double runPIDControl(Ship& ship, double targetHeading, float dt, int ablationGroup);

    QVector<QPointF> m_path;
    double m_kpOverride = -1.0;
};

#endif // BASELINE_CONTROLLER_H