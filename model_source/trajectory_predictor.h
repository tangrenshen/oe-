#ifndef TRAJECTORY_PREDICTOR_H
#define TRAJECTORY_PREDICTOR_H

#include <QObject>
#include "ship.h" // 需要用到 Ship 类和 EnvData 结构体

// 前向声明 MainWindow，避免头文件循环包含
class MainWindow;

class TrajectoryPredictor : public QObject
{
    Q_OBJECT
public:
    // 构造函数传入 MainWindow 指针
    explicit TrajectoryPredictor(MainWindow* ctx, QObject *parent = nullptr);

    /**
     * @brief 执行航迹预测
     * @param currentShip 当前本船状态
     * @param predictTime 预测时长 (秒)，默认 240秒 (4分钟)
     * @return double     返回预测到的未来最大横向偏差 (Max Predicted XTE)
     */
    double predict(const Ship& currentShip, double predictTime = 240.0);

private:
    MainWindow* m_ctx; // 保存主窗口指针
    // 内部使用的仿真步长 (秒)
    // 步长越大计算越快但精度越低，2.0秒对于4分钟预测是很好的平衡
    const double m_simDt = 2.0;
};

#endif // TRAJECTORY_PREDICTOR_H
