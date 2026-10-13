// 无界面批处理：命令行带 --batch 时由 main.cpp 调用，按配置运行一次仿真并导出 CSV
#include "mainwindow.h"

void seedSimNoise(unsigned seed);            // 定义在 mainwindow.cpp
void Mercator2lonLat(double x, double y, double &lon, double &lat);   // 定义在 mainwindow.cpp
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <QtMath>
#include <random>

QString MainWindow::outputDir() const
{
    return m_outputDir.isEmpty() ? QCoreApplication::applicationDirPath() : m_outputDir;
}

int MainWindow::runBatch(const BatchConfig& config)
{
    m_batchMode = true;
    m_outputDir = config.outputDir;
    if (!m_outputDir.isEmpty() && !QDir().mkpath(m_outputDir)) {
        std::fprintf(stderr, "[batch] cannot create output directory: %s\n", qUtf8Printable(m_outputDir));
        return 4;
    }
    if (m_laneUpstream.size() < 10) {
        std::fprintf(stderr, "[batch] upstream lane not loaded, cannot simulate\n");
        return 3;
    }

    // 只导出计划航线（墨卡托坐标与经纬度），用于核算航线曲率和绘图
    if (!config.dumpLanePath.isEmpty()) {
        QFile lane(config.dumpLanePath);
        if (!lane.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            std::fprintf(stderr, "[batch] cannot write lane file: %s\n", qUtf8Printable(config.dumpLanePath));
            return 4;
        }
        QTextStream out(&lane);
        out << "index,x,y,lon,lat\n";
        for (int i = 0; i < m_laneUpstream.size(); ++i) {
            const QPointF &p = m_laneUpstream[i];
            const double mx = p.x() / MERC_SCALE, my = p.y() / MERC_SCALE;
            double lon = mx / MERC_R * 180.0 / M_PI;
            double lat = (2.0 * std::atan(std::exp(my / MERC_R)) - M_PI / 2.0) * 180.0 / M_PI;
            out << i << "," << QString::number(p.x(), 'f', 3) << "," << QString::number(p.y(), 'f', 3) << ","
                << QString::number(lon, 'f', 7) << "," << QString::number(lat, 'f', 7) << "\n";
        }
        std::printf("lane_points=%d written to %s\n", m_laneUpstream.size(), qUtf8Printable(config.dumpLanePath));
        return 0;
    }

    // 只导出重构流场网格（供论文插图使用），不做仿真
    if (!config.dumpFlowPath.isEmpty()) {
        double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
        for (const QPointF &p : m_laneUpstream) {
            minX = qMin(minX, p.x()); maxX = qMax(maxX, p.x());
            minY = qMin(minY, p.y()); maxY = qMax(maxY, p.y());
        }
        const double margin = 1200.0;
        minX -= margin; maxX += margin; minY -= margin; maxY += margin;

        FlowField::BackwaterScenario bwDump = {config.backwaterAmplitude > 0.0,
                                               config.backwaterAmplitude, config.backwaterPeriod,
                                               config.backwaterCelerity, 5.0,
                                               config.backwaterPhaseDeg * M_PI / 180.0};
        m_bridgeEnv.setBackwaterScenario(bwDump);

        QFile ff(config.dumpFlowPath);
        if (!ff.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            std::fprintf(stderr, "[batch] cannot write flow file: %s\n",
                         qUtf8Printable(config.dumpFlowPath));
            return 4;
        }
        QTextStream fout(&ff);
        fout << "x,y,lon,lat,u,v,speed_kn\n";
        const double step = config.flowStep > 1.0 ? config.flowStep : 120.0;
        int nPts = 0;
        for (double y = minY; y <= maxY; y += step) {
            for (double x = minX; x <= maxX; x += step) {
                QPointF fv = m_bridgeEnv.computeAt(x, y, config.flowTime);
                const double sp = std::sqrt(fv.x() * fv.x() + fv.y() * fv.y());
                if (sp <= 1e-6) continue;          // 掩码之外与桥墩内部不输出
                double lon = 0.0, lat = 0.0;
                Mercator2lonLat(x, y, lon, lat);
                fout << QString::number(x, 'f', 2) << "," << QString::number(y, 'f', 2) << ","
                     << QString::number(lon, 'f', 7) << "," << QString::number(lat, 'f', 7) << ","
                     << QString::number(fv.x(), 'f', 4) << "," << QString::number(fv.y(), 'f', 4) << ","
                     << QString::number(sp / 0.5144, 'f', 4) << "\n";
                ++nPts;
            }
        }
        std::printf("flow_points=%d step=%.0fm t=%.0fs written to %s\n", nPts, step,
                    config.flowTime, qUtf8Printable(config.dumpFlowPath));
        std::fflush(stdout);
        return 0;
    }

    // 沿计划航线导出八单元受流采样，用于核算流场空间密度与单元尺度是否匹配，不做仿真。
    // 采样点与 Ship::calculate_fluid_interaction 中的单元中心完全一致（纵向 ±3L/8、±L/8，横向 ±B/4）。
    if (!config.dumpCellsPath.isEmpty()) {
        FlowField::BackwaterScenario bwCells = {config.backwaterAmplitude > 0.0,
                                                config.backwaterAmplitude, config.backwaterPeriod,
                                                config.backwaterCelerity, 5.0,
                                                config.backwaterPhaseDeg * M_PI / 180.0};
        m_bridgeEnv.setBackwaterScenario(bwCells);

        QFile cf(config.dumpCellsPath);
        if (!cf.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            std::fprintf(stderr, "[batch] cannot write cell file: %s\n",
                         qUtf8Printable(config.dumpCellsPath));
            return 4;
        }
        const double L = m_ownShip.L > 10.0 ? m_ownShip.L : 160.0;
        const double B = m_ownShip.B > 3.0 ? m_ownShip.B : 29.0;
        const int nSplit = config.cellSplit >= 1 ? config.cellSplit : 4;
        QVector<double> xLoc(nSplit);
        for (int k = 0; k < nSplit; ++k)          // 首→尾，nSplit=4 时即 ±3L/8、±L/8
            xLoc[k] = L * (0.5 - (k + 0.5) / nSplit);
        const double yLoc[2] = {B / 4.0, -B / 4.0};

        QTextStream cout_(&cf);
        cout_ << "s_m,x,y,hdg_deg,depth_m,u_cg,v_cg";
        for (int i = 0; i < nSplit; ++i)
            for (int j = 0; j < 2; ++j)
                cout_ << QString(",u%1%2,v%1%2").arg(i + 1).arg(j + 1);
        cout_ << "\n";

        const double stationStep = config.flowStep > 1.0 ? config.flowStep : 120.0;
        double acc = 0.0, nextAt = 0.0;
        int nSta = 0;
        for (int k = 1; k < m_laneUpstream.size(); ++k) {
            const QPointF a = m_laneUpstream[k - 1], b = m_laneUpstream[k];
            const double dx = b.x() - a.x(), dy = b.y() - a.y();
            const double segLen = std::sqrt(dx * dx + dy * dy);
            if (segLen < 1e-6) continue;
            if (acc + segLen < nextAt) { acc += segLen; continue; }
            const double C = std::atan2(dx, dy);          // 与 Ship 的罗经约定一致
            const double sC = std::sin(C), cC = std::cos(C);
            while (nextAt <= acc + segLen) {
                const double f = (nextAt - acc) / segLen;
                const double X = a.x() + f * dx, Y = a.y() + f * dy;
                QPointF cg = m_bridgeEnv.computeAt(X, Y, config.flowTime);
                cout_ << QString::number(nextAt, 'f', 1) << ","
                      << QString::number(X, 'f', 2) << "," << QString::number(Y, 'f', 2) << ","
                      << QString::number(C * 180.0 / M_PI, 'f', 2) << ","
                      << QString::number(m_bridgeEnv.getDepthAt(X, Y), 'f', 2) << ","
                      << QString::number(cg.x(), 'f', 5) << "," << QString::number(cg.y(), 'f', 5);
                for (int i = 0; i < nSplit; ++i) {
                    for (int j = 0; j < 2; ++j) {
                        const double gx = X + xLoc[i] * sC + yLoc[j] * cC;
                        const double gy = Y + xLoc[i] * cC - yLoc[j] * sC;
                        QPointF fv = m_bridgeEnv.computeAt(gx, gy, config.flowTime);
                        cout_ << "," << QString::number(fv.x(), 'f', 5)
                              << "," << QString::number(fv.y(), 'f', 5);
                    }
                }
                cout_ << "\n";
                ++nSta;
                nextAt += stationStep;
            }
            acc += segLen;
        }
        std::printf("cell_stations=%d split=%dx2 step=%.0fm L=%.1f B=%.1f t=%.0fs written to %s\n",
                    nSta, nSplit, stationStep, L, B, config.flowTime,
                    qUtf8Printable(config.dumpCellsPath));
        std::fflush(stdout);
        return 0;
    }

    m_gainKp = config.kp;
    m_gainTi = config.ti;
    m_gainTd = config.td;
    m_gainKff = config.kff;
    m_baselineController->setProportionalGain(config.baseKp);

    // 退化实验参数（默认值即完美信息）
    m_baseRpm = config.baseRpm;
    m_ownRpm = config.ownRpm;
    m_ffGain = config.ffGain;
    m_ffBiasRad = config.ffBiasDeg * M_PI / 180.0;
    m_ffLagSec = std::max(0.0, config.ffLagSec);
    m_xteNoiseSigma = std::max(0.0, config.xteNoiseM);
    m_xteNoiseState = 0.0;
    m_poseHistory.clear();
    seedSimNoise(static_cast<unsigned>(config.noiseSeed));

    const int group = qBound(0, config.group, 3);
    const QStringList suffixes = {"Group1_Baseline", "Group2_ConvILOS", "Group3_Feedforward", "Group4_Proposed"};
    m_cmbControlMode->setCurrentIndex(group);
    m_ownShip.m_shallowWaterEnabled = config.shallowWater;
    m_ownShip.m_fsiSplit = config.fsiSplit;      // 流固耦合离散化收敛性核算用，默认 4 保持原结果

    // 移植导数的不确定性：对 16 项横向/首摇导数独立施加 ±frac 的乘性扰动。
    // R0 不参与——它由自航点标定而非移植，扰动它会把航速效应混进来。
    if (config.derivPerturb > 0.0) {
        std::mt19937 gen(static_cast<unsigned>(config.derivSeed));
        std::uniform_real_distribution<double> uni(-1.0, 1.0);
        MMGParams &p = m_ownShip.m_mmg;
        double *terms[] = {&p.Xvv, &p.Xvr, &p.Xrr, &p.Xvvvv,
                           &p.Yv, &p.Yr, &p.Yvvv, &p.Yvvr, &p.Yvrr, &p.Yrrr,
                           &p.Nv, &p.Nr, &p.Nvvv, &p.Nvvr, &p.Nvrr, &p.Nrrr};
        for (double *t : terms)
            *t *= (1.0 + config.derivPerturb * uni(gen));
        std::printf("[batch] derivatives perturbed by +/-%.0f%% (seed %d): Yv=%.4f Nv=%.4f Yr=%.4f Nr=%.4f\n",
                    100.0 * config.derivPerturb, config.derivSeed, p.Yv, p.Nv, p.Yr, p.Nr);
        std::fflush(stdout);
    }

    FlowField::BackwaterScenario backwater = {config.backwaterAmplitude > 0.0, config.backwaterAmplitude,
                                              config.backwaterPeriod, config.backwaterCelerity, 5.0,
                                              config.backwaterPhaseDeg * M_PI / 180.0};
    m_bridgeEnv.setBackwaterScenario(backwater);

    onBtnRecordValidationClicked(); // 开始录制 Log_*.csv 与 FSI_*.csv
    if (!m_isRecordingCSV) return 4;

    onBtnRunClicked();
    m_timer->stop(); // 不用定时器，逐帧直接调用 onTimerTick
    const double tStart = m_totalSimTime;

    // 统计量：每帧（5 个 0.1 s 子步）采样一次
    double maxAbsXte = 0.0, sumSqXte = 0.0, sumAbsRudderChange = 0.0;
    long frames = 0;
    double prevRudder = m_ownShip.delta;
    m_batchArrived = false;

    while (!m_batchArrived && m_totalSimTime - tStart < config.maxSimSeconds) {
        onTimerTick();
        if (m_batchArrived) break;
        double xte = std::fabs(m_ownShip.currentXTE);
        maxAbsXte = std::max(maxAbsXte, xte);
        sumSqXte += xte * xte;
        sumAbsRudderChange += std::fabs(m_ownShip.delta - prevRudder);
        prevRudder = m_ownShip.delta;
        ++frames;
    }
    if (m_isRecordingCSV) onBtnRecordValidationClicked(); // 达到最长仿真时间时停止录制

    QString summary = QString("group=%1 suffix=%2 arrived=%3 sim_time_s=%4 max_abs_xte_m=%5 rms_xte_m=%6 "
                              "sum_abs_rudder_change_deg=%7 shallow_water=%8 backwater_beta=%9")
                          .arg(group)
                          .arg(suffixes[group])
                          .arg(m_batchArrived ? 1 : 0)
                          .arg(m_totalSimTime - tStart, 0, 'f', 1)
                          .arg(maxAbsXte, 0, 'f', 2)
                          .arg(frames > 0 ? std::sqrt(sumSqXte / frames) : 0.0, 0, 'f', 2)
                          .arg(sumAbsRudderChange, 0, 'f', 1)
                          .arg(config.shallowWater ? 1 : 0)
                          .arg(config.backwaterAmplitude, 0, 'f', 3)
                      + QString(" backwater_celerity_mps=%1 backwater_period_s=%2")
                          .arg(config.backwaterCelerity, 0, 'f', 2)
                          .arg(config.backwaterPeriod, 0, 'f', 0)
                      + QString(" backwater_phase_deg=%1").arg(config.backwaterPhaseDeg, 0, 'f', 1)
                      + QString(" kp=%1 ti=%2 td=%3 kff=%4")
                          .arg(config.kp, 0, 'f', 3)
                          .arg(config.ti, 0, 'f', 3)
                          .arg(config.td, 0, 'f', 3)
                          .arg(config.kff, 0, 'e', 3)
                      + QString(" ff_gain=%1 ff_bias_deg=%2 ff_lag_s=%3 xte_noise_m=%4 noise_seed=%5")
                          .arg(config.ffGain, 0, 'f', 2)
                          .arg(config.ffBiasDeg, 0, 'f', 1)
                          .arg(config.ffLagSec, 0, 'f', 1)
                          .arg(config.xteNoiseM, 0, 'f', 2)
                          .arg(config.noiseSeed)
                      + QString(" base_rpm=%1 own_rpm=%2")
                          .arg(config.baseRpm, 0, 'f', 1)
                          .arg(config.ownRpm, 0, 'f', 1)
                      + QString(" fsi_split=%1x2").arg(config.fsiSplit)
                      + QString(" deriv_perturb=%1 deriv_seed=%2")
                          .arg(config.derivPerturb, 0, 'f', 3)
                          .arg(config.derivSeed);

    std::printf("%s\n", qUtf8Printable(summary));
    std::fflush(stdout);

    QFile f(outputDir() + "/Summary_" + suffixes[group] + ".txt");
    if (f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        // 用具名流并显式 flush/close：临时 QTextStream 的写法曾出现摘要文件为空的情况
        QTextStream ts(&f);
        ts << summary << "\n";
        ts.flush();
        f.close();
    } else {
        std::fprintf(stderr, "[batch] cannot write summary file: %s\n",
                     qUtf8Printable(f.fileName()));
    }
    return m_batchArrived ? 0 : 2;
}
