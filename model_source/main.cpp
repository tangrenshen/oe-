#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QDebug>
#include <stdexcept>
#include <windows.h>
#include <stdio.h>
#include <cstdlib>
#include <cstring>

static void printUsage()
{
    std::printf(
        "Usage: xjp_release.exe [--batch options]\n"
        "  --batch            run one simulation without GUI and exit (without it the GUI opens as usual)\n"
        "  --group N          control group 0-3 (default 3, proposed method)\n"
        "  --out DIR          output directory for CSV and summary files (default: executable directory)\n"
        "  --max-time S       maximum simulated time in seconds (default 7200)\n"
        "  --backwater BETA   tidal backwater modulation amplitude 0-0.95 (default 0 = off)\n"
        "  --celerity C       tidal wave celerity in m/s (default 12)\n"
        "  --period S         modulation period in seconds (default M2 tide, 44712)\n"
        "  --phase DEG        tidal phase at departure in degrees (default 0); one transit covers\n"
        "                     only about 0.2 of the tidal period, so this selects the departure time\n"
        "  --no-shallow       disable shallow-water corrections\n"
        "  --dump-lane FILE   write the planned route (Mercator and lon/lat) to FILE and exit\n"
        "  --kp K --ti T --td D --kff K\n"
        "                     gains of the proposed controller (group 3); defaults 8, 50, 2, 3e-6\n"
        "  --base-kp K        inner-loop proportional gain of the reference groups 0-2\n"
        "                     (defaults 15 for groups 0 and 1, 8 for group 2)\n"
        "  --ff-gain G        scale factor on the current the feedforward sees (0.7/1.3 = -/+30%)\n"
        "  --ff-bias DEG      direction error of that current estimate, degrees\n"
        "  --ff-lag S         age of that estimate, seconds (sampled at the past ship position)\n"
        "  --xte-noise SIGMA  cross-track measurement noise, m, first-order correlated (tau = 60 s)\n"
        "  --noise-seed N     random seed of that noise (default 1)\n"
        "  --base-rpm R       commanded rpm of the reference groups 0-2 (default 100)\n"
        "  --own-rpm R        commanded rpm of the proposed controller (default 94.5)\n"
        "  --dump-flow FILE   sample the reconstructed flow field over the route bounding box\n"
        "                     into FILE (x, y, lon, lat, u, v, speed) and exit\n"
        "  --dump-cells FILE  sample the flow at the eight hull cell centres along the route\n"
        "                     into FILE (s, x, y, heading, depth, cg and per-cell u/v) and exit\n"
        "  --deriv-perturb F  multiply each of the 16 transplanted lateral/yaw derivatives by\n"
        "                     (1 + F*U(-1,1)); R0 is left alone (it is calibrated). 0 = off\n"
        "  --deriv-seed N     random seed of that perturbation (default 1)\n"
        "  --fsi-split N      longitudinal cell count of the fluid-structure coupling used in\n"
        "                     the simulation itself (default 4, i.e. the 4x2 eight-cell model)\n"
        "  --cell-split N     longitudinal cell count of that sampling (default 4, as used\n"
        "                     by the model); for grid-convergence checks\n"
        "  --flow-step M      grid spacing of that sampling in metres (default 120); with\n"
        "                     --dump-cells it is the station spacing along the route\n"
        "  --flow-time S      simulated time at which the field is sampled (default 0)\n"
        "Exit codes: 0 arrived, 2 time limit reached, 3 lane not loaded, 4 cannot write output, 1 bad arguments\n");
}

// 解析命令行。返回 -1：普通界面模式；0：批处理；1：参数错误；2：已打印帮助
static int parseArgs(int argc, char *argv[], MainWindow::BatchConfig &cfg)
{
    bool batch = false;
    for (int i = 1; i < argc; ++i) {
        const char *a = argv[i];
        auto number = [&](double &out) -> bool {
            if (i + 1 >= argc) return false;
            char *end = nullptr;
            out = std::strtod(argv[++i], &end);
            return end && *end == '\0';
        };
        double v = 0.0;
        if (!std::strcmp(a, "--batch")) {
            batch = true;
        } else if (!std::strcmp(a, "--help")) {
            return 2;
        } else if (!std::strcmp(a, "--group")) {
            if (!number(v) || v < 0 || v > 3) return 1;
            cfg.group = static_cast<int>(v);
        } else if (!std::strcmp(a, "--out")) {
            if (i + 1 >= argc) return 1;
            cfg.outputDir = QString::fromLocal8Bit(argv[++i]);
        } else if (!std::strcmp(a, "--max-time")) {
            if (!number(v) || v <= 0) return 1;
            cfg.maxSimSeconds = v;
        } else if (!std::strcmp(a, "--backwater")) {
            if (!number(v) || v < 0 || v > 0.95) return 1;
            cfg.backwaterAmplitude = v;
        } else if (!std::strcmp(a, "--celerity")) {
            if (!number(v) || v <= 0) return 1;
            cfg.backwaterCelerity = v;
        } else if (!std::strcmp(a, "--period")) {
            if (!number(v) || v <= 0) return 1;
            cfg.backwaterPeriod = v;
        } else if (!std::strcmp(a, "--no-shallow")) {
            cfg.shallowWater = false;
        } else if (!std::strcmp(a, "--dump-lane")) {
            if (i + 1 >= argc) return 1;
            cfg.dumpLanePath = QString::fromLocal8Bit(argv[++i]);
        } else if (!std::strcmp(a, "--phase")) {
            if (!number(v)) return 1;
            cfg.backwaterPhaseDeg = v;
        } else if (!std::strcmp(a, "--kp")) {
            if (!number(v) || v <= 0) return 1;
            cfg.kp = v;
        } else if (!std::strcmp(a, "--ti")) {
            if (!number(v) || v <= 0) return 1;
            cfg.ti = v;
        } else if (!std::strcmp(a, "--td")) {
            if (!number(v) || v < 0) return 1;
            cfg.td = v;
        } else if (!std::strcmp(a, "--kff")) {
            if (!number(v) || v < 0) return 1;
            cfg.kff = v;
        } else if (!std::strcmp(a, "--base-kp")) {
            if (!number(v) || v <= 0) return 1;
            cfg.baseKp = v;
        } else if (!std::strcmp(a, "--ff-gain")) {
            if (!number(v) || v < 0) return 1;
            cfg.ffGain = v;
        } else if (!std::strcmp(a, "--ff-bias")) {
            if (!number(v)) return 1;
            cfg.ffBiasDeg = v;
        } else if (!std::strcmp(a, "--ff-lag")) {
            if (!number(v) || v < 0) return 1;
            cfg.ffLagSec = v;
        } else if (!std::strcmp(a, "--xte-noise")) {
            if (!number(v) || v < 0) return 1;
            cfg.xteNoiseM = v;
        } else if (!std::strcmp(a, "--noise-seed")) {
            if (!number(v) || v < 0) return 1;
            cfg.noiseSeed = static_cast<int>(v);
        } else if (!std::strcmp(a, "--base-rpm")) {
            if (!number(v) || v <= 0) return 1;
            cfg.baseRpm = v;
        } else if (!std::strcmp(a, "--own-rpm")) {
            if (!number(v) || v <= 0) return 1;
            cfg.ownRpm = v;
        } else if (!std::strcmp(a, "--dump-flow")) {
            if (i + 1 >= argc) return 1;
            cfg.dumpFlowPath = QString::fromLocal8Bit(argv[++i]);
        } else if (!std::strcmp(a, "--dump-cells")) {
            if (i + 1 >= argc) return 1;
            cfg.dumpCellsPath = QString::fromLocal8Bit(argv[++i]);
        } else if (!std::strcmp(a, "--deriv-perturb")) {
            if (!number(v) || v < 0 || v > 0.5) return 1;
            cfg.derivPerturb = v;
        } else if (!std::strcmp(a, "--deriv-seed")) {
            if (!number(v) || v < 0) return 1;
            cfg.derivSeed = (int)v;
        } else if (!std::strcmp(a, "--fsi-split")) {
            if (!number(v) || v < 1 || v > 64) return 1;
            cfg.fsiSplit = (int)v;
        } else if (!std::strcmp(a, "--cell-split")) {
            if (!number(v) || v < 1 || v > 64) return 1;
            cfg.cellSplit = (int)v;
        } else if (!std::strcmp(a, "--flow-step")) {
            if (!number(v) || v <= 1.0) return 1;
            cfg.flowStep = v;
        } else if (!std::strcmp(a, "--flow-time")) {
            if (!number(v) || v < 0) return 1;
            cfg.flowTime = v;
        }
    }
    return batch ? 0 : -1;
}

int main(int argc, char *argv[])
{
    MainWindow::BatchConfig batchConfig;
    int parsed = parseArgs(argc, argv, batchConfig);
    if (parsed == 1 || parsed == 2) {
        printUsage();
        return parsed == 1 ? 1 : 0;
    }
    bool batch = (parsed == 0);

    if (batch) {
        // 批处理：输出保留在进程的标准输出/错误上，便于重定向；界面使用 offscreen 平台，不显示窗口
        qputenv("QT_QPA_PLATFORM", "offscreen");
    } else {
        // 创建控制台窗口以显示qDebug信息
        AllocConsole();
        freopen("CONOUT$", "w", stdout);
        freopen("CONOUT$", "w", stderr);
        freopen("CONIN$", "r", stdin);
    }

    qDebug() << "Starting application...";
    try {
        QApplication a(argc, argv);
        qDebug() << "QApplication created";

        MainWindow w;
        qDebug() << "MainWindow created";

        if (batch) {
            return w.runBatch(batchConfig);
        }

        w.show();
        qDebug() << "MainWindow shown";

        return a.exec();
    } catch (const std::exception &e) {
        qDebug() << "Exception in main:" << e.what();
        return -1;
    } catch (...) {
        qDebug() << "Unknown exception in main";
        return -1;
    }
}
