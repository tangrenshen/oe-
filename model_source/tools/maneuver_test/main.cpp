// 独立于 GUI 的船舶模型验证程序：
//   [1] 静水操纵性试验（回转、Z 形），对照 IMO MSC.137(76) 标准
//   [2] 8 单元流固耦合受力探针（横向剪切流 + 顺/逆流）
//   [3] BaselineController 弯道闭环跟踪（检查模型修正后控制器是否仍然稳定）
// 用法：在工程根目录执行 sh tools/maneuver_test/run.sh <结果标签>
#include "ship.h"
#include "baseline_controller.h"
#include <QVector>
#include <QPointF>
#include <algorithm>
#include <cmath>
#include <cstdio>

// ship.h 声明了这两个投影函数，ship.cpp 本身不用，这里给空实现以便独立链接
void Mercator2lonLat(double, double, double &, double &) {}
void lonLat2Mercator(double, double, double &, double &) {}

static const double H   = 0.1;    // 积分步长 (s)，与 onTimerTick 一致
static const double RPM = 100.0;  // 螺旋桨转速，与消融组 1~3 一致
static const double RAD2DEG = 180.0 / M_PI;
static const EnvData CALM = {0.0, 0.0, 0.0, 0.0, 1000.0}; // 无风无流、深水（H/T ≥ 20 不做浅水修正；IMO 操纵性标准针对深水）

static bool finiteShip(const Ship &s)
{
    return std::isfinite(s.X) && std::isfinite(s.Y) && std::isfinite(s.C)
        && std::isfinite(s.u) && std::isfinite(s.v) && std::isfinite(s.r);
}

// 定转速直航 2000 s 得到稳定进速，再把位置和航向归零
static Ship settledShip()
{
    Ship s;
    s.X = 0; s.Y = 0; s.C = 0;
    s.u = 0; s.v = 13.0f * 0.5144f; s.r = 0;
    s.delta = 0; s.S = s.v; s.np = RPM / 60.0;
    s.isInsideBridgeArea = false; s.isInChannel = true;
    for (int k = 0; k < 20000 && finiteShip(s); ++k) s.bianxiang_move(0.0f, H, CALM, RPM);
    s.X = 0; s.Y = 0; s.C = 0; s.r = 0; s.u = 0; s.delta = 0;
    s.S = std::fabs(s.v);
    return s;
}

struct TurnResult {
    double advance, transfer, tactical, steadyDia, speedRatio, dist10;
    double surgeRatio, sway, rDeg, driftDeg; // 回转结束时的状态，用于判断物理合理性
    bool plausible;
};

static TurnResult turning(const Ship &s0, float rudderDeg)
{
    TurnResult res = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, false};
    Ship s = s0;
    double dist = 0, sumDia = 0;
    int nDia = 0;
    for (int k = 0; k < 60000; ++k) { // 最长 6000 s
        double px = s.X, py = s.Y;
        s.bianxiang_move(rudderDeg, H, CALM, RPM);
        if (!finiteShip(s)) return res;
        dist += std::hypot(s.X - px, s.Y - py);
        double psi = std::fabs(s.C) * RAD2DEG;
        if (std::isnan(res.dist10) && psi >= 10.0) res.dist10 = dist;
        if (std::isnan(res.advance) && psi >= 90.0) { res.advance = s.Y; res.transfer = std::fabs(s.X); }
        if (std::isnan(res.tactical) && psi >= 180.0) res.tactical = std::fabs(s.X);
        if (psi >= 540.0 && std::fabs(s.r) > 1e-6) { sumDia += 2.0 * s.S / std::fabs(s.r); ++nDia; }
        if (psi >= 720.0) break;
    }
    if (nDia > 0) res.steadyDia = sumDia / nDia;
    res.speedRatio = s.S / s0.S;
    res.surgeRatio = s.v / s0.v;
    res.sway = s.u;
    res.rDeg = s.r * RAD2DEG;
    res.driftDeg = std::atan2(-s.u, s.v) * RAD2DEG; // 漂角：本程序约定 u 为横荡(右舷正)、v 为纵荡
    // 定常回转应当：数值有界、漂角远小于 45°、纵荡为正、合速度不增加
    res.plausible = finiteShip(s) && std::fabs(s.u) < 50.0 && std::fabs(s.v) < 50.0
                 && std::fabs(res.driftDeg) < 45.0 && res.surgeRatio > 0.2 && res.speedRatio < 1.02;
    return res;
}

struct ZigResult { double os1, os2; bool plausible; };

static ZigResult zigzag(const Ship &s0, double a)
{
    ZigResult res = {NAN, NAN, true};
    Ship probe = s0; // 先判断正舵角对应的转向
    for (int k = 0; k < 300; ++k) probe.bianxiang_move((float)a, H, CALM, RPM);
    double dir = (probe.C >= s0.C) ? 1.0 : -1.0;

    Ship s = s0;
    double rud = a, peak = 0;
    int phase = 0;
    for (int k = 0; k < 60000; ++k) {
        s.bianxiang_move((float)rud, H, CALM, RPM);
        if (!finiteShip(s)) { res.plausible = false; return res; }
        if (s.v <= 0 || std::fabs(std::atan2(-s.u, s.v)) * RAD2DEG > 45.0) res.plausible = false;
        double psi = dir * (s.C - s0.C) * RAD2DEG;
        if (phase == 0) {
            if (psi >= a) { rud = -a; phase = 1; peak = psi; }
        } else if (phase == 1) {
            peak = std::max(peak, psi);
            if (psi <= -a) { res.os1 = peak - a; rud = a; phase = 2; peak = psi; }
        } else {
            peak = std::min(peak, psi);
            if (psi >= a) { res.os2 = -peak - a; break; }
        }
    }
    return res;
}

static const char *verdict(double v, double limit, bool plausible)
{
    if (!plausible) return "INVALID"; // 运动状态不符合物理规律时，指标数值没有意义
    if (std::isnan(v)) return "n/a";
    return v <= limit ? "PASS" : "FAIL";
}

static void manoeuvring(const Ship &s0)
{
    double L = s0.L, LV = L / s0.S;
    TurnResult tp = turning(s0, 35.0f), tn = turning(s0, -35.0f), t10 = turning(s0, 10.0f);
    ZigResult z10 = zigzag(s0, 10.0), z20 = zigzag(s0, 20.0);

    double os1Lim = LV < 10 ? 10.0 : (LV >= 30 ? 20.0 : 5.0 + 0.5 * LV);
    double os2Lim = LV < 10 ? 25.0 : (LV >= 30 ? 40.0 : 17.5 + 0.75 * LV);

    std::printf("[1] Calm-water manoeuvring vs IMO MSC.137(76)\n");
    std::printf("%-30s %10s %10s %10s %6s\n", "item", "stbd/+", "port/-", "limit", "check");
    std::printf("%-30s %9.2fL %9.2fL %9.2fL %6s\n", "Advance (35 deg)", tp.advance / L, tn.advance / L, 4.5,
                verdict(std::max(tp.advance, tn.advance) / L, 4.5, tp.plausible && tn.plausible));
    std::printf("%-30s %9.2fL %9.2fL %9.2fL %6s\n", "Tactical diameter (35 deg)", tp.tactical / L, tn.tactical / L, 5.0,
                verdict(std::max(tp.tactical, tn.tactical) / L, 5.0, tp.plausible && tn.plausible));
    std::printf("%-30s %9.2fL %9.2fL %10s\n", "Transfer (35 deg)", tp.transfer / L, tn.transfer / L, "-");
    std::printf("%-30s %9.2fL %9.2fL %10s\n", "Steady turning dia (35 deg)", tp.steadyDia / L, tn.steadyDia / L, "-");
    std::printf("%-30s %10.2f %10.2f %10s\n", "Total speed ratio in turn", tp.speedRatio, tn.speedRatio, "-");
    std::printf("%-30s %10.2f %10.2f %10s\n", "Surge ratio in turn", tp.surgeRatio, tn.surgeRatio, "-");
    std::printf("%-30s %8.2fm/s %8.2fm/s %10s\n", "Sway velocity in turn", tp.sway, tn.sway, "-");
    std::printf("%-30s %7.3fdeg/s %6.3fdeg/s %8s\n", "Yaw rate in turn", tp.rDeg, tn.rDeg, "-");
    std::printf("%-30s %7.1fdeg %7.1fdeg %10s\n", "Drift angle in turn", tp.driftDeg, tn.driftDeg, "-");
    std::printf("%-30s %10s %10s %10s\n", "Physically plausible turn", tp.plausible ? "yes" : "NO", tn.plausible ? "yes" : "NO", "-");
    std::printf("%-30s %9.2fL %10s %9.2fL %6s\n", "Initial turning (10 deg)", t10.dist10 / L, "", 2.5,
                verdict(t10.dist10 / L, 2.5, t10.plausible));
    std::printf("%-30s %7.1fdeg %10s %7.1fdeg %6s\n", "10/10 zigzag 1st overshoot", z10.os1, "", os1Lim, verdict(z10.os1, os1Lim, z10.plausible));
    std::printf("%-30s %7.1fdeg %10s %7.1fdeg %6s\n", "10/10 zigzag 2nd overshoot", z10.os2, "", os2Lim, verdict(z10.os2, os2Lim, z10.plausible));
    std::printf("%-30s %7.1fdeg %10s %7.1fdeg %6s\n", "20/20 zigzag 1st overshoot", z20.os1, "", 25.0, verdict(z20.os1, 25.0, z20.plausible));
}

static void fsiProbe()
{
    // 船首向北、对水纵荡 6.7 m/s。流场 = 均匀部分 (Uc, Vm) + 沿船长变化的横流 A*y/(L/2) + 跨船宽变化的纵向流 G*x/(B/2)
    // 物理上均匀流只引起漂移，不应改变非均匀附加力：同组算例的结果应当相同
    struct Case { const char *name; double Uc, Vm, A, G; };
    const Case cases[] = {
        {"A: uniform following 1.3", 0.0, 1.3, 0.0, 0.0},
        {"B: crossflow shear only", 0.0, 0.0, 0.4, 0.0},
        {"B: + uniform crossflow +0.3", 0.3, 0.0, 0.4, 0.0},
        {"B: + uniform crossflow -0.3", -0.3, 0.0, 0.4, 0.0},
        {"C: beam shear + following", 0.0, 1.3, 0.0, 0.3},
        {"C: beam shear + opposing", 0.0, -1.3, 0.0, 0.3},
    };
    std::printf("\n[2] 8-element FSI probe (heading N, surge 6.7 m/s; results within a group should match)\n");
    std::printf("%-30s %12s %12s %14s\n", "case", "X_flow[kN]", "Y_flow[kN]", "N_flow[MN.m]");
    for (const Case &c : cases) {
        Ship s;
        s.X = 0; s.Y = 0; s.C = 0; s.u = 0; s.v = 6.7f; s.r = 0;
        double halfL = s.L / 2.0, halfB = s.B / 2.0;
        s.calculate_fluid_interaction([&](double x, double y, double &uw, double &vw) {
            uw = c.Uc + c.A * y / halfL;
            vw = c.Vm + c.G * x / halfB;
        });
        std::printf("%-30s %12.2f %12.2f %14.3f\n", c.name,
                    s.current_flow_forces.X_flow / 1e3, s.current_flow_forces.Y_flow / 1e3,
                    s.current_flow_forces.N_flow / 1e6);
    }
}

// 3000 m 直线 -> R=2000 m 右转 60 度圆弧 -> 4000 m 直线，点距 20 m（与 BaselineController 前看步长假设一致）
static QVector<QPointF> bendPath()
{
    QVector<QPointF> p;
    const double ds = 20.0, R = 2000.0, turn = 60.0 * M_PI / 180.0;
    for (double y = 0; y < 3000.0; y += ds) p.append(QPointF(0, y));
    int nArc = (int)(R * turn / ds);
    for (int i = 0; i <= nArc; ++i) {
        double th = turn * i / nArc;
        p.append(QPointF(R - R * std::cos(th), 3000.0 + R * std::sin(th)));
    }
    QPointF e = p.last();
    for (double d = ds; d <= 4000.0; d += ds)
        p.append(QPointF(e.x() + d * std::sin(turn), e.y() + d * std::cos(turn)));
    return p;
}

static void closedLoop(const Ship &s0)
{
    const QVector<QPointF> path = bendPath();
    std::printf("\n[3] Closed-loop bend tracking (BaselineController, R=2000 m, 60 deg bend, calm water)\n");
    std::printf("%-6s %10s %10s %12s %12s %10s\n", "group", "maxXTE[m]", "rmsXTE[m]", "maxRud[deg]", "sum|dRud|", "result");
    for (int g = 1; g <= 3; ++g) {
        BaselineController ctrl;
        ctrl.setPath(path);
        Ship s = s0;
        s.pid_integratedErr = 0; s.pid_prevError = 0; s.m_intXTE = 0;
        double maxX = 0, sumSq = 0, maxR = 0, sumDR = 0;
        long n = 0;
        const char *status = "timeout";
        for (int k = 0; k < 36000; ++k) {
            int best = 0;
            double bestD = 1e18;
            for (int i = 0; i < path.size() - 1; ++i) {
                double d = std::pow(s.X - path[i].x(), 2) + std::pow(s.Y - path[i].y(), 2);
                if (d < bestD) { bestD = d; best = i; }
            }
            // 与 MainWindow::calculateSignedDistance 相同的符号约定：右舷一侧为正
            QPointF p1 = path[best], p2 = path[best + 1];
            double dxl = p2.x() - p1.x(), dyl = p2.y() - p1.y();
            double cross = dxl * (s.Y - p1.y()) - dyl * (s.X - p1.x());
            double xte = std::fabs(cross) / std::hypot(dxl, dyl);
            s.currentXTE = (cross > 0) ? -xte : xte;

            double prev = s.delta;
            s.delta = (float)ctrl.execute(s, H, g);
            s.bianxiang_move(s.delta, H, CALM, RPM);
            if (!finiteShip(s)) { status = "NaN"; break; }
            maxX = std::max(maxX, xte); sumSq += xte * xte; ++n;
            maxR = std::max(maxR, (double)std::fabs(s.delta));
            sumDR += std::fabs(s.delta - prev);
            if (xte > 1000.0) { status = "diverged"; break; }
            if (best >= path.size() - 3) { status = "arrived"; break; }
        }
        std::printf("%-6d %10.1f %10.1f %12.1f %12.1f %10s\n", g, maxX,
                    n ? std::sqrt(sumSq / n) : NAN, maxR, sumDR, status);
    }
}

int main()
{
    Ship s0 = settledShip();
    if (!finiteShip(s0)) { std::printf("model diverged while settling\n"); return 1; }
    std::printf("Ship L=%.2f m  B=%.2f m  T=%.2f m  RPM=%.0f  dt=%.2f s\n", s0.L, s0.B, s0.m_draft, RPM, H);
    std::printf("Settled approach speed %.2f kn   L/V = %.1f s\n\n", s0.S / 0.5144, s0.L / s0.S);
    manoeuvring(s0);
    fsiProbe();
    closedLoop(s0);
    return 0;
}
