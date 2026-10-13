// 复现 Y&Y2015 第 5 节的 KVLCC2 操纵仿真，用原文表 4–7 检验 MMG 模块的实现：
//   与 "Cal." 列对比 -> 检验代码是否与原文一致（同一模型、同一系数）
//   与 "Exp." 列对比 -> 检验模型对自航模试验的预报精度
// 用法（工程根目录）: sh tools/mmg_validation/run.sh <结果标签>
#include "mmg_model.h"
#include "mmg_kvlcc2.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

const double PI = 3.14159265358979323846;
const double DEG = PI / 180.0;
const double KN = 0.514444;
const double LAMBDA_L7 = 45.7; // 表 1
double g_xPOverride = NAN;     // 命令行第 1 个参数可覆盖 xP（原文未给出），用于敏感性分析

struct Sim {
    MMGParams p;
    double nP = 0, dt = 0, rudderRate = 0; // r/s, s, rad/s
    double x = 0, y = 0, psi = 0, delta = 0;
    MMGState s = {0, 0, 0};

    void step(double deltaCmd)
    {
        double maxStep = rudderRate * dt;
        delta += std::max(-maxStep, std::min(maxStep, deltaCmd - delta));

        const MMGExternal none = {0, 0, 0};
        auto deriv = [&](const MMGState &st, double ps, double k[6]) {
            MMGAccel a = mmgAccelerations(p, st, delta, nP, none);
            k[0] = st.u * std::cos(ps) - st.vm * std::sin(ps);
            k[1] = st.u * std::sin(ps) + st.vm * std::cos(ps);
            k[2] = st.r;
            k[3] = a.du; k[4] = a.dvm; k[5] = a.dr;
        };
        double k1[6], k2[6], k3[6], k4[6];
        deriv(s, psi, k1);
        MMGState s2 = {s.u + 0.5 * dt * k1[3], s.vm + 0.5 * dt * k1[4], s.r + 0.5 * dt * k1[5]};
        deriv(s2, psi + 0.5 * dt * k1[2], k2);
        MMGState s3 = {s.u + 0.5 * dt * k2[3], s.vm + 0.5 * dt * k2[4], s.r + 0.5 * dt * k2[5]};
        deriv(s3, psi + 0.5 * dt * k2[2], k3);
        MMGState s4 = {s.u + dt * k3[3], s.vm + dt * k3[4], s.r + dt * k3[5]};
        deriv(s4, psi + dt * k3[2], k4);
        auto inc = [&](int i) { return dt * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0; };
        x += inc(0); y += inc(1); psi += inc(2);
        s.u += inc(3); s.vm += inc(4); s.r += inc(5);
    }
};

// 第 5.1 节仿真条件：实船进速 15.5 kn、实船舵速 1.76°/s（船模按 Froude 相似换算），
// 转速保持为进速下的自航转速，回转半径 0.25 Lpp（已在参数中）
Sim makeSim(KVLCC2Scale scale)
{
    bool model = (scale == KVLCC2Scale::L7Model);
    double timeScale = model ? std::sqrt(LAMBDA_L7) : 1.0;
    double U0 = 15.5 * KN / timeScale;
    Sim sim;
    sim.p = kvlcc2Params(scale, U0);
    if (!std::isnan(g_xPOverride)) sim.p.xP = g_xPOverride;
    sim.nP = mmgSelfPropulsionRps(sim.p, U0);
    sim.rudderRate = 1.76 * DEG * timeScale;
    sim.dt = sim.p.Lpp / U0 / 1000.0;
    sim.s = {U0, 0.0, 0.0};
    return sim;
}

struct Turn { double AD, DT; };

// 纵距 AD：航向改变 90° 时沿初始航向的距离；战术直径 DT：航向改变 180° 时的横向距离（均 / Lpp）
Turn turning(Sim sim, double deltaDeg)
{
    Turn t = {NAN, NAN};
    for (long k = 0; k < 5000000; ++k) {
        double px = sim.x, py = sim.y, pa = std::fabs(sim.psi);
        sim.step(deltaDeg * DEG);
        double a = std::fabs(sim.psi);
        auto at = [&](double v0, double v1, double target) { return v0 + (v1 - v0) * (target - pa) / (a - pa); };
        if (std::isnan(t.AD) && a >= PI / 2) t.AD = at(px, sim.x, PI / 2) / sim.p.Lpp;
        if (std::isnan(t.DT) && a >= PI) { t.DT = std::fabs(at(py, sim.y, PI)) / sim.p.Lpp; break; }
    }
    return t;
}

struct Zig { double os1, os2; };

// aDeg > 0：先右舵（10/10Z）；aDeg < 0：先左舵（−10/−10Z）
// switchDeg：换舵的航向阈值；为 NaN 时取 |aDeg|，即常规对称 Z 形（10/10、20/20）。
// 给出不同值即可做 FHR/MARIN 自航模所用的 10/2.5、20/5 这类非对称 Z 形。
Zig zigzag(Sim sim, double aDeg, double switchDeg = NAN)
{
    double sgn = aDeg > 0 ? 1.0 : -1.0, a = std::fabs(aDeg) * DEG;
    double c = (std::isnan(switchDeg) ? std::fabs(aDeg) : std::fabs(switchDeg)) * DEG;
    Zig z = {NAN, NAN};
    double cmd = sgn * a, peak = 0;
    int phase = 0;
    for (long k = 0; k < 5000000; ++k) {
        sim.step(cmd);
        double psi = sgn * sim.psi; // 换算为“先转方向为正”
        if (phase == 0) {
            if (psi >= c) { cmd = -sgn * a; phase = 1; peak = psi; }
        } else if (phase == 1) {
            peak = std::max(peak, psi);
            if (psi <= -c) { z.os1 = (peak - c) / DEG; cmd = sgn * a; phase = 2; peak = psi; }
        } else {
            peak = std::min(peak, psi);
            if (psi >= c) { z.os2 = (-peak - c) / DEG; break; }
        }
    }
    return z;
}

// 指定水动力参数与进速直接做 Z 形（供浅水算例用），返回超越角与航向极值
struct ZigFull { double os1, os2, hd1, hd2, nP; };

ZigFull zigzagAt(const MMGParams &p, double U0, double aDeg, double switchDeg)
{
    Sim sim;
    sim.p = p;
    sim.nP = mmgSelfPropulsionRps(p, U0);
    sim.rudderRate = 1.76 * DEG;
    sim.dt = p.Lpp / U0 / 1000.0;
    sim.s = {U0, 0.0, 0.0};
    Zig z = zigzag(sim, aDeg, switchDeg);
    double c = std::fabs(switchDeg);
    ZigFull f;
    f.os1 = z.os1; f.os2 = z.os2;
    f.hd1 = std::isnan(z.os1) ? NAN : z.os1 + c;   // 航向极值 = 超越角 + 换舵阈值
    f.hd2 = std::isnan(z.os2) ? NAN : z.os2 + c;
    f.nP = sim.nP;
    return f;
}

void row(const char *name, double ours, double cal, double exp)
{
    std::printf("%-22s %9.2f %9.2f %+8.1f%%", name, ours, cal, 100.0 * (ours - cal) / cal);
    if (std::isnan(exp)) std::printf("\n");
    else std::printf(" %9.2f %+8.1f%%\n", exp, 100.0 * (ours - exp) / exp);
}

void report(const char *title, KVLCC2Scale scale, const double cal[10], const double exp[10])
{
    Sim sim = makeSim(scale);
    std::printf("===== %s =====\n", title);
    std::printf("Lpp=%.2f m  U0=%.3f m/s  nP=%.3f rps  R0'=%.5f  xP'=%.3f  dt=%.4f s  rudder rate=%.2f deg/s\n",
                sim.p.Lpp, sim.s.u, sim.nP, sim.p.R0, sim.p.xP, sim.dt, sim.rudderRate / DEG);
    std::printf("%-22s %9s %9s %9s %9s %9s\n", "item", "this code", "Y&Y Cal", "vs Cal", "Y&Y Exp", "vs Exp");

    Turn tp = turning(sim, 35.0), tn = turning(sim, -35.0);
    Zig z10 = zigzag(sim, 10.0), z20 = zigzag(sim, 20.0), n10 = zigzag(sim, -10.0), n20 = zigzag(sim, -20.0);

    row("AD' (35 deg)", tp.AD, cal[0], exp[0]);
    row("DT' (35 deg)", tp.DT, cal[1], exp[1]);
    row("AD' (-35 deg)", tn.AD, cal[2], exp[2]);
    row("DT' (-35 deg)", tn.DT, cal[3], exp[3]);
    row("1st OSA 10/10Z", z10.os1, cal[4], exp[4]);
    row("2nd OSA 10/10Z", z10.os2, cal[5], exp[5]);
    row("1st OSA 20/20Z", z20.os1, cal[6], exp[6]);
    row("1st OSA -10/-10Z", n10.os1, cal[7], exp[7]);
    row("2nd OSA -10/-10Z", n10.os2, cal[8], exp[8]);
    row("1st OSA -20/-20Z", n20.os1, cal[9], exp[9]);
    std::printf("\n");
}

struct TurnDetail { double AD, DT, nP, VV0, rp, drift; };

// 回转指标，以及无量纲时间 t' = t·U0/Lpp 时的速度比、r'（按瞬时合速度无量纲化）和漂角绝对值
TurnDetail turningDetail(const MMGParams &p, double U0, double deltaDeg, double tPrime)
{
    Sim sim;
    sim.p = p;
    sim.nP = mmgSelfPropulsionRps(p, U0); // 该水深下达到 U0 的自航转速
    sim.rudderRate = 1.76 * DEG;           // 与 Y&Y2015 实船仿真相同
    sim.dt = p.Lpp / U0 / 1000.0;
    sim.s = {U0, 0.0, 0.0};

    TurnDetail d;
    Turn t = turning(sim, deltaDeg);
    d.AD = t.AD;
    d.DT = t.DT;
    d.nP = sim.nP;
    long steps = static_cast<long>(tPrime * p.Lpp / U0 / sim.dt);
    for (long k = 0; k < steps; ++k) sim.step(deltaDeg * DEG);
    double U = std::sqrt(sim.s.u * sim.s.u + sim.s.vm * sim.s.vm);
    d.VV0 = U / U0;
    d.rp = sim.s.r * p.Lpp / U;
    d.drift = std::fabs(std::atan2(-sim.s.vm, sim.s.u)) / DEG;
    return d;
}

// 浅水修正比随 H/T 的变化（KVLCC2 船型；修正比只与主尺度比例有关，与缩尺无关）
void printShallowFactors()
{
    MMGParams deep = kvlcc2Params(KVLCC2Scale::FullScale, 7.5 * KN);
    const double hTs[] = {1.2, 1.5, 2.0, 3.0, 4.0, 6.0, 10.0, 19.9, 20.0};
    std::printf("===== Shallow-water correction ratios vs H/T (KVLCC2 form, Taimuri 2020) =====\n");
    std::printf("%6s %6s %6s %6s %6s %6s %6s %7s %7s %7s %6s\n",
                "H/T", "Yv", "Yr", "Nv", "Nr", "my", "Jz", "wP0", "1-tP", "gammaR", "R0");
    for (double hT : hTs) {
        MMGParams s = mmgShallowWater(deep, hT * deep.d);
        std::printf("%6.1f %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f %7.3f %7.3f %7.3f %6.3f\n", hT,
                    s.Yv / deep.Yv, s.Yr / deep.Yr, s.Nv / deep.Nv, s.Nr / deep.Nr, s.my / deep.my, s.Jz / deep.Jz,
                    s.wP0 / deep.wP0, (1.0 - s.tP) / (1.0 - deep.tP), s.gammaRpos / deep.gammaRpos, s.R0 / deep.R0);
    }
    std::printf("\n");
}

// Taimuri et al. (2020) 图 11(b)、图 12：KVLCC2 在 H/T = 1.2、7.5 kn 下左舷 35° 回转
void reportShallow()
{
    // 7 kn、20% 富余水深（H/T = 1.2）与 FHR SIMMAN 2020 分报告 1 的 KVLCC2 算例 2.2 一致，
    // 该报告给出实船自航转速 0.805 r/s (48.3 rpm)、稳态舵角 1.9°，可作为定量对照
    const double U0 = 7.0 * KN;
    MMGParams deep = kvlcc2Params(KVLCC2Scale::FullScale, U0);
    MMGParams shallow = mmgShallowWater(deep, 1.2 * deep.d);

    std::printf("===== KVLCC2 full scale, 7 kn, 35 deg port turn: deep water vs H/T = 1.2 =====\n");
    std::printf("shallow/deep: Yv x%.2f  Yr x%.2f  Nv x%.2f  Nr x%.2f  my x%.2f  Jz x%.2f  R0 x%.2f\n",
                shallow.Yv / deep.Yv, shallow.Yr / deep.Yr, shallow.Nv / deep.Nv, shallow.Nr / deep.Nr,
                shallow.my / deep.my, shallow.Jz / deep.Jz, shallow.R0 / deep.R0);
    std::printf("              wP0 %.3f -> %.3f  tP %.3f -> %.3f  gammaR x%.2f\n",
                deep.wP0, shallow.wP0, deep.tP, shallow.tP, shallow.gammaRpos / deep.gammaRpos);
    std::printf("%-12s %7s %7s %9s %10s %9s %13s\n", "case", "AD'", "DT'", "nP[rps]", "V/V0@t'10", "r'@t'10", "|drift|@t'10");
    const struct { const char *name; const MMGParams *p; } cases[] = {{"deep", &deep}, {"H/T = 1.2", &shallow}};
    for (const auto &c : cases) {
        TurnDetail d = turningDetail(*c.p, U0, -35.0, 10.0);
        std::printf("%-12s %7.2f %7.2f %9.3f %10.2f %9.2f %11.1f deg\n", c.name, d.AD, d.DT, d.nP, d.VV0, d.rp, d.drift);
    }
    // 浅水 Z 形：FHR/MARIN 自航模在 20% ukc 下所用的 10/2.5 与 20/5
    std::printf("\n----- Zigzag at H/T = 1.2 (rudder/switch as used by FHR free-running tests) -----\n");
    std::printf("%-14s %9s %9s %9s %9s %9s\n", "case", "1st OS", "2nd OS", "max hd1", "max hd2", "nP[rps]");
    const struct { const char *name; double rud, sw; } zigs[] = {
        {"10/2.5", 10.0, 2.5}, {"20/5", 20.0, 5.0}, {"10/10", 10.0, 10.0}, {"20/20", 20.0, 20.0}};
    for (const auto &zc : zigs) {
        ZigFull f = zigzagAt(shallow, U0, zc.rud, zc.sw);
        std::printf("%-14s %8.2f° %8.2f° %8.2f° %8.2f° %9.3f\n",
                    zc.name, f.os1, f.os2, f.hd1, f.hd2, f.nP);
    }
    std::printf("Eloot et al. (OMAE2015-41912), KVLCC2 free-running at FHR, 20%% ukc, averages of ten runs:\n"
                "  10/2.5 zigzag: max heading 1 = 3.80 deg (rel. sd 15.3%%), max heading 2 = 8.78 deg (10.9%%)\n"
                "  20/5   zigzag: max heading 1 = 9.73 deg (3.5%%),  max heading 2 = 13.19 deg (3.1%%)\n"
                "  NOTE: that paper does not state the approach speed or propeller rate of the free-running runs\n"
                "  (only 'constant RPM'), so the comparison above is order-of-magnitude, not a matched condition.\n\n");
    std::printf("Taimuri 2020 Fig. 11b (H/T = 1.2, read from figure, approximate): DT' ~3.3, V/V0@t'10 ~0.72-0.75,\n"
                "  r'@t'10 ~ -0.40 to -0.45, drift@t'10 ~3-4 deg. Fig. 12: shallow water gives larger DT', less speed loss, smaller drift.\n"
                "FHR SIMMAN 2020 sub report 1, case 2.2 (KVLCC2, 20%% ukc, 7 kn full scale): self-propulsion 0.805 rps\n"
                "  (SIMMAN 2014 free-running measurement 46.2 rpm = 0.77 rps), steady rudder 1.9 deg; 35 deg turning\n"
                "  circle read from their Figure 9: AD' ~3.1, DT' ~4.5. Taimuri's Fig. 11b gives DT' ~3.3, so the two\n"
                "  published predictions for this benchmark differ by about 35%%.\n"
                "  The shallow-water resistance is calibrated on the 0.77 rps point (mmgShallowWater, kExcess), because\n"
                "  Raven's viscous correction is stated for T/H < 0.5 and h/T = 1.2 means T/H = 0.83. Before calibration\n"
                "  nP was 0.596 rps with AD' 4.44 and DT' 5.32; afterwards the turning indices follow without further\n"
                "  tuning, which indicates the earlier gap came from the resistance rather than the lateral derivatives.\n\n");
}

} // namespace

int main(int argc, char **argv)
{
    if (argc > 1) g_xPOverride = std::atof(argv[1]);

    // 表 4、表 5（L7 船模）
    const double l7Cal[10] = {3.31, 3.36, 3.26, 3.26, 5.2, 15.8, 10.9, 7.6, 10.2, 14.5};
    const double l7Exp[10] = {3.25, 3.34, 3.11, 3.08, 8.2, 21.9, 13.7, 9.5, 15.0, 15.1};
    // 表 6、表 7（实船，原文无试验值）
    const double fsCal[10] = {3.62, 3.71, 3.56, 3.59, 5.8, 20.5, 11.8, 8.8, 12.6, 16.1};
    const double fsExp[10] = {NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN};

    report("KVLCC2 L7 model (Y&Y2015 Tables 4-5)", KVLCC2Scale::L7Model, l7Cal, l7Exp);
    report("KVLCC2 full scale (Y&Y2015 Tables 6-7)", KVLCC2Scale::FullScale, fsCal, fsExp);
    printShallowFactors();
    reportShallow();
    return 0;
}
