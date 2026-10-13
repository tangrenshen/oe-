#include "mmg_model.h"
#include <algorithm>
#include <cmath>

namespace {

const double PI = 3.14159265358979323846;

struct PropellerRudderForces {
    double XP, XR, YR, NR;
};

// 螺旋桨推力（式 8–16）与舵力（式 18–25）
PropellerRudderForces propellerAndRudder(const MMGParams &p, const MMGState &s, double U,
                                         double delta, double nP)
{
    PropellerRudderForces f = {0.0, 0.0, 0.0, 0.0};
    double beta = std::atan2(-s.vm, s.u);             // 船中漂角 β = tan⁻¹(−vm/u)
    double rp = (U > 1e-6) ? s.r * p.Lpp / U : 0.0;    // r'

    // 操纵运动中的伴流分数（式 15、16）
    double betaP = beta - p.xP * rp;
    double C2 = (betaP > 0.0) ? p.C2pos : p.C2neg;
    double oneMinusWP = (1.0 - p.wP0) * (1.0 + (1.0 - std::exp(-p.C1 * std::fabs(betaP))) * (C2 - 1.0));
    double uP = s.u * oneMinusWP;

    // 推力（式 8–11）
    double KT = 0.0;
    if (nP > 1e-6) {
        double JP = uP / (nP * p.DP);
        KT = p.k2 * JP * JP + p.k1 * JP + p.k0;
        f.XP = (1.0 - p.tP) * p.rho * nP * nP * std::pow(p.DP, 4) * KT;
    }

    // 舵的纵向来流（式 25）。将 uP·sqrt(1 + 8KT/(πJ²)) 改写为 sqrt(uP² + 8KT·n²·DP²/π)，
    // 两者等价，但改写后在 J→0（低速、倒车起步）时不会除零
    double eta = p.DP / p.HR;
    double slipstream = std::sqrt(std::max(0.0, uP * uP + 8.0 * KT * nP * nP * p.DP * p.DP / PI));
    double uRP = uP + p.kappa * (slipstream - uP);
    double uR = p.epsilon * std::sqrt(eta * uRP * uRP + (1.0 - eta) * uP * uP);

    // 舵的横向来流（式 23、24）
    double betaR = beta - p.lR * rp;
    double gammaR = (betaR < 0.0) ? p.gammaRneg : p.gammaRpos;
    double vR = U * gammaR * betaR;

    // 舵法向力（式 19–21）
    double alphaR = delta - std::atan2(vR, uR);
    double FN = 0.5 * p.rho * p.AR * (uR * uR + vR * vR) * p.falpha * std::sin(alphaR);

    // 有效舵力（式 18）
    f.XR = -(1.0 - p.tR) * FN * std::sin(delta);
    f.YR = -(1.0 + p.aH) * FN * std::cos(delta);
    f.NR = -(p.xR + p.aH * p.xH) * p.Lpp * FN * std::cos(delta);
    return f;
}

} // namespace

MMGAccel mmgAccelerations(const MMGParams &p, const MMGState &s, double delta, double nP,
                          const MMGExternal &ext)
{
    double U = std::sqrt(s.u * s.u + s.vm * s.vm);

    // 船体水动力（式 6、7）
    double XH = 0.0, YH = 0.0, NH = 0.0;
    if (U > 1e-6) {
        double v = s.vm / U, r = s.r * p.Lpp / U;
        double qF = 0.5 * p.rho * p.Lpp * p.d * U * U;
        double qN = qF * p.Lpp;
        XH = qF * (-p.R0 + p.Xvv * v * v + p.Xvr * v * r + p.Xrr * r * r + p.Xvvvv * v * v * v * v);
        YH = qF * (p.Yv * v + p.Yr * r + p.Yvvv * v * v * v + p.Yvvr * v * v * r
                   + p.Yvrr * v * r * r + p.Yrrr * r * r * r);
        NH = qN * (p.Nv * v + p.Nr * r + p.Nvvv * v * v * v + p.Nvvr * v * v * r
                   + p.Nvrr * v * r * r + p.Nrrr * r * r * r);
    }

    PropellerRudderForces pr = propellerAndRudder(p, s, U, delta, nP);
    double X = XH + pr.XR + pr.XP + ext.X;   // 式 5
    double Y = YH + pr.YR + ext.Y;
    double N = NH + pr.NR + ext.N;

    // 有量纲惯性量
    double qM = 0.5 * p.rho * p.Lpp * p.Lpp * p.d;
    double m = p.rho * p.nabla;
    double mx = p.mx * qM, my = p.my * qM;
    double Jz = p.Jz * qM * p.Lpp * p.Lpp;
    double IzG = m * std::pow(p.kzz * p.Lpp, 2);
    double xG = p.xG;

    // 运动方程（式 4）：纵荡单独求解；横荡与艏摇经 xG 耦合，联立 2×2 线性方程
    MMGAccel a;
    a.du = (X + (m + my) * s.vm * s.r + xG * m * s.r * s.r) / (m + mx);

    double a11 = m + my,  a12 = xG * m,                  b1 = Y - (m + mx) * s.u * s.r;
    double a21 = xG * m,  a22 = IzG + xG * xG * m + Jz,  b2 = N - xG * m * s.u * s.r;
    double det = a11 * a22 - a12 * a21;
    a.dvm = (b1 * a22 - a12 * b2) / det;
    a.dr  = (a11 * b2 - a21 * b1) / det;
    return a;
}

MMGParams mmgScaled(const MMGParams &p, double LppTarget)
{
    MMGParams s = p;
    double k = LppTarget / p.Lpp;
    s.Lpp = p.Lpp * k;
    s.B = p.B * k;
    s.d = p.d * k;
    s.xG = p.xG * k;
    s.DP = p.DP * k;
    s.HR = p.HR * k;
    s.AR = p.AR * k * k;
    s.nabla = p.nabla * k * k * k;
    return s;
}

double mmgSelfPropulsionRps(const MMGParams &p, double u0)
{
    MMGState s = {u0, 0.0, 0.0};
    MMGExternal none = {0.0, 0.0, 0.0};
    double lo = 0.0, hi = 50.0; // 推力随转速单调增加，二分求纵向合力为零的转速
    for (int i = 0; i < 100; ++i) {
        double mid = 0.5 * (lo + hi);
        if (mmgAccelerations(p, s, 0.0, mid, none).du > 0.0)
            hi = mid;
        else
            lo = mid;
    }
    return 0.5 * (lo + hi);
}

MMGParams mmgShallowWater(const MMGParams &deep, double H)
{
    const double L = deep.Lpp, B = deep.B, T = deep.d;
    if (H <= 0.0 || H / T >= 20.0) return deep;
    double hT = std::max(H / T, 1.2);

    MMGParams p = deep;
    double Cb = deep.nabla / (L * B * T);
    double HT = hT - 1.0;   // 表 7：H_T = H/T − 1
    double TH = 1.0 / hT;   // 表 7：T_H = T/H

    // Kijima 修正系数（表 7）
    double K0 = 1.0 + 0.0775 / std::pow(HT, 2) - 0.011 / std::pow(HT, 3) + 0.000068 / std::pow(HT, 5);
    double K1 = -0.0643 / HT + 0.0724 / std::pow(HT, 2) - 0.0113 / std::pow(HT, 3) + 0.0000767 / std::pow(HT, 5);
    double K2 = (B / T > 4.0) ? 0.137 / HT * B / T : 0.0342 / HT;
    double b1T = Cb * B * std::pow(1.0 + B / L, 2) / T; // B1/T

    double gv  = K0 + 2.0 / 3.0 * K1 * b1T + 8.0 / 15.0 * K2 * b1T * b1T;
    double gnr = K0 + 8.0 / 15.0 * K1 * b1T + 40.0 / 105.0 * K2 * b1T * b1T;
    double fyr = K0 + 2.0 / 5.0 * K1 * b1T + 24.0 / 105.0 * K2 * b1T * b1T;
    double fnv = K0 + K1 * b1T + K2 * b1T * b1T;
    double fnr = K0 + 0.5 * K1 * b1T + 1.0 / 3.0 * K2 * b1T * b1T;
    double fyv = 1.5 * fnr - 0.5;

    // 加速度导数（表 7 前四行）：m_y 对应 −Y_v̇，J_z 对应 −N_ṙ
    p.my *= gv;
    p.Jz *= gnr;

    // 船体水动力导数（表 7）
    p.Xvv *= fyv;  p.Xvvvv *= fyv; p.Xrr *= fnr; p.Xvr *= fyr;
    p.Yvvv *= fyv; p.Yvvr *= fyv;  p.Yvrr *= fyv; p.Yrrr *= gnr;
    p.Nv *= fnv;   p.Nvvv *= fyv;  p.Nrrr *= gnr;

    double cbBT = Cb * B / T, cbTB = Cb * T / B;
    p.Yv *= -TH + 1.0 / std::pow(1.0 - TH, 0.4 * cbBT);
    p.Nr *= -TH + 1.0 / std::pow(1.0 - TH, -14.28 * T / L + 1.5);

    double A1Yr = -5.5 * cbBT * cbBT + 26.0 * cbBT - 31.5;
    double A2Yr = 37.0 * cbBT * cbBT - 185.0 * cbBT + 230.0;
    double A3Yr = -38.0 * cbBT * cbBT + 197.0 * cbBT - 250.0;
    p.Yr *= 1.0 + A1Yr * TH + A2Yr * TH * TH + A3Yr * TH * TH * TH;

    double A1Nvvr = 91.0 * cbTB - 25.0, A2Nvvr = -515.0 * cbTB + 144.0, A3Nvvr = 508.0 * cbTB - 143.0;
    double A1Nvrr = 40.0 * cbBT - 88.0, A2Nvrr = -295.0 * cbBT + 645.0, A3Nvrr = 312.0 * cbBT - 678.0;
    p.Nvvr *= 1.0 + A1Nvvr * TH + A2Nvvr * TH * TH + A3Nvvr * TH * TH * TH;
    p.Nvrr *= 1.0 + A1Nvrr * TH + A2Nvrr * TH * TH + A3Nvrr * TH * TH * TH;

    // 伴流分数、推力减额、整流系数（式 26，原出处 Amin & Hasegawa 2010）
    double cbLT = Cb * L / T, cbLB = Cb * L / B, cbBL = Cb * B / L;
    p.wP0 *= 1.0 + (-4.932 + 0.6425 * cbLT - 0.0165 * cbLT * cbLT) * std::pow(TH, 1.655);
    double ctp = 1.0 + (29.495 - 14.089 * cbLB + 1.6486 * cbLB * cbLB)
                     * (1.0 / 250.0 - 7.0 / 200.0 * TH - 13.0 / 125.0 * TH * TH);
    p.tP = 1.0 - ctp * (1.0 - deep.tP);

    // 式 26 中两段整流系数公式的适用范围在排版上有歧义。这里按物理一致性取值：
    // T_H 不超过临界值时用 T_H^4.81 式（T_H→0 时比值趋于 1）；超过时用多项式式
    // （该式在 T_H→0 时会给出负的整流系数，只适用于较浅水域）。与开源实现 pymaneuvering 一致
    double gammaRatio = (TH <= -0.332 * T / B + 0.581)
        ? 1.0 + (-541.0 / 4.0 + 2432.95 * cbBL - 10137.7 * cbBL * cbBL) * std::pow(TH, 4.81)
        : 1.0 + (-5129.0 / 500.0 + 178.207 * cbBL - 2745.0 / 4.0 * cbBL * cbBL)
                * (-1927.0 / 500.0 + 2733.0 / 200.0 * TH - 2617.0 / 250.0 * TH * TH);
    p.gammaRpos *= gammaRatio;
    p.gammaRneg *= gammaRatio;

    // 粘性阻力（式 25，Raven 2016）：原式注明适用于 T/H < 0.5。
    // 极浅水（本文 h/T≈1.2 即 T/H≈0.83）已超出该范围，直接外推会明显低估阻力：
    // 未修正时本模型在 h/T=1.2、7 kn 下的自航转速为 0.596 r/s，而 SIMMAN 2014 自航模实测为
    // 46.2 rpm（0.77 r/s，FHR SIMMAN 2020 分报告 1 算例 2.2；该报告自身仿真为 0.805 r/s）。
    // 因此在适用范围内保留原式，超出部分乘一个随 (T/H − 0.5) 线性增长的标定系数，
    // 系数 kExcess 由上述实测自航点定出，并在 T/H = 0.5 处与原式连续。深水系数不受影响。
    double ravenFactor = 1.0 + 0.57 * std::pow(TH, 1.79);
    if (TH > 0.5) {
        const double kExcess = 2.56; // 由 h/T = 1.2、7 kn 处自航转速 0.77 r/s 标定（迭代确定）
        ravenFactor *= 1.0 + kExcess * (TH - 0.5);
    }
    p.R0 *= ravenFactor;
    return p;
}
