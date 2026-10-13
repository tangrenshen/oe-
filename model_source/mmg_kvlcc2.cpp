#include "mmg_kvlcc2.h"
#include <cmath>

double schoenherrCF(double Re)
{
    double cf = 0.003;
    for (int i = 0; i < 200; ++i) {
        double next = std::pow(0.242 / std::log10(Re * cf), 2);
        if (std::fabs(next - cf) < 1e-12)
            return next;
        cf = next;
    }
    return cf;
}

double kvlcc2R0(double Lpp, double U, double nu)
{
    const double R0_L3 = 0.022;                         // 表 2
    const double Re_L3 = 0.76 * 2.902 / 1.1386e-6;      // L3 船模，船池淡水 15°C
    return R0_L3 * schoenherrCF(U * Lpp / nu) / schoenherrCF(Re_L3);
}

MMGParams kvlcc2Params(KVLCC2Scale scale, double approachSpeed)
{
    MMGParams p;

    // 表 3：船体水动力导数
    p.Xvv = -0.040; p.Xvr = 0.002; p.Xrr = 0.011; p.Xvvvv = 0.771;
    p.Yv = -0.315;  p.Yr = 0.083;  p.Yvvv = -1.607; p.Yvvr = 0.379; p.Yvrr = -0.391; p.Yrrr = 0.008;
    p.Nv = -0.137;  p.Nr = -0.049; p.Nvvv = -0.030; p.Nvvr = -0.294; p.Nvrr = 0.055;  p.Nrrr = -0.013;

    // 表 3：附加质量（Motora 图谱估算）、螺旋桨与舵参数
    p.mx = 0.022; p.my = 0.223; p.Jz = 0.011;
    p.tP = 0.220;
    p.C1 = 2.0; p.C2pos = 1.6; p.C2neg = 1.1;
    p.tR = 0.387; p.aH = 0.312; p.xH = -0.464;
    p.gammaRneg = 0.395; p.gammaRpos = 0.640; p.lR = -0.710;
    p.epsilon = 1.09; p.kappa = 0.50; p.falpha = 2.747;

    // 第 5.1 节正文与符号表
    p.k0 = 0.2931; p.k1 = -0.2753; p.k2 = -0.1385;
    p.kzz = 0.25;
    p.xR = -0.5;

    // 原文表 1–3 未给出螺旋桨纵向位置，取接近几何位置的 −0.48（开源实现 shipmmg 取 −0.69）。
    // 敏感性检验（tools/mmg_validation/results）：两者的回转指标相差 ≤1.5%，超越角相差 ≤0.1°
    p.xP = -0.48;

    if (scale == KVLCC2Scale::L7Model) {           // 表 1，缩尺比 1/45.7
        p.Lpp = 7.00; p.B = 1.27; p.d = 0.46; p.nabla = 3.27; p.xG = 0.25;
        p.DP = 0.216; p.HR = 0.345; p.AR = 0.0539;
        p.rho = 1000.0;
        p.wP0 = 0.40;                               // 第 5.1 节：L7 船模
        p.R0 = kvlcc2R0(p.Lpp, approachSpeed, 1.1386e-6);
    } else {                                        // 表 1，实船
        p.Lpp = 320.0; p.B = 58.0; p.d = 20.8; p.nabla = 312600.0; p.xG = 11.2;
        p.DP = 9.86; p.HR = 15.80; p.AR = 112.5;
        p.rho = 1025.0;
        p.wP0 = 0.35;                               // 第 5.1 节：实船
        p.R0 = kvlcc2R0(p.Lpp, approachSpeed, 1.1883e-6);
    }
    return p;
}
