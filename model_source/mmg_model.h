#ifndef MMG_MODEL_H
#define MMG_MODEL_H

// MMG 标准方法，依据 Yasukawa H, Yoshimura Y. Introduction of MMG standard method for ship
// maneuvering predictions. J Mar Sci Technol, 2015, 20: 37-52（下文简称 Y&Y2015，式号同原文）。
//
// 船体坐标系原点在船中：x 指向船首，y 指向右舷，r 右转为正；舵角 delta 右舵为正（使船右转）。
// 无量纲化：力 (1/2)ρ Lpp d U²，力矩 (1/2)ρ Lpp² d U²，质量 (1/2)ρ Lpp² d，
// 惯性矩 (1/2)ρ Lpp⁴ d，速度 U，长度 Lpp。

struct MMGParams {
    // 主尺度（有量纲：m、m³、kg/m³）
    double Lpp, B, d, nabla;
    double xG;                   // 重心相对船中的纵向坐标，向首为正
    double DP;                   // 螺旋桨直径
    double HR;                   // 舵高
    double AR;                   // 舵面积（Mariner 舵取活动部分）
    double rho;

    // 惯性（无量纲）
    double mx, my, Jz;           // 附加质量、附加惯性矩
    double kzz;                  // 回转半径 / Lpp

    // 船体水动力导数（式 7）
    double R0, Xvv, Xvr, Xrr, Xvvvv;
    double Yv, Yr, Yvvv, Yvvr, Yvrr, Yrrr;
    double Nv, Nr, Nvvv, Nvvr, Nvrr, Nrrr;

    // 螺旋桨（式 8–16）
    double tP, wP0;
    double k0, k1, k2;           // KT = k2·J² + k1·J + k0
    double C1, C2pos, C2neg;     // 式 16，C2 在 βP>0 / βP<0 时取不同值
    double xP;                   // 螺旋桨纵向位置 / Lpp

    // 舵（式 18–25）
    double tR, aH;
    double xH, xR;               // 均为 / Lpp
    double gammaRpos, gammaRneg; // 整流系数，βR>0 / βR<0
    double lR;                   // ℓ'R
    double epsilon, kappa;
    double falpha;               // 舵升力斜率系数
};

// 船中处的对水运动状态：纵荡 u、横荡 vm（右舷正）、艏摇角速度 r（右转正）
struct MMGState { double u, vm, r; };
struct MMGAccel { double du, dvm, dr; };

// 船体/螺旋桨/舵以外的外力（风、非均匀流附加力等），N 为绕船中的力矩
struct MMGExternal { double X, Y, N; };

// 按式 4 求加速度；delta 单位 rad，nP 单位 r/s
MMGAccel mmgAccelerations(const MMGParams &p, const MMGState &s, double delta, double nP,
                          const MMGExternal &ext);

// 几何相似缩放：无量纲系数保持不变，主尺度按 Lpp 比例缩放。
// 注意：未修正阻力系数 R0 的雷诺数尺度效应。
MMGParams mmgScaled(const MMGParams &p, double LppTarget);

// 定常直航速度 u0 下推力与阻力平衡时的螺旋桨转速 (r/s)
double mmgSelfPropulsionRps(const MMGParams &p, double u0);

// 浅水修正，依据 Taimuri G, et al. A 6-DoF maneuvering model for the rapid estimation of
// hydrodynamic actions in deep and shallow waters. Ocean Eng, 2020, 218: 108103（附录 C 表 7、式 25、26）。
// 修正对象：船体水动力导数、横向附加质量与附加惯性矩、伴流分数、推力减额、整流系数、粘性阻力。
// H 为水深 (m)：H ≤ 0（水深未知）或 H/T ≥ 20 时按深水原样返回；H/T < 1.2 时按 1.2 计算（经验公式适用下限）
MMGParams mmgShallowWater(const MMGParams &deep, double H);

#endif // MMG_MODEL_H
