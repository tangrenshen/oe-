#ifndef MMG_KVLCC2_H
#define MMG_KVLCC2_H

#include "mmg_model.h"

// KVLCC2 参数，取自 Y&Y2015 表 1（主尺度）、表 3（仿真用系数）及第 5.1 节正文
enum class KVLCC2Scale { L7Model, FullScale };

// approachSpeed 用于把阻力系数 R0 换算到该尺度与航速（见 kvlcc2R0）
MMGParams kvlcc2Params(KVLCC2Scale scale, double approachSpeed);

// Schoenherr 摩擦阻力系数：0.242/sqrt(CF) = log10(Re·CF)
double schoenherrCF(double Re);

// 由 L3 船模约束试验值 R0'=0.022（表 2，Lpp=2.902 m，U=0.76 m/s）按 (1+k)·CF 换算到
// 目标船长与航速；忽略兴波阻力，与 Y&Y2015 的基本假设一致。nu 为运动黏度 (m²/s)
double kvlcc2R0(double Lpp, double U, double nu);

#endif // MMG_KVLCC2_H
