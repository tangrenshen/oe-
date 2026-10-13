#include "ship.h"
#include <vector>
#include "mmg_kvlcc2.h"
#include <math.h>
#include <QDebug>
#include <functional>

#define Pi 3.14

namespace {

const double KNOT = 0.514444;

// 内河本船：KVLCC2 按几何相似缩放到 Lpp=160 m（缩尺比 2，d=10.4 m），
// 在 12.5 m 深水航道中 h/T≈1.2。无量纲系数均取自 Y&Y2015（见 mmg_kvlcc2.cpp）
const double INLAND_LPP = 160.0;
const double DESIGN_SPEED_KN = 13.0;
const double FRESH_WATER_RHO = 1000.0;
const double FRESH_WATER_NU = 1.1386e-6; // 15°C

// 旧船舶模型在 100 rpm 定速直航时约为 13.15 kn（tools/maneuver_test/results/0_baseline.txt）。
// 主程序与避碰模块中的转速指令都按这一比例解释为目标航速
const double LEGACY_KNOTS_PER_RPM = 13.15 / 100.0;

MMGParams inlandShipParams()
{
    double U = DESIGN_SPEED_KN * KNOT;
    MMGParams p = mmgScaled(kvlcc2Params(KVLCC2Scale::FullScale, U), INLAND_LPP);
    p.rho = FRESH_WATER_RHO;
    p.R0 = kvlcc2R0(p.Lpp, U, FRESH_WATER_NU);
    return p;
}

} // namespace

Ship::Ship()
{
    // 初始化PID状态变量
    pid_prevError = 0.0;
    pid_integratedErr = 0.0;
    m_intXTE = 0.0;
    np = 85.0 / 60.0;

    X = 0.0; Y = 0.0; C = 0.0; S = 0.0;
    u = 0.0; v = 0.0; r = 0.0; delta = 0.0;
    C1 = C2 = 0.0;
    isInsideBridgeArea = false;
    isInChannel = false;
    m_avg_flow_u = 0.0;
    m_avg_flow_v = 0.0;

    // 船舶水动力参数与主尺度（8 单元流固耦合同样使用 L、B、m_draft）
    m_mmg = inlandShipParams();
    this->L = m_mmg.Lpp;
    this->B = m_mmg.B;
    this->m_draft = m_mmg.d;
}

double Ship::rpsForLegacyRpm(double rpm)
{
    if (rpm <= 0.0) return 0.0;
    if (rpm != m_rpsCacheRpm) {
        m_rpsCacheRpm = rpm;
        m_rpsCacheValue = mmgSelfPropulsionRps(m_mmg, rpm * LEGACY_KNOTS_PER_RPM * KNOT);
    }
    return m_rpsCacheValue;
}

void Ship::baoxiang_move(float time)
{
    float dis = S * time;
    X += dis * sin(C);
    Y += dis * cos(C);
    this->r = 0;
    this->delta = 0;
    C1 = C2 = 0;
}

// 核心运动函数：MMG 标准方法（Y&Y2015）+ 风力 + 8 单元非均匀流附加力 + 水流漂移
// thea：目标舵角 (deg，右舵为正)；h：步长 (s)；targetRpm：转速指令（按旧模型比例解释为目标航速）
void Ship::bianxiang_move(float thea, float h, const EnvData& env, float targetRpm)
{
    // === A. 舵机速率限制 ===
    float maxRatePerSec = this->isInsideBridgeArea ? 10.5f : 25.0f;
    float maxRudderRate = maxRatePerSec * h;
    float diff = thea - this->delta;
    if (diff > maxRudderRate) {
        this->delta += maxRudderRate;
    } else if (diff < -maxRudderRate) {
        this->delta -= maxRudderRate;
    } else {
        this->delta = thea;
    }
    double delta = this->delta * M_PI / 180.0;

    // === B. 螺旋桨转速 ===
    double nP = rpsForLegacyRpm(targetRpm);
    this->np = nP;

    // === C. 外力：风 + 8 单元非均匀流附加力（calculate_fluid_interaction 在每帧调用前算好）===
    double rho_a = 1.225;
    double Cx, Cy, Cn;
    calculateWindCoeffs(env.windDir - this->C, Cx, Cy, Cn);
    double V_wind_sq = env.windSpeed * env.windSpeed;
    double At = this->B * 15.0;
    double Al = this->L * 15.0;
    MMGExternal ext;
    ext.X = 0.5 * rho_a * At * V_wind_sq * Cx + current_flow_forces.X_flow;
    ext.Y = 0.5 * rho_a * Al * V_wind_sq * Cy + current_flow_forces.Y_flow;
    ext.N = 0.5 * rho_a * Al * this->L * V_wind_sq * Cn + current_flow_forces.N_flow;

    // 均匀流引起的漂移（大地坐标：x 向东、y 向北）
    double dx_current = env.currentSpeed * sin(env.currentDir);
    double dy_current = env.currentSpeed * cos(env.currentDir);

    // === D. 浅水修正 ===
    // Y&Y2015 系数为深水值，按当前位置水深修正（水深未知或足够深时不修正，见 mmgShallowWater）
    MMGParams hydro = m_shallowWaterEnabled ? mmgShallowWater(m_mmg, env.waterDepth) : m_mmg;

    // === E. RK4 积分 ===
    // 本类约定 v 为纵荡、u 为横荡（右舷正），对应 MMG 状态 (u_mmg = v, vm = u, r)
    struct Deriv { double x, y, c; MMGAccel a; };
    auto deriv = [&](double surge, double sway, double yawRate, double heading) -> Deriv {
        Deriv d;
        d.x = surge * sin(heading) + sway * cos(heading) + dx_current;
        d.y = surge * cos(heading) - sway * sin(heading) + dy_current;
        d.c = yawRate;
        MMGState s = {surge, sway, yawRate};
        d.a = mmgAccelerations(hydro, s, delta, nP, ext);
        return d;
    };

    double v0 = this->v, u0 = this->u, r0 = this->r, C0 = this->C;
    Deriv k1 = deriv(v0, u0, r0, C0);
    Deriv k2 = deriv(v0 + 0.5 * h * k1.a.du, u0 + 0.5 * h * k1.a.dvm, r0 + 0.5 * h * k1.a.dr, C0 + 0.5 * h * k1.c);
    Deriv k3 = deriv(v0 + 0.5 * h * k2.a.du, u0 + 0.5 * h * k2.a.dvm, r0 + 0.5 * h * k2.a.dr, C0 + 0.5 * h * k2.c);
    Deriv k4 = deriv(v0 + h * k3.a.du, u0 + h * k3.a.dvm, r0 + h * k3.a.dr, C0 + h * k3.c);

    this->X += h * (k1.x + 2 * k2.x + 2 * k3.x + k4.x) / 6.0;
    this->Y += h * (k1.y + 2 * k2.y + 2 * k3.y + k4.y) / 6.0;
    this->C  = C0 + h * (k1.c + 2 * k2.c + 2 * k3.c + k4.c) / 6.0;
    this->v  = v0 + h * (k1.a.du + 2 * k2.a.du + 2 * k3.a.du + k4.a.du) / 6.0;
    this->u  = u0 + h * (k1.a.dvm + 2 * k2.a.dvm + 2 * k3.a.dvm + k4.a.dvm) / 6.0;
    this->r  = r0 + h * (k1.a.dr + 2 * k2.a.dr + 2 * k3.a.dr + k4.a.dr) / 6.0;
    this->S  = sqrt(this->u * this->u + this->v * this->v);
}

void Ship::calculateWindCoeffs(double alphaR, double &Cx, double &Cy, double &Cn)
{
    Cx = -0.8 * cos(alphaR);
    Cy = 0.9 * sin(alphaR);
    Cn = 0.2 * sin(2.0 * alphaR);
}

double Ship::calculateShallowEffect(double depth)
{
    if (m_draft <= 0) m_draft = 10.3;
    if (depth <= 0.1) return 1.0;
    double h_T = depth / m_draft;
    if (h_T >= 5.0) return 1.0;
    return 1.0 + 2.0 * pow((1.0 / h_T), 2);
}

// =========================================================================
void Ship::calculate_fluid_interaction(std::function<void(double, double, double&, double&)> get_water_2d)
{
    if (this->L <= 0 || this->B <= 0) return; // 有了构造函数初始化，这里就不会被拦截了！

    double draft = (m_draft > 0) ? m_draft : 10.3;

    const int nSplit = (m_fsiSplit >= 1) ? m_fsiSplit : 4;
    const int nElem = 2 * nSplit;

    struct ElementState {
        double x_local, y_local;
        double u_w, v_w;
    };
    std::vector<ElementState> elements(nElem);

    std::vector<double> x_loc(nSplit);
    for (int k = 0; k < nSplit; ++k)      // 首→尾；nSplit = 4 时即 ±3L/8、±L/8
        x_loc[k] = this->L * (0.5 - (k + 0.5) / nSplit);
    double y_loc[2] = { this->B/4.0, -this->B/4.0 };

    double sum_u_w = 0.0, sum_v_w = 0.0;

    // 船舶 u、v 是对水速度，均匀流已由 bianxiang_move 中的漂移项计入（取重心处流速），
    // 因此各单元只计入相对重心处来流的差值
    double U_cg = 0.0, V_cg = 0.0;
    get_water_2d(this->X, this->Y, U_cg, V_cg);
    double ref_v_w = U_cg * sin(this->C) + V_cg * cos(this->C); // 纵向
    double ref_u_w = U_cg * cos(this->C) - V_cg * sin(this->C); // 横向
    int idx = 0;

    for (int i = 0; i < nSplit; i++) {
        for (int j = 0; j < 2; j++) {
            double global_X = this->X + x_loc[i] * sin(this->C) + y_loc[j] * cos(this->C);
            double global_Y = this->Y + x_loc[i] * cos(this->C) - y_loc[j] * sin(this->C);

            double U_earth = 0.0, V_earth = 0.0;
            get_water_2d(global_X, global_Y, U_earth, V_earth);

            // 🌟 核心修复 4：坐标投影逻辑彻底重构！
            double v_w = U_earth * sin(this->C) + V_earth * cos(this->C); // 纵向(Surge)
            double u_w = U_earth * cos(this->C) - V_earth * sin(this->C); // 横向(Sway)

            elements[idx] = {x_loc[i], y_loc[j], u_w, v_w};
            sum_u_w += u_w;
            sum_v_w += v_w;
            idx++;
        }
    }

    double avg_u_w = sum_u_w / nElem;
    double avg_v_w = sum_v_w / nElem;

    this->m_avg_flow_u = avg_u_w;
    this->m_avg_flow_v = avg_v_w;

    double Area_n = (this->L / nSplit) * draft;
    double Area_tau = (this->B / 2.0) * draft;

    double rho = 1000.0; // 长江淡水，与 bianxiang_move 中 midu 一致
    double C_n = 1.20;
    double C_tau = 0.15;

    double delta_X = 0.0, delta_Y = 0.0, delta_N = 0.0;

    for (int i = 0; i < nElem; i++) {
        double x_l = elements[i].x_local;
        double y_l = elements[i].y_local;

        // 🌟 核心修复 5：旋转杠杆效应符号必须用这个！
        double surge_ship_local = this->v - this->r * y_l;
        double sway_ship_local = this->u + this->r * x_l;

        double u_rel_tau = (elements[i].v_w - ref_v_w) - surge_ship_local;
        double u_rel_n   = (elements[i].u_w - ref_u_w) - sway_ship_local;

        double Fx_real = 0.5 * rho * C_tau * Area_tau * u_rel_tau * fabs(u_rel_tau);
        double Fy_real = 0.5 * rho * C_n * Area_n * u_rel_n * fabs(u_rel_n);

        // 基准状态：同一单元在均匀流（静水）中的相对速度
        double eq_rel_tau = -surge_ship_local;
        double eq_rel_n   = -sway_ship_local;

        double Fx_eq = 0.5 * rho * C_tau * Area_tau * eq_rel_tau * fabs(eq_rel_tau);
        double Fy_eq = 0.5 * rho * C_n * Area_n * eq_rel_n * fabs(eq_rel_n);

        double dX = Fx_real - Fx_eq; // Surge(纵向)附加力
        double dY = Fy_real - Fy_eq; // Sway(横向)附加力

        if (i < 8) {                 // 日志数组固定 8 槽，细分时只记前 8 个
            m_fsi_log[i].dX = dX;
            m_fsi_log[i].dY = dY;
        }

        delta_X += dX; // 总Surge
        delta_Y += dY; // 总Sway
        delta_N += (x_l * Fy_real - y_l * Fx_real) - (x_l * Fy_eq - y_l * Fx_eq);
    }

    // X_flow 赋 Surge，Y_flow 赋 Sway
    this->current_flow_forces = {delta_X, delta_Y, delta_N};
}
