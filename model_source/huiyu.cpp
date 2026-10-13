#include "huiyu.h"
#include<math.h>
#include<QDebug>
#include<QSqlQuery>
#include "mainwindow.h" // 包含全局日志计数器定义

// 辅助：角度归一化 (-PI ~ PI)
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
QVector<Jumin>juminarray;

// --- 新增：静态障碍物检测函数 ---
bool HuiYu::checkStaticCollision(const Ship &ownship, float checkCourse, float checkDist, const QVector<ChannelPoly> &obstacles)
{
    // 生成探测线段的终点
    QPointF startPos(ownship.X, ownship.Y);
    QPointF endPos(ownship.X + checkDist * sin(checkCourse),
                   ownship.Y + checkDist * cos(checkCourse));
    
    // 简单的碰撞检测：检查终点是否在障碍物多边形内
    // (更严谨的做法是检查线段与多边形相交，但对于高频检测，检查终点和中点通常足够)
    QPointF midPos = (startPos + endPos) / 2.0;

    for (const auto &poly : obstacles) {
        if (poly.points.isEmpty()) continue;

        // 构造 QPolygonF 用于几何判断
        QPolygonF polygon;
        for (const auto &p : poly.points) polygon << p;

        // 检查终点或中点是否在障碍物内
        if (polygon.containsPoint(endPos, Qt::OddEvenFill) ||
            polygon.containsPoint(midPos, Qt::OddEvenFill)) {
            return true; // 撞了
        }
    }
    return false;
}
// 角度归一化到 [0, 2π)
float HuiYu::normalizeAngle(float angle) const
{
    // 确保角度在 [-2π, 2π) 范围内
    angle = fmod(angle, 2 * M_PI);
    
    // 如果为负，转换为正角度
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    
    return angle;
}

// 辅助函数：判断船是否顺航道 (航向与航道夹角小于 30 度)
bool HuiYu::isFollowingChannel(float shipCourse, float channelCourse) const
{
    float diff = fabs(normalizeAngle(shipCourse - channelCourse));
    return diff < (30.0 * M_PI / 180.0); // 30度以内视为顺航道
}

HuiYu::HuiYu()
{

}
void HuiYu::calculate_channelinfomation( Ship &ownship)
{


}



void HuiYu::duiyu(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn)
{
    if(CRITime<1200)
    {
        limitGxj=5;
        Turn=1;
    }
    else
    {
        limitGxj=5;
        Turn=3;
    }
}

void HuiYu::crossinggiveway(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn)
{
    limitGxj=5;
    Turn=1;
}

void HuiYu::crossingstandon(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn)
{
    if(CRITime<740)
    {
        limitGxj=5;
        Turn=1;
    }
    else
    {
        limitGxj=5;
        Turn=0;
    }
}

void HuiYu::overtakegiveway(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn)
{
    // 论文规则：尽可能从左舷追越
    // Turn = 2 代表向左搜索避让航向
    // Turn = 1 代表向右
    
    limitGxj = 5; // 最小改向幅度
    Turn = 2;     // 【强制左转】 符合内河规则
    
    // 补充逻辑：如果左侧空间不足（比如有静态障碍），analysis 函数中的 checkStaticCollision 会检测到
    // 如果左转会导致撞桥，analysis 应该有回退机制或者您可以扩展这里
    // 目前基于您的 analysis 逻辑，如果 Turn=2 找不到解，它没有自动切回 Turn=1。
    // 建议在 analysis 中增加“如果首选方向不行，尝试另一侧”的逻辑（见下文优化建议）。
}

void HuiYu::overtakestandon(const Ship &ownship, const Ship &targetship, int &CRITime, int &limitGxj, int &Turn)
{
    if(CRITime<740)
    {
        limitGxj=5;
        Turn=3;
    }else
    {
        limitGxj=5;
        Turn=0;
    }
}

float HuiYu::analysis(const Ship &ownship, const QVector<Ship> &targetship, const QVector<ChannelPoly> &staticObstacles, int &TurnOS, float &aimnp, float intendedCourse)
{
    // 【修复 1】必须先清空全局数组，否则历史数据会堆积！
    juminarray.clear();
    
    // 默认全速 (假设满速是 110 RPM 或对应的归一化值，这里假设 1.0 代表满速，或者维持当前速度)
    // 这里我们设定一个标准巡航转速，例如 100%
    float cruiseSpeed = 110.0/60.0; // 约 1.83 RPS
    aimnp = cruiseSpeed;
    
    // 1. 状态有效性检查 (如果锁定的目标消失了，重置锁)
    if (m_isOvertaking) {
        if (m_activeOvertakingTargetIdx < 0 || m_activeOvertakingTargetIdx >= targetship.size()) {
            // 目标索引越界（比如目标船驶出画面被删除了），重置
            resetState();
        } else {
            // 检查目标是否“已经让清”（比如目标已经跑到我屁股后面了）
            float D, Q1, Q2;
            calculate_XianJiao(ownship, targetship[m_activeOvertakingTargetIdx], D, Q1, Q2);
            
            // 【修改点】 增加滞后量 (Hysteresis)
            // 原逻辑: if (Q1 > 90 && Q1 < 270)
            // 新逻辑: 只有目标跑到该船舷角 115度 之后才算完成
            // 这样能防止在 90度 附近反复跳变
            if (Q1 > 115 && Q1 < 245) {
                if (g_globalLogCounter % 30 == 0)
                    qDebug() << ">>> [追越完成] 目标已显著落后(Q1:" << Q1 << ")，解锁状态。";
                resetState();
            }
        }
    }
    
    // =========================================================
    // 【新增】桥区特别规则检查 (Bridge Zone Rules)
    // =========================================================
    if (ownship.isInsideBridgeArea) {
        // 规则1: 桥区限速 (例如限制为最高速的 60%)
        float bridgeLimitSpeed = cruiseSpeed * 0.6;
        if (aimnp > bridgeLimitSpeed) {
            aimnp = bridgeLimitSpeed;
        }

        // 规则2: 桥区禁止追越 (熔断机制)
        // 检查是否正处于“主动追越”局面
        for(int i=0; i<targetship.size(); ++i) {
            int tempCRI = 0;
            float tempD = 0;
            // 预判局面
            int state = meeting_state(ownship, targetship[i], tempCRI, tempD, intendedCourse);
            
            if (state == 5) { // 5 = 我是追越船 (Overtake Give-way)
                qDebug() << ">>> [桥区禁令] 检测到追越意图，但处于桥区！强制禁止变道，保持跟随。";
                TurnOS = 0; // 强制锁死舵角，禁止变道
                // 减速跟随：目标船航速的单位是 m/s，而 aimnp 是转速 (r/s)，必须换算后再比较，
                // 否则会把 m/s 的数值当转速下发（与本文件 589 行的写法保持一致）
                float baseSpeed = (ownship.S > 0.1f) ? ownship.S : (12.5f * 0.5144f);
                float followRpm = cruiseSpeed * (targetship[i].S / baseSpeed);
                if (followRpm < aimnp) aimnp = followRpm;
                
                // 直接返回 0.0 改向角，不再进行后续的避让搜索
                return 0.0;
            }
        }
    }
    // =========================================================
    
    // 1. 如果没有动态目标，直接返回
    if (targetship.isEmpty()) {
        TurnOS = 0;
        return 0.0;
    }

    for(int i=0;i<targetship.size();++i)
    {
        // =========================================================
        // 【核心修改】 智能追越逻辑：拉开 -> 并列 -> 回正
        // =========================================================
        if (m_isOvertaking && i == m_activeOvertakingTargetIdx) {
            
            // 1. 计算横向距离 (Lateral Distance)
            float D, Q1, Q2;
            calculate_XianJiao(ownship, targetship[i], D, Q1, Q2);
            
            // 将舷角 Q1 (0~360) 转换为相对方位 (0~180) 用于计算正弦
            float relativeBearing = (Q1 > 180) ? (360 - Q1) : Q1;
            // 横向距离公式： D * sin(theta)
            float lateralDist = D * sin(relativeBearing * M_PI / 180.0);

            // 设定安全横向距离阈值 (例如 150米，或 3倍船宽)
            // 如果横向距离已经足够大，就不需要再死命往左转了
            float safeLateralDist = 150.0;

            // 动态判断准入条件：横距足够 OR 已经在前方
            bool isAhead = (Q2 < 70 || Q2 > 290);

            if (lateralDist > safeLateralDist || isAhead) {
                
                // --- 阶段 B: 横距达标 --- 
                // 【修改点】区分“并列阶段”与“回归阶段”
                
                // 1. 如果还没跑到前面 (Not Ahead Yet)，严禁回归！只能并列！
                if (!isAhead) {
                    if (g_globalLogCounter % 30 == 0)
                        qDebug() << ">>> [追越-并列] 横距达标(" << lateralDist << "m)，但未领先，保持平行直行。";
                    
                    // 策略：保持当前航向 (或者计算一个纯平行的航向)
                    // 只要前方无障碍，就稳住舵角直行
                    float maintainCourse = ownship.C;
                    
                    // 保持航向
                    TurnOS = 2; // 避碰接管：保持
                    return 0.0; // 改向 0度 (保持直行)
                }

                // 2. 只有当已经领先 (isAhead) 时，才执行原来的“回归逻辑”
                // --- 阶段 B2: 回归阶段 (Return) ---

                // 1. 计算理论回归角
                float angleDiff = intendedCourse - ownship.C;
                angleDiff = normalizeAngle(angleDiff);
                
                // 限制最大回归舵角 (平滑切入)
                float maxReturnAngle = 15.0 * M_PI / 180.0;
                if (angleDiff > maxReturnAngle) angleDiff = maxReturnAngle;
                if (angleDiff < -maxReturnAngle) angleDiff = -maxReturnAngle;

                float tryCourse = ownship.C + angleDiff;

                // ==========================================================
                // 【核心修改】 轨迹预测校验 (Trajectory Prediction Check)
                // 不再只看当前时刻，而是推演未来 120秒 的动态变化
                // ==========================================================
                
                bool futureSafe = true;
                float predictionHorizon = 120.0; // 预测未来 2分钟
                float timeStep = 5.0;            // 每 5秒 检查一次
                float safeDomain = 150.0;        // 最小安全距离 (动静安全圈)

                // 记录碰撞风险源 (用于日志)
                int collisionSourceId = -1;

                // 预测循环
                for (float t = 0; t <= predictionHorizon; t += timeStep) {
                    
                    // 1. 预测本船位置 (假设以当前速度变向到 tryCourse 行驶)
                    // (此处简化为线性运动，实际可引入转弯半径模型，但线性预测对安全更保守)
                    float futureOwnX = ownship.X + ownship.v * t * sin(tryCourse);
                    float futureOwnY = ownship.Y + ownship.v * t * cos(tryCourse);

                    // 2. 预测被追越船位置 (假设保持当前航速航向)
                    float futureTargetX = targetship[i].X + targetship[i].v * t * sin(targetship[i].C);
                    float futureTargetY = targetship[i].Y + targetship[i].v * t * cos(targetship[i].C);
                    
                    // 检查 1: 与被追越船的未来距离
                    float futureDist = sqrt(pow(futureOwnX - futureTargetX, 2) + pow(futureOwnY - futureTargetY, 2));
                    
                    if (futureDist < safeDomain) {
                        futureSafe = false;
                        if (g_globalLogCounter % 30 == 0)
                            qDebug() << ">>> [预测报警] T+" << t << "s 后将与被追越船距离过近(" << futureDist << "m)，禁止回归。";
                        break;
                    }

                    // 检查 2: 与其他船的未来距离 (防止避开了这个撞了那个)
                    for(int j=0; j<targetship.size(); ++j) {
                        if (i == j) continue;
                        float fOtherX = targetship[j].X + targetship[j].v * t * sin(targetship[j].C);
                        float fOtherY = targetship[j].Y + targetship[j].v * t * cos(targetship[j].C);
                        float fOtherDist = sqrt(pow(futureOwnX - fOtherX, 2) + pow(futureOwnY - fOtherY, 2));
                        
                        if (fOtherDist < safeDomain) {
                            futureSafe = false;
                            collisionSourceId = j;
                            break;
                        }
                    }
                    if (!futureSafe) break;
                    
                    // 检查 3: 动态障碍物预测 (防止切回去撞船)
                    // 静态障碍检查已删除
                }
                
                // ==========================================================

                if (futureSafe) {
                    // === 情况 1: 未来轨迹全程安全 ===
                    if (g_globalLogCounter % 30 == 0)
                        qDebug() << ">>> [追越-回归] 轨迹预测安全(120s)，执行回归。";
                    
                    // 标记为避碰接管 (2)，防止 LOS 干扰，直到彻底回到航线
                    TurnOS = (fabs(angleDiff) < 0.001) ? 2 : ((angleDiff > 0) ? 1 : -1);
                    return angleDiff;
                }
                else {
                    // === 情况 2: 预测有风险，保持并列 ===
                    if (g_globalLogCounter % 30 == 0) {
                        if (collisionSourceId != -1)
                            qDebug() << ">>> [追越-并列] 回归将导致与船(ID:" << collisionSourceId << ")冲突，维持直行。";
                        else
                            qDebug() << ">>> [追越-并列] 回归路径动态预测不安全，维持直行。";
                    }

                    float maintainCourse = ownship.C;
                    // 保持航向
                    TurnOS = 2; // 2 = 避碰接管中的保持
                    return 0.0;
                }
            }
            else {
                // --- 阶段 A: 拉开距离 (Diverging) ---
                
                // 计算当前偏角
                float currentDivergence = normalizeAngle(ownship.C - intendedCourse);
                
                int limitGxj = 5; // 默认强行左转
                
                // 如果已经向左偏了，允许保持
                if (currentDivergence < -2.0 * M_PI / 180.0) {
                    limitGxj = 0;
                    if (g_globalLogCounter % 30 == 0)
                        qDebug() << ">>> [追越保持] 船头已向左偏，允许保持航向。";
                } else {
                    if (g_globalLogCounter % 30 == 0)
                        qDebug() << ">>> [追越动作] 船头未偏，强制左转拉开。";
                }

                int fakeCRITime = 100;
                int Turn = 2; // 依然标记为左转意图
                
                // 这里调用 overtakegiveway 只是为了设置 limitGxj 和 Turn 的默认值
                overtakegiveway(ownship, targetship[i], fakeCRITime, limitGxj, Turn);
                
                // 手动修正 limitGxj (因为 overtakegiveway 里是写死的 5)
                if (currentDivergence < -2.0 * M_PI / 180.0) {
                    limitGxj = 0;
                }
                
                Jumin jumin(5, fakeCRITime, limitGxj, Turn);
                juminarray.push_back(jumin);
                continue; // 跳过常规 meeting_state 判断，进入下方联合搜索执行左转
            }
        }

        float D=0;
        int CRITime=0;
        int state=meeting_state(ownship,targetship[i],CRITime,D, intendedCourse);
        int limitGxj=0;
        int Turn=0;
        switch (state)
        {
        case 1:
            duiyu(ownship,targetship[i],CRITime,limitGxj,Turn);
            break;
        case 2:
            crossingstandon(ownship, targetship[i],CRITime,limitGxj,Turn);
            break;
        case 3:
            crossinggiveway(ownship, targetship[i],CRITime,limitGxj,Turn);
            break;
        case 4:
            overtakestandon(ownship, targetship[i],CRITime,limitGxj,Turn);
            break;
        case 5:
            overtakegiveway(ownship, targetship[i],CRITime,limitGxj,Turn);
            break;
        case 6:
            limitGxj=0;
            Turn = 0;
            break;
        default:
            break;
        }
        Jumin jumin(state,CRITime,limitGxj,Turn);
        juminarray.push_back(jumin);
        
        // 如果这里发现了新的追越对象，并且当前没有锁定其他任务
        if (!m_isOvertaking && state == 5) {
            // 计算一下相对方位
            float tempD, tempQ1, tempQ2;
            calculate_XianJiao(ownship, targetship[i], tempD, tempQ1, tempQ2);

            // 【修改点】 增加前方扇区检查
            // 只有当目标在本船前方 (例如舷角 < 80度) 时，才允许开始追越锁定
            // 如果目标已经在 90度 附近或后面，说明是已经超过去的船，或者是被追越完的船，不要再锁了
            bool isTargetAhead = (tempQ1 < 80 || tempQ1 > 280);

            if (D < 1000.0 && isTargetAhead) {
                m_activeOvertakingTargetIdx = i;
                m_isOvertaking = true;
                if (g_globalLogCounter % 30 == 0)
                    qDebug() << ">>> [追越开始] 锁定目标" << i << " (Q1:" << tempQ1 << ")";
            }
        }
    }

    // 【修复 3】增加判空保护
    if (juminarray.isEmpty()) {
        TurnOS = 0;
        return 0.0;
    }

    // 2. 寻找最危险动态目标 (使用 CRI 替代简单的 minCRI)
    int k = -1;
    float maxCRI = -1.0f; // 找最大的 CRI
    bool hasRisk = false;

    for(int i=0; i<juminarray.size(); ++i) {
        // 如果状态是 6 (无碰撞)，CRI 为 0
        if(juminarray[i].state == 6) continue;
        
        // 计算该目标的 CRI 值
        float currentCRI = calculate_CRI_Value(juminarray[i].CRITime);
        
        // 寻找最大威胁
        if(currentCRI > maxCRI) {
            maxCRI = currentCRI;
            k = i;
            hasRisk = true;
        }
    }

    // 3. 决策生成 (加入阈值判断)
    float gxj=0;
    int time=1;
    float finalGxj = 0.0;
    
    // 【改进点】增加 CRI 阈值判断 (例如 > 0.3 才开始避让)
    // 0.3 对应的时间大约是：(1 - 0.3^(1/3)) * 1200 ≈ 400秒 (约6-7分钟)
    // 意味着 7分钟以外的碰撞风险暂时忽略，保持航向
    float reactionThreshold = 0.3f;

    if (!hasRisk || maxCRI < reactionThreshold) {
        // --- 风险太小，暂不理会，或者作为直航船 ---
        TurnOS = 0;
        gxj = 0.0;
        return gxj;
    }
    else if(juminarray[k].state==6)
    {
        gxj=0;
        TurnOS=0; // 【修复 4】无危险时TurnOS必须为0，把控制权还给LOS
    }
    else if(juminarray[k].Turn==0)
    {
        gxj=0;
        TurnOS=2;
    }
    else if(juminarray[k].Turn==1 || juminarray[k].Turn==2)
    {
        // --- 联合搜索逻辑 (Joint Search) ---
        TurnOS = 1;
        
        int startAngle = juminarray[k].limitGxj; // 最小改向角
        int sign = (juminarray[k].Turn == 2) ? -1 : 1;
        
        bool solutionFound = false;

        // 如果当前有速度指令，优先参考当前指令，或者默认为巡航速度
        float currentBaseSpeed = (ownship.S > 0.1) ? ownship.S : (12.5 * 0.5144);

        // 外层循环：航向 (Angle) - 优先小角度
        // 如果处于锁定追越状态，强制使用左侧搜索（sign = -1）
        int effectiveSign = (m_isOvertaking) ? -1 : sign;
        
        for(int angle = startAngle; angle <= 90; angle += 5)
        {
            float gxj = effectiveSign * angle * M_PI / 180.0;
            float tryCourse = ownship.C + gxj;        
            
            // 内层循环：速度 (Speed) - 优先保速，其次减速
            // 速度系数：1.0(全速), 0.8, 0.6, 0.4, 0.2
            for (float speedRatio = 1.0f; speedRatio >= 0.2f; speedRatio -= 0.2f)
            {
                float trySpeed = currentBaseSpeed * speedRatio;
                // 对应的 RPM (假设线性关系)
                float tryRpm = cruiseSpeed * speedRatio;

                // 1. 动态检查 (Dynamic Check)
                int tempCRI = 0;
                bool dynamicSafe = true;
                for(int j=0; j<targetship.size(); j++) {
                    // 使用 trySpeed 进行推演
                    if(calculate_CRITime(ownship, targetship[j], tempCRI, tryCourse, trySpeed)) {
                        dynamicSafe = false;
                        break;
                    }
                }
                
                // 只进行动态检查
                if(dynamicSafe) {
                    gxj = gxj;
                    finalGxj = gxj;
                    aimnp = tryRpm; // 采纳建议转速
                    
                    if (g_globalLogCounter % 30 == 0)
                        qDebug() << ">>> [联合避让] 改向:" << angle * sign << "度 | 速度系数:" << speedRatio;
                    
                    solutionFound = true;
                    break;
                }
            }
            if (solutionFound) break; // 找到最优航向即退出
        }

        // 【新增代码开始】 === 零度兜底机制 ===
        // 如果强制要求的改向 (比如 5度) 找不到解，尝试一下 0度 (保持航向) 是否安全
        // 这样可以避免直接掉入 "减速跟随" 的死循环，允许船只先直行通过狭窄区域
        if (!solutionFound && startAngle > 0) {
             float tryCourse = ownship.C; // 0度，即当前航向
             float trySpeed = currentBaseSpeed; // 保持速度
             float tryRpm = cruiseSpeed;
             
             // 检查 0度 是否安全
             int tempCRI = 0;
             bool dynamicSafe = true;
             for(int j=0; j<targetship.size(); j++) {
                 if(calculate_CRITime(ownship, targetship[j], tempCRI, tryCourse, trySpeed)) {
                     dynamicSafe = false; break;
                 }
             }

             if(dynamicSafe) {
                 gxj = 0.0;
                 finalGxj = 0.0;
                 aimnp = tryRpm;
                 TurnOS = 2; // 【关键修改】标记为"避碰模式下的直行"
                 solutionFound = true; // 标记为找到解了！
                 
                 if (g_globalLogCounter % 30 == 0)
                     qDebug() << ">>> [联合避让-兜底] 强制转向失败，降级为保持航向(0度)通过。";
             }
        }
        // 【新增代码结束】 =====================

        // 如果循环结束还没找到合适角度和速度组合
        if (!solutionFound) {
            // 检查是否处于追越局面
            bool isOvertakeSituation = false;
            Ship targetForFollow;
            for(int i=0; i<targetship.size(); ++i) {
                int tempCRI = 0;
                float tempD = 0;
                int state = meeting_state(ownship, targetship[i], tempCRI, tempD, intendedCourse);
                if (state == 5) { // 5 = 我是追越船 (Overtake Give-way)
                    isOvertakeSituation = true;
                    targetForFollow = targetship[i];
                    break;
                }
            }
            
            if (isOvertakeSituation) {
                // 追越局面：减速跟随目标船，不要紧急停车
                // 速度设为目标船速度或稍小 (0.9倍)
                float followSpeed = targetForFollow.S * 0.9;
                float followRpm = cruiseSpeed * (followSpeed / currentBaseSpeed);
                
                aimnp = followRpm;
                TurnOS = 2; // 【关键修改】跟随模式，也要屏蔽 LOS
                gxj = 0.0;
            } else {
                // 非追越局面：执行紧急停车
                qDebug() << ">>> [紧急] 无法找到避让方案，紧急停车！";
                aimnp = 0;
                TurnOS = 0; // 既然没路走，保持航向减速可能比乱转安全
                gxj = 0.0;
            }
        } else {
            // 设置TurnOS值
            if (finalGxj > 0) {
                TurnOS = 1; // 右转
            } else if (finalGxj < 0) {
                TurnOS = -1; // 左转
            } else {
                // 【关键修改】如果是 0 度，检查是否处于追越锁定中
                if (m_isOvertaking) {
                    TurnOS = 2; // 追越中，即使不转舵，也要屏蔽 LOS
                } else {
                    TurnOS = 0; // 平时巡航，还给 LOS
                }
            }
            gxj = finalGxj;
        }
    }
    else
    {
        float gxjR=0;
        for(int i=juminarray[k].limitGxj;i<=90;++i)
        {
            gxjR=i*M_PI/180;
            bool gxjflag=true;
            for(int j=0;j<targetship.size();++j)
            {
                Ship ship0=ownship;
                Ship shipT=targetship[j];
                int CRITime=0;
                float aimC=ship0.C+gxjR;
                float delta = 0;
                float aimnp=ownship.np;
                float Kp = 1200, Td = 0.74, Ti = 75;

                int time = 1;
                float C1 = 0, C2 = 0;
                int max_steering_steps = 100; // 防止无限循环的最大步数
                bool steering_complete = false;
                for(int k = 1; k <= max_steering_steps; k++)
                {
                    delta += Kp*(1+time/Ti+Td/time)*(aimC-ship0.C)
                            - Kp*(1+2*Td/time)*(aimC-ship0.C2)
                            + Kp*(Td/time)*(aimC-ship0.C1);
                    ship0.C1 = ship0.C2;
                    ship0.C2 = ship0.C;
                    if(delta >= 35*M_PI/180)
                    {
                        delta = 35*M_PI/180;
                    }
                    if(delta <= -35*M_PI/180)
                    {
                        delta = -35*M_PI/180;
                    }
                    // 创建默认的环境数据
                    EnvData env;
                    env.windSpeed = 0.0;
                    env.windDir = 0.0;
                    env.currentSpeed = 0.0;
                    env.currentDir = 0.0;
                    env.waterDepth = 20.0; // 默认水深
                    // 使用默认转速110 RPM（与原来的硬编码值一致）
                    float defaultRpm = 110.0;
                    ship0.bianxiang_move(delta*180/M_PI, time, env, defaultRpm);
                    if(fabs(ship0.C - aimC) <= 1*M_PI/180)
                    {
                        shipT.baoxiang_move(k*time);
                        steering_complete = true;
                        break;
                    }
                }
                if (!steering_complete) {
                    // 超过最大转向步数，强制完成转向
                    shipT.baoxiang_move(max_steering_steps * time);
                }
                if(calculate_CRITime(ship0,shipT,CRITime, ship0.C+gxjR))
                {
                    gxjflag=false;
                    break;
                }
            }
            if(!gxjflag)
            {
                if(i==90)qDebug()<<"右转无可行航向";
                continue;
            }
            else break;
        }

        float gxjL=0;
        for(int i=-juminarray[k].limitGxj;i>=-90;--i)
        {
            gxjL=i*M_PI/180;
            bool gxjflag=true;
            for(int j=0;j<targetship.size();++j)
            {
                Ship ship0=ownship;
                Ship shipT=targetship[j];
                int CRITime=0;
                float aimC=ship0.C+gxjL;
                float delta = 0;
                float aimnp=ownship.np;
                float Kp = 1200, Td = 0.74, Ti = 75;
                int max_steering_steps = 100; // 防止无限循环的最大步数
                
                for(int k = 1; k <= max_steering_steps; k++)
                {
                    delta += Kp*(1+time/Ti+Td/time)*(aimC-ship0.C)
                            - Kp*(1+2*Td/time)*(aimC-ship0.C2)
                            + Kp*(Td/time)*(aimC-ship0.C1);
                    ship0.C1 = ship0.C2;
                    ship0.C2 = ship0.C;
                    if(delta >= 35*M_PI/180)
                    {
                        delta = 35*M_PI/180;
                    }
                    if(delta <= -35*M_PI/180)
                    {
                        delta = -35*M_PI/180;
                    }
                    // 创建默认的环境数据
                    EnvData env;
                    env.windSpeed = 0.0;
                    env.windDir = 0.0;
                    env.currentSpeed = 0.0;
                    env.currentDir = 0.0;
                    env.waterDepth = 20.0; // 默认水深
                    // 使用默认转速110 RPM（与原来的硬编码值一致）
                    float defaultRpm = 110.0;
                    ship0.bianxiang_move(delta*180/M_PI, time, env, defaultRpm);
                    
                    if(fabs(ship0.C - aimC) <= 1*M_PI/180)
                    {
                        break;
                    }
                }
                shipT.baoxiang_move(time);
                if(calculate_CRITime(ship0,shipT,CRITime, ship0.C+gxjL))
                {
                    gxjflag=false;
                    break;
                }
            }
            if(!gxjflag)
            {
                if(i==-90)qDebug()<<"左转无可行航向";
                continue;
            }
            else break;
        }
        gxj=fabs(gxjR)>=fabs(gxjL)?gxjL:gxjR;
    }

    // 设置TurnOS值
    if (gxj > 0) {
        TurnOS = 1; // 右转
    } else if (gxj < 0) {
        TurnOS = -1; // 左转
    } else {
        TurnOS = 0; // 不转向
    }

    // 【新增调试日志】输出最终避碰决策
    if (TurnOS != 0 && g_globalLogCounter % 30 == 0) {
        qDebug() << "=== [避碰决策] 针对目标船" << k << " | 局面类型:" << juminarray[k].state 
                 << " | 建议改向:" << gxj * 180.0/M_PI << "度 | TurnOS:" << TurnOS;
    }
    return gxj;

}

void HuiYu::calculate_XianJiao(const Ship &pO, const Ship &pT, float &D, float &Q1, float &Q2)
{
    double dx = pT.X - pO.X;
    double dy = pT.Y - pO.Y;
    D = sqrt(dx*dx + dy*dy);

    // 1. 计算目标真方位 TB (0~2PI)
    float TB = atan2(dx, dy);
    if (TB < 0) TB += 2 * M_PI;

    // 2. 计算 Q1 (本船看目标舷角)
    float TC = TB - pO.C;
    TC = normalizeAngle(TC);
    if (TC < 0) TC += 2 * M_PI; // 转为 0~360 格式
    Q1 = TC * 180.0 / M_PI;

    // 3. 计算 Q2 (目标看本船舷角)
    float TB_opp = TB + M_PI; // 反方位
    float TC1 = TB_opp - pT.C;
    TC1 = normalizeAngle(TC1);
    if (TC1 < 0) TC1 += 2 * M_PI;
    Q2 = TC1 * 180.0 / M_PI;
}

bool HuiYu::calculate_CRITime(const Ship &pO, const Ship &pT, int &CRITime, float intendedCourse, float proposedSpeed)
{
    // 使用意图航向进行预测
    float CO = intendedCourse;
    
    // 【关键】如果传入了推演速度，就用推演速度；否则用当前速度
    float SO = (proposedSpeed >= 0.0f) ? proposedSpeed : pO.S;
    
    // 预测参数
    int max_predict_steps = 1800; // 预测未来 1800秒 (30分钟)
    float time_step = 1.0f;
    
    // 【核心修改】增大碰撞安全阈值 (Safety Margin)
    // 设为 1.5 * pO.L (1.5倍船长) 或 固定 300米
    float safeDistance = (pO.L > 10.0) ? (1.5 * pO.L) : 300.0f;
    float collision_threshold = safeDistance;

    for(int i = 1; i <= max_predict_steps; i++)
    {
        // 1. 预测本船位置 (匀速直线)
        double xo1 = pO.X + SO * sin(CO) * i * time_step;
        double yo1 = pO.Y + SO * cos(CO) * i * time_step;
        
        // 2. 预测目标船位置 (保向保速)
        double xt1 = pT.X + pT.S * sin(pT.C) * i * time_step;
        double yt1 = pT.Y + pT.S * cos(pT.C) * i * time_step;
        
        // 3. 计算未来距离
        double dist = sqrt(pow(xo1 - xt1, 2) + pow(yo1 - yt1, 2));
        
        // 4. 判定风险
        if(dist <= collision_threshold)
        {
            CRITime = i;
            return true; // 发现碰撞风险
        }
    }
    return false;
}

// 新增函数：根据论文公式 (3-7) 计算危险度
float HuiYu::calculate_CRI_Value(int collisionTime)
{
    // T1: 碰撞危险形成的时间阈值 (论文建议 1200秒)
    const float T1 = 1200.0f;
    
    // Ts: 预计碰撞时间
    float Ts = (float)collisionTime;

    if (Ts <= 0) return 1.0f;           // 已经撞上了
    if (Ts > T1) return 0.0f;           // 还在安全时间外

    // 公式：((T1 - Ts) / T1)^3
    // 3次方让近距离的风险权重急剧增加
    float risk = pow((T1 - Ts) / T1, 3.0);
    
    return risk;
}

// 辅助函数：计算航向差的绝对值 (0 ~ PI)
double getAngleDiff(double angle1, double angle2) {
    double diff = fabs(normalizeAngle(angle1 - angle2));
    if (diff > M_PI) diff = 2 * M_PI - diff;
    return diff;
}

int HuiYu::meeting_state(const Ship &ownShip, const Ship &targetShip, int &CRITime, float &D, float intendedCourse)
{
    int Type = 6; // 默认安全/无风险

    float Q1 = 0, Q2 = 0;
    calculate_XianJiao(ownShip, targetShip, D, Q1, Q2);

    // 1. 风险预判 (CRI)
    // 如果没有碰撞风险，直接返回 6
    if(!calculate_CRITime(ownShip, targetShip, CRITime, intendedCourse)) {
        // 调试日志
        // qDebug() << "  [风险] 无碰撞风险";
        return 6;
    }

    // =========================================================
    // 【基于论文 3.1.2 的会遇局面辨识模型】
    // =========================================================
    
    // 阈值定义 (参考论文或根据内河实际调整)
    // Theta1: 顺航道判定的角度容差 (论文通常取 30度 或更小，如 22.5度)
    double Theta1 = 30.0 * M_PI / 180.0;
    // Theta2: 横越判定的角度容差 (对90度的偏差)
    double Theta2 = 15.0 * M_PI / 180.0;

    // 2. 分析本船状态
    // 本船航向与航道走向的夹角
    double diffOwn = getAngleDiff(ownShip.C, intendedCourse);
    bool ownIsFollowing = (diffOwn <= Theta1); // 本船是否顺航道

    // 3. 分析目标船状态
    // 目标船与“本船所在航道”同向的夹角
    double diffTargetSame = getAngleDiff(targetShip.C, intendedCourse);
    // 目标船与“本船所在航道”反向的夹角
    double diffTargetOpposite = getAngleDiff(targetShip.C, intendedCourse + M_PI);
    
    bool targetIsSameWay = (diffTargetSame <= Theta1);      // 目标船同向顺航道
    bool targetIsOppositeWay = (diffTargetOpposite <= Theta1); // 目标船反向顺航道
    
    // 目标船是否在横越? (航向与航道垂直，夹角在 90 +/- Theta2 范围内)
    // 即：航向差在 [90-15, 90+15] -> [75, 105] 之间
    double diffTargetCross = getAngleDiff(targetShip.C, intendedCourse);
    bool targetIsCrossing = (diffTargetCross >= (M_PI/2.0 - Theta2) && diffTargetCross <= (M_PI/2.0 + Theta2));

    // =================== 局面判定逻辑 ===================

    // Case A: 【对驶相遇】(Head-on)
    // 表3-1定义: 两船均顺航道，且航向相反 (一个同向，一个反向)
    if (ownIsFollowing && targetIsOppositeWay) {
        // qDebug() << "  [辨识] 论文模型 -> 对驶相遇 (Type 1)";
        return 1; // 对遇
    }

    // Case B: 【追越】(Overtaking)
    // 表3-1定义: 两船均顺航道，且航向相同
    if (ownIsFollowing && targetIsSameWay) {
        // 进一步细分：谁追谁？
        // 论文提到结合速度判断
        if (ownShip.S > targetShip.S) {
            // qDebug() << "  [辨识] 论文模型 -> 我追他 (Type 5)";
            return 5; // 我是追越船 (让路)
        } else {
            // qDebug() << "  [辨识] 论文模型 -> 他追我 (Type 4)";
            return 4; // 我是被追越船 (直航)
        }
    }

    // Case C: 【横越】(Transverse)
    // 表3-1定义: 一船顺航道，一船横越
    
    // 情况 1: 我顺航道，他横越 -> 他让路，我直航
    if (ownIsFollowing && targetIsCrossing) {
        // qDebug() << "  [辨识] 论文模型 -> 目标船横越 (Type 2)";
        return 2; // 交叉直航
    }
    
    // 情况 2: 我横越 (假设本船因避让等原因处于横越状态)，他顺航道 -> 我让路
    // 注意：这里还要判断目标船是顺航道 (无论同向还是反向)
    bool targetIsFollowing = targetIsSameWay || targetIsOppositeWay;
    
    // 本船横越判定 (相对航道接近垂直)
    bool ownIsCrossing = (diffOwn >= (M_PI/2.0 - Theta2) && diffOwn <= (M_PI/2.0 + Theta2));

    if (ownIsCrossing && targetIsFollowing) {
        // qDebug() << "  [辨识] 论文模型 -> 本船横越 (Type 3)";
        return 3; // 交叉让路
    }

    // Case D: 【交叉相遇】(Intersection)
    // 表3-1定义: 不属于上述三种情况 (例如两船都在斜着走，或者都在航道外)
    // 此时回退到几何判断，或者依据右舷让路规则
    // 简单起见，依据 COLREGs 舷角判断
    if (Q1 > 0 && Q1 <= 112.5) { // 他在我的右舷
        // qDebug() << "  [辨识] 论文模型 -> 一般交叉/右舷来船 (Type 3)";
        return 3; // 让路
    } else {
        // qDebug() << "  [辨识] 论文模型 -> 一般交叉/左舷来船 (Type 2)";
        return 2; // 直航
    }
}





