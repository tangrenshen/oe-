#include "flow_field.h"
#include <QtMath>
#include <QPolygonF>
#include <algorithm>
#include <QDebug>
#include <QSqlError>
#include <QCoreApplication>
#include <QPainterPathStroker>

FlowField::FlowField() :
    m_U0(1.5), m_flowAngleRad(120.0 * M_PI / 180.0),
    m_gridSize(100.0), // 提升流场网格精度到 100m
    m_rbfShapeParam(2000.0), m_rbfDepthWeight(1000.0), m_rbfInitialized(false),
    m_lastRBFUpdateTime(-1.0),
    m_isRecording(false), m_isPlayback(false),
    m_pylonFocusMode(false)
{
}

double FlowField::getDepthAt(double x, double y) const {
    return getInterpolatedDepthOptimized(x, y);
}

QPointF FlowField::getFlowVectorAt(double x, double y, double t) {
    return computeAt(x, y, t);
}

void FlowField::setPylons(const QVector<ChannelPoly>& pylons) {
    m_pylons = pylons;
}

void FlowField::setSandBars(const QVector<ChannelPoly>& sandBars) {
    m_sandBars = sandBars;
    qDebug() << "✅ 已更新沙洲数据，共" << sandBars.size() << "个沙洲多边形";
}

void FlowField::setBackgroundFlow(double U0, double flowAngleDeg) {
    m_U0 = U0;
    m_flowAngleRad = flowAngleDeg * M_PI / 180.0;
}

void FlowField::initGlobalFlow(const QVector<QPolygonF>& shorelines,
                                const QVector<ChannelPoly>& landPolygons,
                                const QVector<SoundingPoint>& soundings,
                                const QVector<ChannelPoly>& deparePolygons) {
    m_shorelines = shorelines;
    m_soundings = soundings;
    m_landPolygons = landPolygons;

    // 【新增】保存 DEPARE 数据
    m_deparePolygons = deparePolygons;

    // =================================================
    // 【新增】 构建空间索引 (包围盒缓存)
    m_shorelineBounds.clear();
    m_shorelineBounds.reserve(m_shorelines.size());
    for (const auto& line : m_shorelines) {
        m_shorelineBounds.append(line.boundingRect());
    }

    m_depareBounds.clear();
    m_depareBounds.reserve(m_deparePolygons.size());
    for (const auto& poly : m_deparePolygons) {
        if (poly.points.isEmpty()) {
            m_depareBounds.append(QRectF());
        } else {
            // 手动计算 ChannelPoly 的包围盒
            double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
            for (const auto& p : poly.points) {
                if (p.x() < minX) minX = p.x();
                if (p.x() > maxX) maxX = p.x();
                if (p.y() < minY) minY = p.y();
                if (p.y() > maxY) maxY = p.y();
            }
            m_depareBounds.append(QRectF(QPointF(minX, minY), QPointF(maxX, maxY)));
        }
    }

    qDebug() << "🚀 空间索引构建完成: 岸线" << m_shorelineBounds.size() << "个, 水深面" << m_depareBounds.size() << "个";
    // =================================================

    buildDepthGrid();

    m_waterDomainMask = QPainterPath();

    // � 动态解算流场环境包围盒，取代硬编码
    double minX = 1e18, maxX = -1e18, minY = 1e18, maxY = -1e18;
    bool hasData = false;
    for (const auto& rect : m_shorelineBounds) {
        if (rect.isNull()) continue;
        hasData = true;
        minX = qMin(minX, rect.left()); maxX = qMax(maxX, rect.right());
        minY = qMin(minY, rect.top());  maxY = qMax(maxY, rect.bottom());
    }
    for (const auto& rect : m_depareBounds) {
        if (rect.isNull()) continue;
        hasData = true;
        minX = qMin(minX, rect.left()); maxX = qMax(maxX, rect.right());
        minY = qMin(minY, rect.top());  maxY = qMax(maxY, rect.bottom());
    }
    
    double margin = 5000.0; // 外扩 5km 冗余
    QRectF totalRect = hasData ? QRectF(minX - margin, minY - margin, maxX - minX + 2*margin, maxY - minY + 2*margin)
                               : QRectF(0, 0, 10000, 10000);
    m_waterDomainMask.addRect(totalRect);

    // 1. 无差别扣除所有 LNDARE (面状陆地)
    for (const auto& land : landPolygons) {
        if (land.points.isEmpty()) continue;
        QPainterPath landPath;
        // 多环几何：每个环单独作为子路径，配合 QPainterPath 默认的 OddEvenFill，内环自然成为洞。
        // 岸陆是一个整体多边形、河道即其内环，若只用外环会把整条航道扣成陆地。
        if (land.rings.size() > 1) {
            for (const auto& ring : land.rings) {
                if (ring.size() > 2) landPath.addPolygon(QPolygonF(ring));
            }
        } else {
            landPath.addPolygon(QPolygonF(land.points));
        }
        m_waterDomainMask = m_waterDomainMask.subtracted(landPath);
    }

    // 2. 将所有 COALNE (线状岸线) 转化为 120m 物理屏障
    if (!m_shorelines.isEmpty()) {
        QPainterPath shorelinePath;
        for (const auto& line : m_shorelines) {
            if (line.size() < 2) continue;
            shorelinePath.moveTo(line[0]);
            for (int i = 1; i < line.size(); ++i) shorelinePath.lineTo(line[i]);
        }
        QPainterPathStroker stroker;
        stroker.setWidth(120.0);
        stroker.setCapStyle(Qt::RoundCap);
        stroker.setJoinStyle(Qt::RoundJoin);
        m_waterDomainMask = m_waterDomainMask.subtracted(stroker.createStroke(shorelinePath));
    }
    qDebug() << "✅ 通用全流域环境掩码已建立";
}

void FlowField::setTidalSources(const QVector<FEBPoint>& sources) {
    m_febSources = sources;
    qDebug() << "⚓ 已建立" << m_febSources.size() << "个潮流动力源观测点";
}

void FlowField::loadAnchorsFromDb(QSqlDatabase &db) {
    QVector<FEBPoint> anchors;
    
    QSqlQuery query(db);
    // 使用完全匹配的数据库字段名
    if (query.exec("SELECT id, lon, lat, speed, direction, depth FROM VIRTUAL_FLOW_ANCHORS")) {
        while (query.next()) {
            FEBPoint anchor;
            
            // 提取基础字段
            anchor.id = query.value(0).toInt();
            double lon = query.value(1).toDouble();
            double lat = query.value(2).toDouble();
            
            // 🌟 核心修复：高精度 Web Mercator 投影转换 (EPSG:3857)
            // 与全工程一致：缩放成真实米（MERC_SCALE 见 types.h），否则锚点会与航道错开 15%
            double lonRad = lon * M_PI / 180.0;
            double latRad = lat * M_PI / 180.0;

            anchor.x = MERC_SCALE * MERC_R * lonRad;
            anchor.y = MERC_SCALE * MERC_R * std::log(std::tan(M_PI / 4.0 + latRad / 2.0));
            
            // 流速和流向直接映射
            anchor.floodSpeed = query.value(3).toDouble();
            anchor.floodDir = query.value(4).toDouble();
            
            // 参考水深
            anchor.refDepth = query.value(5).toDouble();
            
            // 既然已经明确该水域为无潮汐恒定单向流，type 赋予默认值即可
            anchor.type = 0;
            
            anchors.append(anchor);
        }
    } else {
        qDebug() << "❌ 场源锚点数据读取失败:" << query.lastError().text();
        return;
    }
    
    setTidalSources(anchors);
    qDebug() << "✅ 从数据库成功加载并投影了" << anchors.size() << "个恒定流场锚点";
}

void FlowField::calculateDynamicRBFParam() {
    if (m_febSources.size() < 2) {
        qDebug() << "⚠️ 观测站数量不足，使用默认RBF参数:" << m_rbfShapeParam;
        return;
    }

    QVector<FEBPoint> floodSources;
    for (const auto& feb : m_febSources) {
        if (feb.floodSpeed > 0.1) {
            floodSources.append(feb);
        }
    }

    if (floodSources.size() < 2) {
        qDebug() << "⚠️ 涨潮观测站数量不足，使用默认RBF参数:" << m_rbfShapeParam;
        return;
    }

    std::sort(floodSources.begin(), floodSources.end(), [](const FEBPoint& a, const FEBPoint& b) {
        return a.x < b.x;
    });

    QVector<double> distances;
    for (int i = 0; i < floodSources.size() - 1; ++i) {
        double dx = floodSources[i].x - floodSources[i + 1].x;
        double dy = floodSources[i].y - floodSources[i + 1].y;
        double dist = qSqrt(dx * dx + dy * dy);
        distances.append(dist);
        qDebug() << "   - FEB" << floodSources[i].id << "到FEB" << floodSources[i + 1].id
                 << "距离:" << dist / 1000.0 << "km";
    }

    std::sort(distances.begin(), distances.end());

    double minDist = distances.first();
    double maxDist = distances.last();
    double avgDist = 0;
    for (double d : distances) {
        avgDist += d;
    }
    avgDist /= distances.size();

    double medianDist;
    int mid = distances.size() / 2;
    if (distances.size() % 2 == 0) {
        medianDist = (distances[mid - 1] + distances[mid]) / 2.0;
    } else {
        medianDist = distances[mid];
    }

    m_rbfShapeParam = qBound(1000.0, medianDist * 1.2, 5000.0);

    qDebug() << "📊 动态RBF参数计算结果 (涨潮观测站):";
    qDebug() << "   - 涨潮观测站数量:" << floodSources.size();
    qDebug() << "   - 最小间距:" << minDist / 1000.0 << "km";
    qDebug() << "   - 最大间距:" << maxDist / 1000.0 << "km";
    qDebug() << "   - 平均间距:" << avgDist / 1000.0 << "km";
    qDebug() << "   - 中位数间距:" << medianDist / 1000.0 << "km";
    qDebug() << "   - RBF形状参数:" << m_rbfShapeParam / 1000.0 << "km (限制在1-5km)";
}

void FlowField::buildDepthGrid() {
    m_depthGrid.clear();
    for (int i = 0; i < m_soundings.size(); ++i) {
        m_depthGrid[getGridKey(m_soundings[i].x, m_soundings[i].y)].append(i);
    }
}

uint64_t FlowField::getGridKey(double x, double y) const {
    int64_t gx = static_cast<int64_t>(qFloor(x / m_gridSize));
    int64_t gy = static_cast<int64_t>(qFloor(y / m_gridSize));
    return (static_cast<uint64_t>(gx) << 32) | (static_cast<uint32_t>(gy) & 0xFFFFFFFF);
}

double FlowField::getInterpolatedDepthOptimized(double x, double y) const {
    if (m_soundings.isEmpty()) return -1.0;

    double totalWeight = 0.0;
    double weightedDepth = 0.0;
    int cellX = qFloor(x / m_gridSize);
    int cellY = qFloor(y / m_gridSize);

    const double epsilonSq = 100.0;
    int pointsCount = 0;
    const int minPointsRequired = 6;

    for (int radius = 1; radius <= 4.5; ++radius) {
        for (int i = -radius; i <= radius; ++i) {
            for (int j = -radius; j <= radius; ++j) {
                uint64_t searchKey = (static_cast<uint64_t>(cellX + i) << 32) |
                                     (static_cast<uint32_t>(cellY + j) & 0xFFFFFFFF);

                if (m_depthGrid.contains(searchKey)) {
                    for (int idx : m_depthGrid[searchKey]) {
                        const auto& sp = m_soundings[idx];
                        double d2 = qPow(x - sp.x, 2) + qPow(y - sp.y, 2);

                        double weight = 1.0 / (d2 + epsilonSq);
                        totalWeight += weight;
                        weightedDepth += weight * sp.depth;
                        pointsCount++;
                    }
                }
            }
        }
        if (pointsCount >= minPointsRequired) break;
    }

    if (totalWeight > 0) {
        return weightedDepth / totalWeight;
    }

    return -1.0;
}

// 【新增/恢复】 设置体育场形桥墩
void FlowField::setIdealizedPylons(const QVector<IdealizedPylon>& pylons) {
    m_idealizedPylons = pylons;
    qDebug() << "✅ 已启用体育场形计算模型，共" << pylons.size() << "个桥墩";
}

QPointF FlowField::computePylonPerturbation(double x, double y, double t, double referenceSpeed, double flowAngle) const {
    if (referenceSpeed < 0.01 || m_idealizedPylons.isEmpty()) return QPointF(0, 0);

    QPointF perturbation(0, 0);
    
    // 1. 重构基础流速向量 (Base Velocity Vector)
    double vx_base = referenceSpeed * std::cos(flowAngle);
    double vy_base = referenceSpeed * std::sin(flowAngle);

    // 扰动参数
    double wakeLength = 800.0; // 尾迹拖多长 (米)
    double wakeWidth = 150.0;  // 尾迹初始宽度 (米)

    for(const auto& p : m_idealizedPylons) {
        // --- 几何计算 (保持固定东西向) ---
        QPointF p1(p.x - p.Offset, p.y);
        QPointF p2(p.x + p.Offset, p.y);
        QPointF pt(x, y);

        // 计算点到骨架的距离
        double dx_skel = p2.x() - p1.x();
        double dy_skel = p2.y() - p1.y();
        double l2 = dx_skel*dx_skel + dy_skel*dy_skel;
        double u = ((x - p1.x()) * dx_skel + (y - p1.y()) * dy_skel) / l2;
        u = qBound(0.0, u, 1.0);
        
        // 最近点
        double projX = p1.x() + u * dx_skel;
        double projY = p1.y() + u * dy_skel;
        
        // 法线向量 (从桥墩指向水流)
        double nx = x - projX;
        double ny = y - projY;
        double dist = std::sqrt(nx*nx + ny*ny);
        
        // 归一化法线
        if (dist > 1e-6) {
            nx /= dist;
            ny /= dist;
        }

        double distSurface = dist - p.R_head;

        // =========================================================
        // 【核心修改】 替换原来的"弹力"，改用"切向滑移"
        // =========================================================
        double margin = 300.0; // 影响范围
        
        if (distSurface < margin && dist > 1.0) {
            // 计算"过渡因子" (0.0 ~ 1.0)
            // 在表面处为1.0 (完全滑移)，在边界处为0.0 (无影响)
            double factor = (margin - distSurface) / margin;
            if (factor < 0) factor = 0;
            // 使用平滑过渡 (Smoothstep) 效果更好
            factor = factor * factor * (3.0 - 2.0 * factor);

            // 1. 计算水流在法线方向上的投影分量 (Normal Velocity Component)
            // v_normal = V_base · Normal
            double v_normal = vx_base * nx + vy_base * ny;

            // 2. 只有当水流是"流向墙面" (v_normal < 0) 时，才需要抵消
            if (v_normal < 0) {
                // 生成抵消分量：大小等于 v_normal，方向沿法线向外
                // 乘上 factor 实现平滑过渡
                perturbation.rx() += -v_normal * nx * factor;
                perturbation.ry() += -v_normal * ny * factor;
            }
        }

        // =========================================================
        // 下面是尾迹/涡旋逻辑 (保持不变，用于生成漂亮的尾巴)
        // =========================================================
        // 投影计算
        double cosFlow = std::cos(flowAngle);
        double sinFlow = std::sin(flowAngle);
        double dx_center = x - p.x;
        double dy_center = y - p.y;
        double streamDist = dx_center * cosFlow + dy_center * sinFlow;
        double crossDist = -dx_center * sinFlow + dy_center * cosFlow;

        if (streamDist > 0 && streamDist < wakeLength) {
            double distFactor = 1.0 - (streamDist / wakeLength);
            double wakeIntensity = distFactor * distFactor;
            double widthSigma = p.R_head * 1.5;
            double crossFactor = std::exp(-(crossDist * crossDist) / (widthSigma * widthSigma));

            if (crossFactor > 0.01) {
                double St = 0.2;
                double D = p.R_head * 2.0;
                double f_v = St * referenceSpeed / D;
                double phase = 2 * M_PI * f_v * t - (streamDist / (referenceSpeed * 0.8));
                double v_vortex = 0.5 * referenceSpeed * wakeIntensity * crossFactor * std::sin(phase);
                
                perturbation.rx() += v_vortex * (-sinFlow);
                perturbation.ry() += v_vortex * (cosFlow);
            }
        }
    }
    return perturbation;
}

// 计算点到多边形的SDF
FlowField::SDFResult FlowField::getPolygonSDF(const QPointF& p, const ChannelPoly& poly) const {
    if (poly.points.isEmpty()) {
        return { 99999.0, QPointF(0, 0) };
    }

    double minDistSq = std::numeric_limits<double>::max();
    QPointF bestNormal(0, 0);

    // 遍历多边形的所有边
    for (int i = 0; i < poly.points.size(); ++i) {
        QPointF p1 = poly.points[i];
        QPointF p2 = poly.points[(i + 1) % poly.points.size()];

        QPointF edge = p2 - p1;
        double edgeLenSq = edge.x() * edge.x() + edge.y() * edge.y();
        if (edgeLenSq < 1e-6) continue;

        // 计算点在边上的投影参数t
        double t = QPointF::dotProduct(p - p1, edge) / edgeLenSq;
        t = qBound(0.0, t, 1.0); // 限制在[0,1]范围内

        QPointF closest = p1 + t * edge;
        QPointF diff = p - closest;
        double distSq = diff.x() * diff.x() + diff.y() * diff.y();

        if (distSq < minDistSq) {
            minDistSq = distSq;
            // 计算法向量 (指向外部)
            if (distSq > 1e-8) {
                bestNormal = diff / std::sqrt(distSq);
            } else {
                // 极度接近表面，用法向近似 (旋转90度)
                bestNormal = QPointF(-edge.y(), edge.x()) / std::sqrt(edgeLenSq);
            }
        }
    }

    return { std::sqrt(minDistSq), bestNormal };
}

// 计算沙洲扰动
QPointF FlowField::computeSandBarPerturbation(double x, double y, double t, const QPointF& baseFlow) const {
    QPointF totalPerturbation(0, 0);
    QPointF p(x, y);

    // 假设影响范围是500米 (沙洲比较大，影响范围设大一点)
    double influenceRange = 500.0;

    for (const auto& sandBar : m_sandBars) {
        SDFResult sdf = getPolygonSDF(p, sandBar);

        // 只处理沙洲外部、且在影响范围内的点
        if (sdf.distance > 0 && sdf.distance < influenceRange) {
            // 计算背景流在沙洲法线上的分量
            // v_n < 0 表示流速正对着沙洲冲过来 (需要被推开)
            double v_n = QPointF::dotProduct(baseFlow, sdf.normal);

            if (v_n < 0) {
                // 衰减系数：离沙洲越近，修正越强 (平方衰减)
                double decay = std::pow(1.0 - sdf.distance / influenceRange, 2.0);

                // 施加反向推力，抵消穿透分量，实现滑移
                QPointF v_corr = -v_n * sdf.normal * decay;

                totalPerturbation += v_corr;
            }
        }
    }

    return totalPerturbation;
}

double FlowField::getDistanceToNearestShore(double x, double y) const {
    if (m_shorelines.isEmpty()) return 1e6;

    QPointF target(x, y);
    double minDist = 1e6;
    double minDistSq = 1e12; // 使用平方距离比较，减少开方运算

    for (int i = 0; i < m_shorelines.size(); ++i) {
        const QRectF& rect = m_shorelineBounds[i];

        // 【新增】 包围盒快速剔除
        // 计算点到矩形的最小欧氏距离
        // 如果 点到包围盒的距离 已经大于 当前找到的最小距离，则这一整条岸线都不用算了
        double dx = qMax(0.0, qMax(rect.left() - x, x - rect.right()));
        double dy = qMax(0.0, qMax(rect.top() - y, y - rect.bottom()));
        double distToRectSq = dx*dx + dy*dy;

        if (distToRectSq >= minDistSq) continue; // ⏭️ 跳过！

        // 只有当可能更近时，才进行昂贵的线段距离计算
        const auto& shoreline = m_shorelines[i];
        for (int j = 0; j < shoreline.size() - 1; ++j) {
            double dist = pointToSegmentDistance(target, shoreline[j], shoreline[j + 1]);
            if (dist < minDist) {
                minDist = dist;
                minDistSq = dist * dist;
            }
        }
    }

    return minDist;
}

double FlowField::pointToSegmentDistance(const QPointF& p, const QPointF& a, const QPointF& b) const {
    QPointF ab = b - a;
    QPointF ap = p - a;

    double abLenSq = ab.x() * ab.x() + ab.y() * ab.y();
    if (abLenSq < 1e-10) {
        return qSqrt(ap.x() * ap.x() + ap.y() * ap.y());
    }

    double t = (ap.x() * ab.x() + ap.y() * ab.y()) / abLenSq;
    t = qMax(0.0, qMin(1.0, t));

    QPointF projection = a + ab * t;
    QPointF diff = p - projection;
    return qSqrt(diff.x() * diff.x() + diff.y() * diff.y());
}

void FlowField::setBackwaterScenario(const BackwaterScenario& scenario) {
    QMutexLocker locker(&m_mutex);
    m_backwater = scenario;
    m_backwater.amplitude = qBound(0.0, scenario.amplitude, 0.95); // β < 1，流向不反转
    if (m_rbfInitialized) updateRBFWeights(m_lastRBFUpdateTime);
}

double FlowField::backwaterFactor(const FEBPoint& pt, double t) const {
    if (!m_backwater.enabled || m_backwater.amplitude <= 0.0) return 1.0;
    // 未参与中心线校准的锚点视为位于最下游（无滞后）
    double s = m_anchorAlongChannel.value(pt.id, m_maxAlongChannel);
    double lag = (m_maxAlongChannel - s) / m_backwater.celerity;
    double omega = 2.0 * M_PI / m_backwater.period;
    return 1.0 - m_backwater.amplitude * std::cos(omega * (t - lag) + m_backwater.phase);
}

QPointF FlowField::calculateTidalVector(const FEBPoint& pt, double t) const {
    // 1. 以径流为主的恒定流；启用顶托情景时按锚点位置叠加周期调制
    double currentSpeed = pt.floodSpeed * backwaterFactor(pt, t);

    // 2. 坐标转换 (地理北 -> 数学笛卡尔)
    // S-57: 0=North, 90=East
    // Math: 0=East, 90=North
    double angleRad = (90.0 - pt.floodDir) * M_PI / 180.0;

    return QPointF(currentSpeed * qCos(angleRad), currentSpeed * qSin(angleRad));
}

double FlowField::rbfKernel(double r) const {
    double c = m_rbfShapeParam;
    return std::exp(-(r * r) / (c * c));
}

double FlowField::calculateAnisotropicDist(double dx, double dy) const {
    double cosA = std::cos(m_flowAngleRad);
    double sinA = std::sin(m_flowAngleRad);

    double dx_prime = dx * cosA + dy * sinA;
    double dy_prime = -dx * sinA + dy * cosA;

    double stretch_long = 1.0;
    double stretch_trans = 2.0;

    return std::sqrt(qPow(dx_prime / stretch_long, 2) + qPow(dy_prime / stretch_trans, 2));
}

void FlowField::solveRBFWeights(double t) {
    QMutexLocker locker(&m_mutex);

    m_rbfNodes.clear();
    m_rbfStations.clear();
    m_rbfMatrix.clear();

    QVector<QPointF> centers;

    auto isTooClose = [&](const QPointF& p) {
        for (const auto& c : centers) {
            double dx = p.x() - c.x();
            double dy = p.y() - c.y();
            if (dx*dx + dy*dy < 1.0) return true;
        }
        return false;
    };

    QVector<FEBPoint> activeStations;

    for (const auto& feb : m_febSources) {
        // 1. 基础过滤：剔除流速极小（无数据）的无效点
        if (feb.floodSpeed < 0.05) continue;

        // 🌟 2. 删除了所有基于 feb.type 的涨落潮动态过滤逻辑
        // 所有的控制点现在同时、永久生效，构建全局唯一的静态背景流

        QPointF pos(feb.x, feb.y);
        if (isTooClose(pos)) continue;

        activeStations.append(feb);
    }

    std::sort(activeStations.begin(), activeStations.end(), [](const FEBPoint& a, const FEBPoint& b) {
        return a.x < b.x;
    });

    QVector<double> distances;
    for (int i = 0; i < activeStations.size() - 1; ++i) {
        double dx = activeStations[i+1].x - activeStations[i].x;
        double dy = activeStations[i+1].y - activeStations[i].y;
        double dist = std::sqrt(dx*dx + dy*dy);
        distances.append(dist);
    }

    if (!distances.isEmpty()) {
        std::sort(distances.begin(), distances.end());
        double medianDist = distances[distances.size() / 2];
        m_rbfShapeParam = qBound(2000.0, medianDist * 2.0, 100000.0);
        m_rbfDepthWeight = m_rbfShapeParam * 0.02;
    } else {
        m_rbfShapeParam = 10000.0;
        m_rbfDepthWeight = 200.0;
    }

    for (const auto& feb : activeStations) {
        centers.append(QPointF(feb.x, feb.y));
    }

    int N = centers.size();
    if (N == 0) {
        qDebug() << "⚠️ RBF: 没有观测点，无法计算权重";
        return;
    }

    // 核矩阵（含 Tikhonov 正则项 0.1）只与锚点位置和水深有关，求一次后保存
    m_rbfMatrix = QVector<QVector<double>>(N, QVector<double>(N, 0.0));
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            double dist2d = calculateAnisotropicDist(centers[i].x() - centers[j].x(),
                                                     centers[i].y() - centers[j].y());
            double dz = activeStations[i].refDepth - activeStations[j].refDepth;
            double dist3d = std::sqrt(dist2d * dist2d + std::pow(m_rbfDepthWeight * dz, 2));
            m_rbfMatrix[i][j] = rbfKernel(dist3d);
        }
        m_rbfMatrix[i][i] += 0.1;
    }
    m_rbfStations = activeStations;

    updateRBFWeights(t);

    qDebug() << "✅ RBF 权重计算完成 (已正则化)";
    qDebug() << "   时刻 t=" << t;
    qDebug() << "   参与计算站点数:" << m_rbfNodes.size();
}

void FlowField::updateRBFWeights(double t) {
    QMutexLocker locker(&m_mutex);

    int N = m_rbfStations.size();
    if (N == 0) return;

    QVector<QPointF> vels(N);
    for (int i = 0; i < N; ++i) {
        vels[i] = calculateTidalVector(m_rbfStations[i], t);
    }

    QVector<std::complex<double>> weights(N, std::complex<double>(0.0, 0.0));

    for (int part = 0; part < 2; ++part) {
        QVector<double> b(N, 0.0);
        for (int i = 0; i < N; ++i) {
            b[i] = (part == 0) ? vels[i].x() : vels[i].y();
        }

        QVector<QVector<double>> Mat = m_rbfMatrix;

        for (int k = 0; k < N; ++k) {
            int maxRow = k;
            for (int i = k + 1; i < N; ++i) {
                if (std::abs(Mat[i][k]) > std::abs(Mat[maxRow][k])) {
                    maxRow = i;
                }
            }
            std::swap(Mat[k], Mat[maxRow]);
            std::swap(b[k], b[maxRow]);

            if (std::abs(Mat[k][k]) < 1e-12) Mat[k][k] = 1e-12;

            for (int i = k + 1; i < N; ++i) {
                double factor = Mat[i][k] / Mat[k][k];
                for (int j = k; j < N; ++j) {
                    Mat[i][j] -= factor * Mat[k][j];
                }
                b[i] -= factor * b[k];
            }
        }

        QVector<double> x(N, 0.0);
        for (int i = N - 1; i >= 0; --i) {
            double sum = 0.0;
            for (int j = i + 1; j < N; ++j) {
                sum += Mat[i][j] * x[j];
            }
            x[i] = (b[i] - sum) / Mat[i][i];
        }

        for (int i = 0; i < N; ++i) {
            if (part == 0) {
                weights[i].real(x[i]);
            } else {
                weights[i].imag(x[i]);
            }
        }
    }

    m_rbfNodes.clear();
    for (int i = 0; i < N; ++i) {
        m_rbfNodes.append({QPointF(m_rbfStations[i].x, m_rbfStations[i].y), m_rbfStations[i].refDepth, weights[i]});
    }

    m_rbfInitialized = true;
    m_lastRBFUpdateTime = t;
}

QPointF FlowField::rbfInterpolate(double x, double y) const {
    if (!m_rbfInitialized || m_rbfNodes.isEmpty()) {
        return QPointF(0, 0);
    }

    double localDepth = getDepthAt(x, y);

    // 【清理后】纯粹的深水航道兜底逻辑
    if (localDepth < 0.5) {
        if (isInDepare(x, y)) {
            localDepth = 5.0; // 处于航道面但缺散点，给默认可流通水深
        } else {
            return QPointF(0, 0); // 彻底拒绝插值
        }
    }

    double u = 0, v = 0;

    int nodeIndex = 0;
    for (const auto& node : m_rbfNodes) {
        double dx = x - node.pos.x();
        double dy = y - node.pos.y();
        double dz = localDepth - node.depth;
        double dist2d = calculateAnisotropicDist(dx, dy);
        double dist3d = std::sqrt(dist2d * dist2d + std::pow(m_rbfDepthWeight * dz, 2));
        double phi = rbfKernel(dist3d);

        double contribU = node.weight.real() * phi;
        double contribV = node.weight.imag() * phi;

        u += contribU;
        v += contribV;

        nodeIndex++;
    }

    return QPointF(u, v);
}

double FlowField::calculateAverageSourceSpeed(double t) const {
    if (m_febSources.isEmpty()) {
        return 0.0;
    }

    double totalSpeed = 0.0;
    int validCount = 0;

    for (const auto& source : m_febSources) {
        QPointF tidalVector = calculateTidalVector(source, t);
        double speed = std::sqrt(tidalVector.x() * tidalVector.x() + tidalVector.y() * tidalVector.y());
        totalSpeed += speed;
        validCount++;
    }

    return validCount > 0 ? totalSpeed / validCount : 0.0;
}





void FlowField::setRecordingMode(bool enable) {
    m_isRecording = enable;
    if (m_isRecording) {
        m_isPlayback = false;
        initRecordTable();
        qDebug() << "⏺️ 流场录制模式已开启";
    }
}

void FlowField::setPlaybackMode(bool enable) {
    m_isPlayback = enable;
    if (m_isPlayback) {
        m_isRecording = false;

        if (!m_recordDb.isOpen()) {
            m_recordDb = QSqlDatabase::addDatabase("QSQLITE", "flow_playback_conn");
            m_recordDb.setDatabaseName(QCoreApplication::applicationDirPath() + "/FlowRecord.db");
            if (!m_recordDb.open()) {
                qDebug() << "❌ 无法打开回放数据库:" << m_recordDb.lastError().text();
                m_isPlayback = false;
            }
        }
        qDebug() << "▶️ 流场回放模式已开启 (优先读取数据库)";
    }
}

void FlowField::initRecordTable() {
    QString dbPath = QCoreApplication::applicationDirPath() + "/FlowRecord.db";
    m_recordDb = QSqlDatabase::addDatabase("QSQLITE", "flow_record_conn");
    m_recordDb.setDatabaseName(dbPath);

    if (m_recordDb.open()) {
        QSqlQuery query(m_recordDb);
        query.exec("CREATE TABLE IF NOT EXISTS FLOW_CACHE ("
                   "sim_time INTEGER, "
                   "grid_x INTEGER, "
                   "grid_y INTEGER, "
                   "u DOUBLE, v DOUBLE, "
                   "PRIMARY KEY (sim_time, grid_x, grid_y))");
    } else {
        qDebug() << "❌ 无法创建录制数据库";
    }
}

void FlowField::initSnapshotTable() {
    QString dbPath = QCoreApplication::applicationDirPath() + "/FlowSnapshot.db";
    m_snapshotDb = QSqlDatabase::addDatabase("QSQLITE", "snapshot_conn");
    m_snapshotDb.setDatabaseName(dbPath);

    if (m_snapshotDb.open()) {
        QSqlQuery query(m_snapshotDb);
        query.exec("CREATE TABLE IF NOT EXISTS SNAPSHOT_FLOW ("
                   "grid_x INTEGER, "
                   "grid_y INTEGER, "
                   "u DOUBLE, v DOUBLE, "
                   "PRIMARY KEY(grid_x, grid_y))");
        qDebug() << "✅ 快照数据库已初始化: FlowSnapshot.db";
    } else {
        qDebug() << "❌ 无法打开快照数据库:" << m_snapshotDb.lastError().text();
    }
}

void FlowField::saveToCache(double t, double x, double y, double u, double v) {
    if (!m_recordDb.isOpen()) return;

    int t_key = static_cast<int>(t / 10.0) * 10;
    // 🌟 将 50.0 改为 20.0，匹配高精度 8 单元需求
    int x_key = static_cast<int>(x / 20.0) * 20;
    int y_key = static_cast<int>(y / 20.0) * 20;

    QSqlQuery query(m_recordDb);
    query.prepare("REPLACE INTO FLOW_CACHE (sim_time, grid_x, grid_y, u, v) VALUES (?, ?, ?, ?, ?)");
    query.addBindValue(t_key);
    query.addBindValue(x_key);
    query.addBindValue(y_key);
    query.addBindValue(u);
    query.addBindValue(v);
    query.exec();
}

bool FlowField::loadFromCache(double t, double x, double y, QPointF& outFlow) {
    if (!m_recordDb.isOpen()) return false;

    int t_key = static_cast<int>(t / 10.0) * 10;
    // 🌟 同步修改为 20.0
    int x_key = static_cast<int>(x / 20.0) * 20;
    int y_key = static_cast<int>(y / 20.0) * 20;

    QSqlQuery query(m_recordDb);
    query.prepare("SELECT u, v FROM FLOW_CACHE WHERE sim_time = ? AND grid_x = ? AND grid_y = ?");
    query.addBindValue(t_key);
    query.addBindValue(x_key);
    query.addBindValue(y_key);

    if (query.exec() && query.next()) {
        outFlow.setX(query.value(0).toDouble());
        outFlow.setY(query.value(1).toDouble());
        return true;
    }
    return false;
}

void FlowField::recordStartingFlowField(std::function<void(int)> progressCallback) {
    // 【修改】涨潮急流 (1/4 周期, Phase = +1.0)
    // 会自动激活涨潮观测站，并使流向变为西北方向
    double t = T_SIMULATION_START;

    if (!m_snapshotDb.isOpen()) initSnapshotTable();

    // 1. 强制更新一次 RBF
    if (!m_febSources.isEmpty()) {
        if (progressCallback) progressCallback(5);
        solveRBFWeights(t);
        m_lastRBFUpdateTime = t;
    }

    qDebug() << "🚀 开始全域自适应流场录制...";

    if (!m_snapshotDb.transaction()) return;

    QSqlQuery query(m_snapshotDb);
    query.prepare("REPLACE INTO SNAPSHOT_FLOW (grid_x, grid_y, u, v) VALUES (:x, :y, :u, :v)");

    bool originalRecordState = m_isRecording;
    m_isRecording = false;

    // 🌟 动态解算扫描范围，取代原先的 LIMIT_LON/LAT 和硬编码 area
    double scanMinX = 1e18, scanMaxX = -1e18, scanMinY = 1e18, scanMaxY = -1e18;
    for (const auto& anchor : m_febSources) {
        scanMinX = qMin(scanMinX, anchor.x); scanMaxX = qMax(scanMaxX, anchor.x);
        scanMinY = qMin(scanMinY, anchor.y); scanMaxY = qMax(scanMaxY, anchor.y);
    }
    for (const auto& rect : m_depareBounds) {
        if (rect.isNull()) continue;
        scanMinX = qMin(scanMinX, rect.left()); scanMaxX = qMax(scanMaxX, rect.right());
        scanMinY = qMin(scanMinY, rect.top());  scanMaxY = qMax(scanMaxY, rect.bottom());
    }
    
    double buffer = 2000.0;
    scanMinX -= buffer; scanMaxX += buffer;
    scanMinY -= buffer; scanMaxY += buffer;

    // =========================================================
    // 【Pass 1】 全局标准录制 (Step = 50m，提升全局弯道平滑度)
    // =========================================================
    double stepGlobal = 50.0;
    int count = 0;

    for (double y = scanMinY; y <= scanMaxY; y += stepGlobal) {
        for (double x = scanMinX; x <= scanMaxX; x += stepGlobal) {
            
            // 【新增】严格剔除桥墩内部！
            // 防止生成"桥墩内的水深5m流场"
            if (isInsidePylon(x, y)) continue;

            // 2. 【关键】如果在桥墩附近 1km，跳过 (留给 Pass 2 处理)
            if (isNearPylon(x, y, 1000.0)) continue;

            // 3. 水域判定 (包含 DEPARE 补全)
            if (!m_waterDomainMask.contains(QPointF(x, y))) continue;
            if (getDepthAt(x, y) < 0.5 && !isInDepare(x, y)) continue; // 如果无水深点且不在DEPARE内，剔除

            // 4. 计算流速
            QPointF flow = computeAt(x, y, t);
            if (qAbs(flow.x()) < 1e-6 && qAbs(flow.y()) < 1e-6) continue;

            query.bindValue(":x", (int)x);
            query.bindValue(":y", (int)y);
            query.bindValue(":u", flow.x());
            query.bindValue(":v", flow.y());
            query.exec();
            count++;
        }
    }

    // =========================================================
    // 【Pass 2】 桥墩/复杂区域精细录制 (Step = 20m)
    // =========================================================
    // 强制赋予桥区 20 米的究极精度，完美触发船体的微观扰动力矩
    double stepFine = 20.0;
    // 【核心修复】将扫描外扩范围从1000.0改为1500.0，确保覆盖Pass 1的剔除范围
    double scanMargin = 1500.0; // 外扩1.5km

    for (const auto& p : m_idealizedPylons) {
        // 构建扫描包围盒: 中心 ± (Offset + R + Margin)
        double halfW = p.Offset + p.R_head + scanMargin;
        double halfH = p.R_head + scanMargin;
        
        QRectF scanArea(p.x - halfW, p.y - halfH, halfW*2, halfH*2);

        for (double y = scanArea.top(); y < scanArea.bottom(); y += stepFine) {
            for (double x = scanArea.left(); x < scanArea.right(); x += stepFine) {
                // 【新增】严格剔除桥墩内部！
                // 防止生成"桥墩内的水深5m流场"
                if (isInsidePylon(x, y)) continue;

                if (!m_waterDomainMask.contains(QPointF(x, y))) continue;
                if (getDepthAt(x, y) < 0.5 && !isInDepare(x, y)) continue;

                QPointF flow = computeAt(x, y, t);
                if (qAbs(flow.x()) < 1e-6 && qAbs(flow.y()) < 1e-6) continue;

                query.bindValue(":x", (int)x);
                query.bindValue(":y", (int)y);
                query.bindValue(":u", flow.x());
                query.bindValue(":v", flow.y());
                query.exec();
                count++;
            }
        }
    }

    m_isRecording = originalRecordState;

    if (progressCallback) progressCallback(100);

    if (m_snapshotDb.commit()) {
        qDebug() << "✅ 录制完成！共计" << count << "个点 (包含桥区" << stepFine << "m 加密 + DEPARE 补全)";
    } else {
        qDebug() << "❌ 事务提交失败:" << m_snapshotDb.lastError().text();
        m_snapshotDb.rollback();
    }
}

bool FlowField::isInDepare(double x, double y) const {
    QPointF p(x, y);
    for (int i = 0; i < m_deparePolygons.size(); ++i) {
        // 【新增】 包围盒快速剔除
        if (!m_depareBounds[i].contains(p)) {
            continue; // ⏭️ 如果不在包围盒内，绝对不可能在多边形内
        }

        // 精确判定
        // 外环内且不落在任何内环（洞）里才算水深面之内
        const ChannelPoly& dp = m_deparePolygons[i];
        if (!QPolygonF(dp.points).containsPoint(p, Qt::OddEvenFill)) continue;
        bool inHole = false;
        for (int k = 1; k < dp.rings.size(); ++k) {
            if (dp.rings[k].size() > 2 && QPolygonF(dp.rings[k]).containsPoint(p, Qt::OddEvenFill)) {
                inHole = true;
                break;
            }
        }
        if (!inHole) return true;
    }
    return false;
}

bool FlowField::isNearPylon(double x, double y, double threshold) const {
    for (const auto& pylon : m_pylons) {
        if (pylon.points.isEmpty()) continue;
        // 快速包围盒检测
        double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
        for(const auto& p : pylon.points) {
            minX = qMin(minX, p.x()); maxX = qMax(maxX, p.x());
            minY = qMin(minY, p.y()); maxY = qMax(maxY, p.y());
        }
        // 在包围盒外扩 threshold 范围内
        if (x >= minX - threshold && x <= maxX + threshold &&
            y >= minY - threshold && y <= maxY + threshold) {
            return true;
        }
    }
    return false;
}

bool FlowField::isInsidePylon(double x, double y) const {
    // 优先使用高精度的体育场模型判定
    for (const auto& p : m_idealizedPylons) {
        // 计算点到骨架线段的距离
        QPointF pt(x, y);
        QPointF p1(p.x - p.Offset, p.y);
        QPointF p2(p.x + p.Offset, p.y);
        
        double dist = pointToSegmentDistance(pt, p1, p2);
        
        // 如果距离小于半径，就在内部
        if (dist < p.R_head) return true;
    }
    
    return false;
}

QPointF FlowField::computeAt(double x, double y, double t) {
    QMutexLocker locker(&m_mutex); // 加锁保护整个计算过程，防止多线程冲突

    // 顶托情景：按仿真时间刷新 RBF 权重（核矩阵不变，只重解 N 维线性方程组）
    if (m_backwater.enabled && m_rbfInitialized &&
        std::fabs(t - m_lastRBFUpdateTime) >= m_backwater.updateInterval) {
        updateRBFWeights(t);
    }
    
    // 1. 【最高优先级】 严格剔除桥墩内部 (体育场模型)
    if (isInsidePylon(x, y)) return QPointF(0, 0);

    // 2. 基础掩码检查 (排除岸线和陆地)
    if (!m_waterDomainMask.contains(QPointF(x, y))) return QPointF(0, 0);

    // 3. 全域统一水深与深水航道判定
    double depth = getDepthAt(x, y);
    if (depth < 0.5 && !isInDepare(x, y)) {
        return QPointF(0, 0);
    }

    if (m_isPlayback) {
        QPointF cachedFlow;
        if (loadFromCache(t, x, y, cachedFlow)) {
            return cachedFlow;
        }
    }

    // 计算基础流场（RBF插值）
    QPointF baseFlow;
    if (!m_febSources.isEmpty()) {
        baseFlow = rbfInterpolate(x, y);
    } else {
        return QPointF(0, 0);
    }

    // 使用当前点的 RBF 背景流速作为参考流速
    double referenceSpeed = std::sqrt(baseFlow.x()*baseFlow.x() + baseFlow.y()*baseFlow.y());
    
    // 计算实时流向角度 (弧度)
    // 这是“水流原本想去的方向”
    double currentFlowAngle = 0.0;
    if (referenceSpeed > 1e-6) {
        currentFlowAngle = std::atan2(baseFlow.y(), baseFlow.x());
    } else {
        currentFlowAngle = m_flowAngleRad; // 静止时用默认值
    }

    // 计算桥墩扰动（几何排斥算法）
    QPointF perturbation = computePylonPerturbation(x, y, t, referenceSpeed, currentFlowAngle);
    
    // 计算沙洲扰动（SDF滑移边界条件）
    QPointF sandBarPerturbation = computeSandBarPerturbation(x, y, t, baseFlow);
    
    // 合并所有扰动
    QPointF finalFlow = baseFlow + perturbation + sandBarPerturbation;

    // 速度限制（防止过大）
    // 在 20m 网格精度下，降低最大流速限制，防止船体受力高频震荡
    double speedSq = finalFlow.x()*finalFlow.x() + finalFlow.y()*finalFlow.y();
    if (speedSq > 25.0) { // 原为 36.0，改为 25.0 (约5节流速上限)，更加符合长江真实水文
        finalFlow *= (5.0 / std::sqrt(speedSq));
    }

    // 录制模式下保存到缓存
    if (m_isRecording) {
        saveToCache(t, x, y, finalFlow.x(), finalFlow.y());
    }

    return finalFlow;
}

void FlowField::autoAlignAnchorDirections(const QVector<QPointF>& referencePath) {
    if (referencePath.size() < 2 || m_febSources.isEmpty()) {
        qDebug() << "⚠️ 缺乏参考航道或锚点，无法执行流向自适应校准";
        return;
    }

    // 中心线累计里程，用于顶托情景中计算潮波自下游传至各锚点的时间
    QVector<double> cumLength(referencePath.size(), 0.0);
    for (int i = 1; i < referencePath.size(); ++i) {
        double dx = referencePath[i].x() - referencePath[i-1].x();
        double dy = referencePath[i].y() - referencePath[i-1].y();
        cumLength[i] = cumLength[i-1] + std::sqrt(dx * dx + dy * dy);
    }
    m_anchorAlongChannel.clear();
    m_maxAlongChannel = 0.0;

    int alignedCount = 0;
    for (auto& anchor : m_febSources) {
        QPointF pos(anchor.x, anchor.y);
        double minDistSq = 1e18;
        int bestIdx = 0;

        // 1. 寻找参考线上距离该锚点最近的节点
        for (int i = 0; i < referencePath.size() - 1; ++i) {
            double dx = pos.x() - referencePath[i].x();
            double dy = pos.y() - referencePath[i].y();
            double distSq = dx * dx + dy * dy;
            
            if (distSq < minDistSq) {
                minDistSq = distSq;
                bestIdx = i;
            }
        }

        // 2. 获取该节点处的切线方向 (向后展望一小段距离，避免微观抖动)
        // 中心线插值了 1000 个点，非常密集，所以往后看 5 个点作为矢量方向最稳定
        int targetIdx = qMin(bestIdx + 5, (int)referencePath.size() - 1);
        QPointF p1 = referencePath[bestIdx];
        QPointF p2 = referencePath[targetIdx];

        double dx = p2.x() - p1.x();
        double dy = p2.y() - p1.y(); // 注意：墨卡托坐标中，y 向上增加(北)

        // 3. 转换为 S-57 航海角度 (0=北, 90=东, 顺时针)
        // atan2(dx, dy) 正好满足 Y 轴为基准顺时针旋转的数学特性
        double bearing = std::atan2(dx, dy) * 180.0 / M_PI;
        if (bearing < 0) bearing += 360.0;

        m_anchorAlongChannel[anchor.id] = cumLength[bestIdx];
        m_maxAlongChannel = qMax(m_maxAlongChannel, cumLength[bestIdx]);

        // 4. 强制覆盖数据库里的流向
        double oldDir = anchor.floodDir;
        anchor.floodDir = bearing;
        
        qDebug() << QString("⚓ 锚点 ID:%1 | 原流向: %2° -> 修正后顺流向: %3° | 沿航道里程: %4 m")
                    .arg(anchor.id).arg(oldDir, 0, 'f', 1).arg(bearing, 0, 'f', 1)
                    .arg(cumLength[bestIdx], 0, 'f', 0);
        
        alignedCount++;
    }

    qDebug() << "✅ 已成功根据弯道曲率自适应校准" << alignedCount << "个场源锚点的流向！";
}
