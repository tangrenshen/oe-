#ifndef LINESTRING_GROUPING_H
#define LINESTRING_GROUPING_H

#include <QVector>
#include <QPointF>
#include <QSqlDatabase>
#include <QSqlQuery>

// 线特性结构体
struct LineFeature {
    QVector<QPointF> line;   // 线的点集
    QString rcid;            // RCID标识
    QString lnam;            // LNAM标识
    QString grup;            // GRUP标识
    QString type;            // 线类型
};

// 墨卡托投影转换函数（需要根据实际实现调整）
extern QPointF mercatorPoint(double lon, double lat);

// 辅助函数：解析WKT点字符串
bool parseWktPoint(const QString &wkt, double &lon, double &lat) {
    // 检查是否是POINT类型
    if (!wkt.startsWith("POINT(", Qt::CaseInsensitive)) {
        return false;
    }
    
    // 提取坐标部分
    QString coordPart = wkt.mid(6); // 去掉 "POINT("
    coordPart = coordPart.left(coordPart.length() - 1); // 去掉最后的 ")"
    
    // 分割经纬度
    QStringList coords = coordPart.split(' ', Qt::SkipEmptyParts);
    if (coords.size() != 2) {
        return false;
    }
    
    // 转换为数值
    bool ok1, ok2;
    lon = coords[0].toDouble(&ok1);
    lat = coords[1].toDouble(&ok2);
    
    return ok1 && ok2;
}

// 通用的linestring分组函数，确保RCID和LNAM一致时才连接点
QVector<LineFeature> fetchLinesWithRcidLnamGrouping(QSqlDatabase& db, const QString& table, const QString& type) {
    QVector<LineFeature> out;
    QSqlQuery query(db);
    
    // 查询数据并按RCID、LNAM、id排序
    query.exec(QString("SELECT RCID, LNAM, GRUP, geometry_wkt FROM %1 ORDER BY RCID, LNAM, id").arg(table));
    
    QString lastRcid;
    QString lastLnam;
    QString lastGrup;
    QVector<QPointF> pts;
    
    while (query.next()) {
        QString rcid = query.value(0).toString();
        QString lnam = query.value(1).toString();
        QString grup = query.value(2).toString();
        QString geometry_wkt = query.value(3).toString();
        
        // 解析WKT获取经纬度
        double lon, lat;
        if (!parseWktPoint(geometry_wkt, lon, lat)) {
            continue; // 解析失败，跳过
        }
        
        // 如果RCID、LNAM或GRUP变化，则输出前一个line
        if ((!lastRcid.isEmpty() || !lastLnam.isEmpty() || !lastGrup.isEmpty()) && 
            (rcid != lastRcid || lnam != lastLnam || grup != lastGrup) && pts.size() >= 2) {
            LineFeature f;
            f.line = pts;
            f.rcid = lastRcid;
            f.lnam = lastLnam;
            f.grup = lastGrup;
            f.type = type;
            out.append(f);
            pts.clear();
        }
        
        lastRcid = rcid;
        lastLnam = lnam;
        lastGrup = grup;
        pts.append(mercatorPoint(lon, lat));
    }
    
    // 输出最后一个line
    if (pts.size() >= 2) {
        LineFeature f;
        f.line = pts;
        f.rcid = lastRcid;
        f.lnam = lastLnam;
        f.grup = lastGrup;
        f.type = type;
        out.append(f);
    }
    
    return out;
}

#endif // LINESTRING_GROUPING_H