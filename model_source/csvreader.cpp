#include "csvreader.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QStringList>
#include <QVector>

CSVReader::CSVReader()
{
}

CSVReader::~CSVReader()
{
}

QVector<QVector<AISData>> CSVReader::readAISDataFromCSV(const QString &filePath, const QString &delimiter)
{
    QVector<QVector<AISData>> result;
    QFile file(filePath);
    
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Could not open file:" << filePath;
        return result;
    }
    
    QTextStream in(&file);
    // 建议加上编码设置，防止中文路径乱码（视文件实际编码而定，通常是 UTF-8 或 GBK）
    // in.setCodec("UTF-8");

    bool isFirstLine = true;
    int errorCount = 0; // 错误计数器
    
    // 如果调用者传了空字符串，我们自动纠正为逗号
    QString actualDelimiter = delimiter;
    if (actualDelimiter.isEmpty()) {
        actualDelimiter = ",";
    }
    
    // 列名到索引的映射
    QMap<QString, int> columnMap;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty()) continue;
        
        // 处理标题行，建立列名到索引的映射
        if (isFirstLine) {
            isFirstLine = false;
            
            // 解析标题行，建立列名映射
            QStringList headers = line.split(actualDelimiter);
            for (int i = 0; i < headers.size(); ++i) {
                QString header = headers[i].trimmed().remove('"').toLower();
                columnMap[header] = i;
                // 支持常见的列名变体
                if (header.contains("lon") || header.contains("longitude")) {
                    columnMap["lon"] = i;
                    columnMap["longitude"] = i;
                }
                if (header.contains("lat") || header.contains("latitude")) {
                    columnMap["lat"] = i;
                    columnMap["latitude"] = i;
                }
                if (header.contains("time") || header.contains("timestamp")) {
                    columnMap["time"] = i;
                }
                if (header.contains("mmsi")) {
                    columnMap["mmsi"] = i;
                }
                if (header.contains("speed")) {
                    columnMap["speed"] = i;
                }
                if (header.contains("heading")) {
                    columnMap["heading"] = i;
                }
                if (header.contains("course")) {
                    columnMap["course"] = i;
                }
            }
            
            // 检查必要的列是否存在
            if (!columnMap.contains("time") || !columnMap.contains("mmsi") || 
                !columnMap.contains("lon") || !columnMap.contains("lat")) {
                qDebug() << "必要的列不存在，无法解析AIS数据";
                file.close();
                return result;
            }
            
            continue;
        }
        
        QStringList fields = line.split(actualDelimiter);
        
        // 检查字段数量是否足够
        bool hasEnoughFields = true;
        if (!columnMap.contains("time") || columnMap["time"] >= fields.size()) hasEnoughFields = false;
        if (!columnMap.contains("mmsi") || columnMap["mmsi"] >= fields.size()) hasEnoughFields = false;
        if (!columnMap.contains("lon") || columnMap["lon"] >= fields.size()) hasEnoughFields = false;
        if (!columnMap.contains("lat") || columnMap["lat"] >= fields.size()) hasEnoughFields = false;
        
        if (!hasEnoughFields) {
            // 限制错误输出，防止卡死
            if (errorCount < 10) qDebug() << "Invalid line (insufficient fields):" << line;
            errorCount++;
            continue;
        }
        
        AISData data;
        bool ok;
        
        // 辅助 Lambda：去引号并转 Double
        auto toDoubleClean = [](const QString& str, bool* ok) -> double {
            // 移除可能存在的双引号
            QString s = str;
            s.remove('"');
            return s.toDouble(ok);
        };

        // 解析字段 - 使用列名映射
        data.time = fields[columnMap["time"]].remove('"');
        data.mmsi = fields[columnMap["mmsi"]].remove('"');
        
        // 使用去引号的方式转换
        data.longitude = toDoubleClean(fields[columnMap["lon"]], &ok);
        if (!ok) {
            if(errorCount++ < 10) qDebug() << "Invalid longitude:" << fields[columnMap["lon"]];
            continue;
        }
        
        data.latitude = toDoubleClean(fields[columnMap["lat"]], &ok);
        if (!ok) {
            if(errorCount++ < 10) qDebug() << "Invalid latitude:" << fields[columnMap["lat"]];
            continue;
        }
        
        // 解析可选字段
        if (columnMap.contains("speed") && columnMap["speed"] < fields.size()) {
            data.speed = toDoubleClean(fields[columnMap["speed"]], &ok);
            if (!ok) data.speed = 0.0; // 速度错误设为0，不跳过该行
        } else {
            data.speed = 0.0;
        }
        
        if (columnMap.contains("heading") && columnMap["heading"] < fields.size()) {
            data.heading = toDoubleClean(fields[columnMap["heading"]], &ok);
            if (!ok) data.heading = 0.0;
        } else {
            data.heading = 0.0;
        }
        
        if (columnMap.contains("course") && columnMap["course"] < fields.size()) {
            data.course = toDoubleClean(fields[columnMap["course"]], &ok);
            if (!ok) data.course = 0.0;
        } else {
            data.course = 0.0;
        }
        
        // 目前只支持单船舶轨迹，因此将所有数据放在第一个向量中
        if (result.isEmpty()) {
            result.append(QVector<AISData>());
        }
        result[0].append(data);
    }
    
    file.close();
    
    if (result.isEmpty()) {
        qDebug() << "Read finished but no data found. Total errors:" << errorCount;
    }
    
    return result;
}
