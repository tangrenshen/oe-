#ifndef CSVREADER_H
#define CSVREADER_H

#include <QString>
#include <QVector>
#include "types.h" // 包含AISData定义

class CSVReader {
public:
    CSVReader();
    ~CSVReader();
    
    // 读取AIS数据
    QVector<QVector<AISData>> readAISDataFromCSV(const QString &filePath, const QString &delimiter = ",");
};

#endif // CSVREADER_H