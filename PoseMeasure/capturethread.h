#pragma once

#include <QThread>
#include <QImage>

#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

// 前向声明
class SaveThread;
class MemoryPool;


class CaptureThread : public QThread
{
    Q_OBJECT
public:
    explicit CaptureThread(int cameraIndex = 0, QObject* parent = 0);

public:
    void run();
    void stream();
    void pause();
    void stop();

    // 设置异步保存系统
    void setSaveThread(SaveThread* saveThread);
    void setMemoryPool(MemoryPool* memoryPool);

    void captureROI(int cameraIndex = 0);
    // 新的异步保存ROI方法
    void captureROIAsync(int cameraIndex);
    // 异步坐标获取
    void captureCoordinatePixel(int cameraIndex);

    bool quit;


signals:
    void captured(QImage img, int cameraIndex);
    void coordinateUpdated();
private:
    bool pause_status;
    int m_cameraIndex;  // 指定处理的相机索引
    
    // 异步保存系统
    SaveThread* m_saveThread;
    MemoryPool* m_memoryPool;

    QVector<QRgb> grayColourTable;
    QVector<QRgb> ColourTable;

public slots:

};