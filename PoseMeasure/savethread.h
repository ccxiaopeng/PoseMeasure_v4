#pragma once

#include <QThread>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include <QImage>
#include <QString>
#include <QAtomicInt>

// 保存任务结构体
struct SaveTask {
    QByteArray imageData;  // 图像数据
    int width;             // 图像宽度
    int height;            // 图像高度
    int cameraIndex;       // 相机索引
    qint64 timestamp;      // 时间戳
    QString basePath;      // 保存路径
};

class SaveThread : public QThread
{
    Q_OBJECT

public:
    explicit SaveThread(QObject* parent = nullptr);
    ~SaveThread();

    // 添加保存任务
    void addSaveTask(const SaveTask& task);
    
    // 线程控制
    void start();
    void stop();
    
    // 获取队列状态
    int getQueueSize() const;
    int getTotalSaved() const;

protected:
    void run() override;

private:
    // 保存单个图像
    void saveImage(const SaveTask& task);
    
    // 线程控制
    QAtomicInt m_running;
    
    // 任务队列
    QQueue<SaveTask> m_taskQueue;
    mutable QMutex m_queueMutex;
    QWaitCondition m_queueCondition;
    
    // 统计信息
    QAtomicInt m_totalSaved;
    
    // 颜色表（为了避免重复创建）
    QVector<QRgb> m_grayColourTable;
    
    // 最大队列大小（防止内存溢出）
    static const int MAX_QUEUE_SIZE = 1000;
};
