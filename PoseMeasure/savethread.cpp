#include "savethread.h"
#include <QDebug>
#include <QDir>
#include <QStandardPaths>

SaveThread::SaveThread(QObject* parent)
    : QThread(parent), m_running(0), m_totalSaved(0)
{
    // 初始化灰度颜色表
    for (int i = 0; i < 256; i++) {
        m_grayColourTable.append(qRgb(i, i, i));
    }
}

SaveThread::~SaveThread()
{
    stop();
    wait();
}

void SaveThread::addSaveTask(const SaveTask& task)
{
    QMutexLocker locker(&m_queueMutex);
    
    // 检查队列大小，防止内存溢出
    if (m_taskQueue.size() >= MAX_QUEUE_SIZE) {
        qWarning() << "Save queue is full, dropping oldest task";
        m_taskQueue.dequeue(); // 移除最老的任务
    }
    
    m_taskQueue.enqueue(task);
    m_queueCondition.wakeOne();
}

void SaveThread::start()
{
    m_running.storeRelaxed(1);
    QThread::start();
}

void SaveThread::stop()
{
    m_running.storeRelaxed(0);
    m_queueCondition.wakeAll();
}

int SaveThread::getQueueSize() const
{
    QMutexLocker locker(&m_queueMutex);
    return m_taskQueue.size();
}

int SaveThread::getTotalSaved() const
{
    return m_totalSaved.loadRelaxed();
}

void SaveThread::run()
{
    while (m_running.loadRelaxed()) {
        SaveTask task;
        bool hasTask = false;
        
        // 从队列中获取任务
        {
            QMutexLocker locker(&m_queueMutex);
            if (!m_taskQueue.isEmpty()) {
                task = m_taskQueue.dequeue();
                hasTask = true;
            } else {
                // 等待新任务或停止信号
                m_queueCondition.wait(&m_queueMutex, 100); // 100ms超时
            }
        }
        
        if (hasTask) {
            saveImage(task);
            m_totalSaved.fetchAndAddOrdered(1);
        }
    }
    
    // 处理剩余任务
    QMutexLocker locker(&m_queueMutex);
    while (!m_taskQueue.isEmpty()) {
        SaveTask task = m_taskQueue.dequeue();
        saveImage(task);
        m_totalSaved.fetchAndAddOrdered(1);
    }
}

void SaveThread::saveImage(const SaveTask& task)
{
    try {
        // 构造文件路径和文件名
        QString filename = QString("%1/Camera%2_X%3.bmp")
            .arg(task.basePath)
            .arg(task.cameraIndex + 1)
            .arg(task.timestamp);
        
        // 创建QImage并保存
        QImage img(reinterpret_cast<const uchar*>(task.imageData.constData()), 
                   task.width, task.height, QImage::Format_Indexed8);
        img.setColorTable(m_grayColourTable);
        
        if (!img.save(filename, "BMP")) {
            qWarning() << "Failed to save image:" << filename;
        }
    } catch (const std::exception& e) {
        qWarning() << "Exception in saveImage:" << e.what();
    } catch (...) {
        qWarning() << "Unknown exception in saveImage";
    }
}
