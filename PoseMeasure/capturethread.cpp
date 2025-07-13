#include "capturethread.h"
#include "mainwindow.h"
#include "savethread.h"
#include "memorypool.h"
#include <QDebug>

#include <windows.h>
#include "CameraApi.h"
#include <opencv2\opencv.hpp> 

#include <sstream>
#include <iomanip>

using namespace std;

//SDK使用
extern int                  g_hCamera[2];        //设备句柄
extern BYTE*                g_pRawBuffer[2];     //raw数据
extern BYTE*                g_pMono8Buffer[2];   //单色数据缓冲区
extern BYTE*                g_pROIBuffer[2];     //ROI图像数据缓冲区
extern tSdkFrameHead        g_tFrameHead[2];     //图像帧头信息
extern tSdkCameraCapbility  g_tCapability[2];    //设备描述信息


extern Width_Height         g_W_H_INFO[2];       //显示窗口到实际图像大小
extern BYTE*                g_readBuf[2];        //显示数据buffer
extern int                  g_read_fps[2];       //统计帧率
extern int                  g_disply_fps[2];     //统计显示帧率
extern int                  g_disply_fps_target; //设定显示帧率
extern int                  g_SaveImage_type;    //保存图像格式
extern INT64                g_timestamp_disply_front[2]; //前一显示帧时间戳    
extern INT64                g_timestamp[2];      //当前帧时间戳
extern int                  g_cameraCount;       //实际检测到的相机数量

extern QString              g_captureROI_path;
extern bool                 g_captureROI_flag; //是否采集并保存ROI图像

extern double               g_coordinate[6];   //x1,y1,z1,x2,y2,z2
extern double               g_coordinatePixel[2][4];   //u1,v1,u2,v2

extern cv::Mat              g_mask[2];
extern bool                 g_mask_flag;
extern QImage               showImage;

CaptureThread::CaptureThread(int cameraIndex, QObject* parent) :
    QThread(parent), m_cameraIndex(cameraIndex), m_saveThread(nullptr), m_memoryPool(nullptr)
{
    pause_status = true;
    quit = false;

    for (int i = 0; i < 256; i++)
    {
        grayColourTable.append(qRgb(i, i, i));
    }
}

void CaptureThread::run()
{
    forever
    {
        if (!pause_status)
        {
            if (quit) break;
            
            // 处理所有相机的图像采集
            // 只处理指定的相机
            int i = m_cameraIndex;
            if (i >= 0 && i < g_cameraCount && g_hCamera[i] != -1)
            {
                if (CameraGetImageBuffer(g_hCamera[i], &g_tFrameHead[i], &g_pRawBuffer[i], 100) == CAMERA_STATUS_SUCCESS)
                {
                    //获取当前帧的时间戳，单位0.1毫秒
                    g_timestamp[i] = g_tFrameHead[i].uiTimeStamp;

                    CameraImageProcess(g_hCamera[i], g_pRawBuffer[i], g_pMono8Buffer[i], &g_tFrameHead[i]);
                    CameraReleaseImageBuffer(g_hCamera[i], g_pRawBuffer[i]);

                    cv::Mat imageMat(g_tFrameHead[i].iHeight, g_tFrameHead[i].iWidth, CV_8UC1, g_pMono8Buffer[i]);

                    cv::resize(imageMat, imageMat, cv::Size(512, 512), 0, 0, cv::INTER_LINEAR);

                    if (g_mask_flag)
                    {
                        cv::Mat imageMaskMat = imageMat.clone();

                        imageMaskMat.setTo(255, g_mask[i]);


                        // 计算行和
                        vector<int> row_sums(imageMaskMat.rows, 0);

                        for (int i = 0; i < imageMaskMat.rows; i++) {
                            for (int j = 0; j < imageMaskMat.cols; j++) {
                                row_sums[i] += imageMaskMat.at<uchar>(i, j);
                            }
                            row_sums[i] = 255 * 512 - row_sums[i]; // 转换为峰值检测格式（反转）
                        }

                        // 找到峰值,确定v坐标
                        vector<int> peaks;
                        vector<int> data = row_sums;
                        int prominence = 1000;
                        for (int i = 1; i < data.size() - 1; i++) {
                            if (data[i] > data[i - 1] && data[i] > data[i + 1]) {
                                // 检查prominence
                                int leftMin = data[i];
                                int rightMin = data[i];

                                // 向左查找最小值
                                for (int j = i - 1; j >= 0; j--) {
                                    leftMin = min(leftMin, data[j]);
                                    if (data[j] >= data[i]) break;
                                }

                                // 向右查找最小值
                                for (int j = i + 1; j < data.size(); j++) {
                                    rightMin = min(rightMin, data[j]);
                                    if (data[j] >= data[i]) break;
                                }

                                int actualProminence = data[i] - max(leftMin, rightMin);
                                if (actualProminence >= prominence) {
                                    peaks.push_back(i);
                                }
                            }
                        }
                        
                        // 确定u坐标
                        if (peaks.size() == 2) {
                            double v1 = peaks[0];
                            double v2 = peaks[1];

                            qDebug() << peaks[0] << peaks[1];

                            // 计算u1
                            vector<int> indices1;
                            for (int j = 0; j < imageMaskMat.cols; j++) {
                                if (imageMaskMat.at<uchar>(v1, j) < 200) {
                                    indices1.push_back(j);
                                }
                            }
                            double u1 = (indices1.front() + indices1.back()) / 2.0;

                            // 计算u2
                            vector<int> indices2;
                            for (int j = 0; j < imageMaskMat.cols; j++) {
                                if (imageMaskMat.at<uchar>(v2, j) < 200) {
                                    indices2.push_back(j);
                                }
                            }
                            double u2 = (indices2.front() + indices2.back()) / 2.0;

                            // 添加随机数
                            random_device rd;
                            mt19937 gen(rd());
                            uniform_real_distribution<> dis(-0.5, 0.5);

                            u1 += dis(gen);
                            v1 += dis(gen);
                            u2 += dis(gen);
                            v2 += dis(gen);

                            u1 *= 10;
                            v1 *= 10;
                            u2 *= 10;
                            v2 *= 10;

                            g_coordinatePixel[i][0] = u1;
                            g_coordinatePixel[i][1] = v1;
                            g_coordinatePixel[i][2] = u2;
                            g_coordinatePixel[i][3] = v2;

                            // 绘制结果
                            cvtColor(imageMat, imageMat, cv::COLOR_GRAY2BGR);

                            cv::circle(imageMat, cv::Point(int(u1 / 10), int(v1 / 10)), 5, cv::Scalar(255, 0, 0), -1);
                            cv::circle(imageMat, cv::Point(int(u2 / 10), int(v2 / 10)), 5, cv::Scalar(255, 0, 0), -1);

                            //// 添加坐标显示（保留两位小数）
                            //std::stringstream ss1, ss2;
                            //ss1 << "(" << std::fixed << std::setprecision(2) << u1 << ", " << std::fixed << std::setprecision(2) << v1 << ")";
                            //ss2 << "(" << std::fixed << std::setprecision(2) << u2 << ", " << std::fixed << std::setprecision(2) << v2 << ")";

                            //// 在圆圈旁边显示坐标
                            //cv::putText(imageMat, ss1.str(), cv::Point(int(u1/10) + 60, int(v1/10) - 60),
                            //    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3);
                            //cv::putText(imageMat, ss2.str(), cv::Point(int(u2/10) + 60, int(v2/10) - 60),
                            //    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 3);

                            qDebug() << "刀尖点像素坐标" << i << "(" << u1 << "," << v1 << ")\n";
                            qDebug() << "刀轴点像素坐标" << i << "(" << u2 << "," << v2 << ")\n\n";

                            double m = 100.0 / 5120;

                            if (i==0)
                            {
                                //刀轴
                                g_coordinate[0] = u2 * m; // x1
                                //g_coordinate[1] = 0 // y1
                                g_coordinate[2] = v2 * m; // z1

                                //刀尖
                                g_coordinate[3] = u1 * m; // x2
                                //g_coordinate[4] = v2 // y2
                                g_coordinate[5] = v1 * m; // z2
                            }
                            else
                            {
                                //刀轴
                                //g_coordinate[0] = 0; // x1
                                g_coordinate[1] = u2 * m; // y1
                                //g_coordinate[2] = 0; // z1
                                //刀尖
                                //g_coordinate[3] = 0; // x2
                                g_coordinate[4] = u1 * m; // y2
                                //g_coordinate[5] = v2; // z2
							}

							
                        }
                    }

                    if ((g_timestamp[i] - g_timestamp_disply_front[i]) / 10000.0 > 1.0 / g_disply_fps_target)
                    //if(1)
                    {
                        //memcpy(g_readBuf[i], g_pMono8Buffer[i], g_W_H_INFO[i].buffer_size);

                        //if (quit) break;

                        //// 创建原始图像
                        //QImage originalImg(g_readBuf[i], g_W_H_INFO[i].sensor_width, g_W_H_INFO[i].sensor_height, QImage::Format_Indexed8);
                        //originalImg.setColorTable(grayColourTable);

                        //// 将图像缩小为原来的1/10（512，512）
                        //QImage scaledImg = originalImg.scaled(g_W_H_INFO[i].sensor_width / 10, g_W_H_INFO[i].sensor_height / 10,
                        //    Qt::KeepAspectRatio, Qt::FastTransformation);
                        
                        if (imageMat.type() == CV_8UC1)
                        {
                            cvtColor(imageMat, imageMat, cv::COLOR_GRAY2BGR);
                        }
                        
                        QImage qimg(imageMat.data, imageMat.cols, imageMat.rows, imageMat.step, QImage::Format_RGB888);

                        showImage = qimg.copy();
                        
                        emit captured(showImage, i);     //发射信号，通知主线程更新显示

                        g_timestamp_disply_front[i] = g_timestamp[i]; // 更新显示时间戳
                    }                    
                    if (g_captureROI_flag) //如果需要采集ROI图像
                    {
                        // 使用新的异步保存方法
                        if (m_saveThread && m_memoryPool) {
                            captureROIAsync(i);
                        } else {
                            // 回退到原来的同步方法
                            captureROI(i);
                        }
                    }

                    g_read_fps[i]++; //统计抓取帧率
                    emit coordinateUpdated();     //发射信号，通知主线程更新显示
                }
            }
        }
        else usleep(1000);
        
        if (quit) break;
    }
}

void CaptureThread::stream()
{
    pause_status = false;
}

void CaptureThread::pause()
{
    pause_status = true;
}

void CaptureThread::stop()
{
    pause_status = false;
    quit = true;
}

void CaptureThread::setSaveThread(SaveThread* saveThread)
{
    m_saveThread = saveThread;
}

void CaptureThread::setMemoryPool(MemoryPool* memoryPool)
{
    m_memoryPool = memoryPool;
}

void CaptureThread::captureROI(int cameraIndex)
{
    if (cameraIndex < 0 || cameraIndex >= g_cameraCount) return;
    
    // 复制当前相机灰度图像数据到临时缓冲区
    BYTE* pbImgBuffer = (BYTE*)malloc(g_W_H_INFO[cameraIndex].buffer_size);
    if (pbImgBuffer == nullptr || g_pMono8Buffer[cameraIndex] == nullptr) return;
    memcpy(pbImgBuffer, g_pMono8Buffer[cameraIndex], g_W_H_INFO[cameraIndex].buffer_size);
    
    int width = g_W_H_INFO[cameraIndex].sensor_width;
    int height = g_W_H_INFO[cameraIndex].sensor_height;
    int ROI_w = 512; // ROI宽度
    int ROI_h = 512; // ROI高度
    int ROI_x = (width - ROI_w) / 2; // ROI左上角x坐标
    int ROI_y = (height - ROI_h) / 2; // ROI左上角y坐标

    std::vector<BYTE> ROI;
    ROI.resize(ROI_w * ROI_h);
    for (int r = 0; r < ROI_h; r++) {
        memcpy(&ROI[r * ROI_w], &pbImgBuffer[(ROI_y + r) * width + ROI_x], ROI_w);
    }

    //构造文件路径和文件名
    char filename[512] = { 0 };
    char* dir;
    QByteArray tmp = g_captureROI_path.toLatin1();
    dir = tmp.data();
    sprintf_s(filename, sizeof(filename), "%s/Camera%d_X%I64d", dir, cameraIndex + 1, g_timestamp[cameraIndex]);

    //保存ROI图像
    QImage imgROI(&ROI[0], ROI_w, ROI_h, QImage::Format_Indexed8);
    imgROI.setColorTable(grayColourTable);
    imgROI.save(QString::fromLatin1(filename) + ".bmp", "BMP");

    free(pbImgBuffer);


}

void CaptureThread::captureROIAsync(int cameraIndex)
{
    if (cameraIndex < 0 || cameraIndex >= g_cameraCount) return;
    if (!m_saveThread || !m_memoryPool) return;
    
    // 从内存池获取缓冲区
    QByteArray* buffer = m_memoryPool->acquireBlock();
    if (!buffer) {
        qWarning() << "Failed to acquire memory block for camera" << cameraIndex;
        return;
    }
    
    int width = g_W_H_INFO[cameraIndex].sensor_width;
    int height = g_W_H_INFO[cameraIndex].sensor_height;
    int ROI_w = 512; // ROI宽度
    int ROI_h = 512; // ROI高度
    int ROI_x = (width - ROI_w) / 2; // ROI左上角x坐标
    int ROI_y = (height - ROI_h) / 2; // ROI左上角y坐标
    
    // 确保缓冲区大小足够
    if (buffer->size() < ROI_w * ROI_h) {
        buffer->resize(ROI_w * ROI_h);
    }
    
    // 直接从原始缓冲区提取ROI数据，减少内存拷贝
    BYTE* roiData = reinterpret_cast<BYTE*>(buffer->data());
    BYTE* sourceData = g_pMono8Buffer[cameraIndex];
    
    if (!sourceData) {
        m_memoryPool->releaseBlock(buffer);
        return;
    }
    
    // 高效的ROI提取：逐行拷贝
    for (int r = 0; r < ROI_h; r++) {
        memcpy(&roiData[r * ROI_w], 
               &sourceData[(ROI_y + r) * width + ROI_x], 
               ROI_w);
    }
    
    // 创建保存任务
    SaveTask task;
    task.imageData = *buffer; // 这里会进行一次拷贝，但在异步线程中处理
    task.width = ROI_w;
    task.height = ROI_h;
    task.cameraIndex = cameraIndex;
    task.timestamp = g_timestamp[cameraIndex];
    task.basePath = g_captureROI_path;
    
    // 将任务加入保存队列
    m_saveThread->addSaveTask(task);
    
    // 立即释放内存块供下次使用
    m_memoryPool->releaseBlock(buffer);
}

void CaptureThread::captureCoordinatePixel(int cameraIndex)
{

}
