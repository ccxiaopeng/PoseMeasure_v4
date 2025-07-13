#include <QFileDialog>
#include <QGraphicsPixmapItem>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "capturethread.h"
#include <stdio.h>

#ifdef _WIN64
#pragma comment(lib,"MVCAMSDK_X64.lib")
#else
#pragma comment(lib,"MVCAMSDK.lib")
#endif

//SDK
int                     g_hCamera[2] = {-1, -1};           //设备句柄，支持两个相机
BYTE*                   g_pRawBuffer[2] = {NULL, NULL};    //raw数据
BYTE*                   g_pMono8Buffer[2] = {NULL, NULL};  //单色数据缓冲区
BYTE*                   g_pROIBuffer[2] = {NULL, NULL};    //ROI图像数据缓冲区
tSdkFrameHead           g_tFrameHead[2];                   //图像帧头信息
tSdkCameraCapbility     g_tCapability[2];                  //设备描述信息

int                     g_SaveParameter_num = 0;          //保存参数数
int                     g_SaveImage_type = 0;             //保存图像格式

Width_Height            g_W_H_INFO[2];                    //显示窗口到实际图像大小
BYTE*                   g_readBuf[2] = {NULL, NULL};      //用于显示的buffer
int                     g_read_fps[2] = {0, 0};           //统计读取帧率
int                     g_disply_fps[2] = {0, 0};         //统计显示帧率
int                     g_disply_fps_target = 25;          //设定显示帧率
INT64                   g_timestamp_disply_front[2] = {0, 0}; //前一显示帧时间戳，单位0.1ms    
INT64                   g_timestamp[2] = {0, 0};          //当前帧时间戳，单位0.1ms
int                     g_cameraCount = 0;                //实际检测到的相机数量

QString                 g_captureROI_path = "";
bool                    g_captureROI_flag = false;        //是否采集并保存ROI图像

double                  g_coordinate[6] = { 0,0,0,0,0,0 };   //x1,y1,z1,x2,y2,z2
double                  g_coordinatePixel[2][4];   //u1,v1,u2,v2

cv::Mat                 g_mask[2]; 
bool                    g_mask_flag = false;
QImage                  showImage;
        
MainWindow::MainWindow(QWidget* parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindowClass), m_scene(nullptr), m_scene2(nullptr), 
    m_image_item(nullptr), m_image_item2(nullptr), m_currentCamera(0)
{

    if (init_SDK() == -1) {
        status = 0;
        return;
    }

    ui->setupUi(this);
    m_scene = new QGraphicsScene(this);
    m_scene2 = new QGraphicsScene(this);
    ui->gvMain->setScene(m_scene);
    ui->gvMain_2->setScene(m_scene2);
    
    // 初始化相机选择下拉框
    ui->comboBox_camera_select->clear();
    for (int i = 0; i < g_cameraCount; i++) {
        ui->comboBox_camera_select->addItem(QString("Camera %1").arg(i + 1));
    }
    if (g_cameraCount > 0) {
        ui->comboBox_camera_select->setCurrentIndex(0);
        m_currentCamera = 0;
    }

    //设置定时器，每秒刷新一次相机状态栏
    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(camera_statues()));
    m_timer->start(1000);    // 初始化内存池 (ROI图像大小 512x512 = 262144 字节，池大小100)
    m_memoryPool = new MemoryPool(512 * 512, 100);
    
    // 初始化异步保存线程
    m_saveThread = new SaveThread(this);
    m_saveThread->start();

    // 初始化两个图像采集线程
    for (int i = 0; i < 2; i++) {
        m_thread[i] = nullptr;
    }

    // 为每个相机创建独立的采集线程
    for (int i = 0; i < g_cameraCount; i++) {
        m_thread[i] = new CaptureThread(i, this);

        // 传递保存线程和内存池给采集线程
        m_thread[i]->setSaveThread(m_saveThread);
        m_thread[i]->setMemoryPool(m_memoryPool);

        connect(m_thread[i], SIGNAL(captured(QImage, int)),
            this, SLOT(Image_process(QImage, int)), Qt::QueuedConnection); //异步调用，非阻塞

        connect(m_thread[i], SIGNAL(coordinateUpdated()),
			this, SLOT(updateCoordinateDisplay()), Qt::QueuedConnection); //更新坐标显示

        m_thread[i]->start();
        m_thread[i]->stream();
    }

    //添加状态栏，显示帧率UI
    m_camera_statuesFps = new QLabel(this);
    m_camera_statuesFps->setAlignment(Qt::AlignHCenter);
    ui->statusBar->addWidget(m_camera_statuesFps);

    if (g_cameraCount > 0) {
        GUI_init_parameter(g_hCamera[0], &g_tCapability[0]);
    }
    if (g_cameraCount > 1) {
        GUI_init_parameter(g_hCamera[1], &g_tCapability[1]);
	}

    status = 1;
}

MainWindow::~MainWindow()
{

    // 停止所有采集线程
    for (int i = 0; i < 2; i++) {
        if (m_thread[i]) {
            m_thread[i]->stop();
            m_thread[i]->wait();
            delete m_thread[i];
            m_thread[i] = nullptr;
        }
    }
    
    // 清理内存池
    if (m_memoryPool) {
        delete m_memoryPool;
        m_memoryPool = nullptr;
    }
    
    delete ui;
}

void MainWindow::changeEvent(QEvent* e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}


/*------ 关闭程序处理 ------*/

void MainWindow::closeEvent(QCloseEvent* e)
{

    // 停止所有采集线程
    for (int i = 0; i < 2; i++) {
        if (m_thread[i]) {
            m_thread[i]->stop();
            while (!m_thread[i]->wait(100)) {
                QCoreApplication::processEvents();
            }
        }
    }

    // 释放所有相机的资源
    for (int i = 0; i < 2; i++) {
        if (g_readBuf[i] != NULL) {
            free(g_readBuf[i]);
            g_readBuf[i] = NULL;
        }

        if (g_pMono8Buffer[i] != NULL) {
            free(g_pMono8Buffer[i]);
            g_pMono8Buffer[i] = NULL;
        }

        if (g_hCamera[i] > 0) {
            //反初始化，释放资源。
            CameraUnInit(g_hCamera[i]);
            g_hCamera[i] = -1;
        }
    }

    QMainWindow::closeEvent(e);
}


/*------ 相机状态栏 ------*/

void MainWindow::camera_statues()
{
    // 获取保存队列和内存池状态
    int queueSize = m_saveThread ? m_saveThread->getQueueSize() : 0;
    int totalSaved = m_saveThread ? m_saveThread->getTotalSaved() : 0;
    int availableMemory = m_memoryPool ? m_memoryPool->getAvailableCount() : 0;
    float fDevTemperature1 = 0.0;
    CameraSpecialControl(g_hCamera[0], 20, 0, (LPVOID)&fDevTemperature1);
    float fDevTemperature2 = 0.0;
    CameraSpecialControl(g_hCamera[1], 20, 0, (LPVOID)&fDevTemperature2);

    qDebug() << "Camera 1 Temperature:" << fDevTemperature1 << "Camera 2 Temperature:" << fDevTemperature2;
    
    // 显示两个相机的帧率信息以及保存状态
    QString statusText = QString("Cam1: 采集:%1FPS 显示:%2FPS 温度:%3°C | Cam2: 采集:%4FPS 显示:%5FPS 温度:%6°C | Queue:%7 Saved:%8 Mem:%9")
        .arg(QString::number(g_read_fps[0], 'f', 1))
        .arg(QString::number(g_disply_fps[0], 'f', 1))
        .arg(QString::number(fDevTemperature1, 'f',1))
        .arg(QString::number(g_read_fps[1], 'f', 1))
        .arg(QString::number(g_disply_fps[1], 'f', 1))
        .arg(QString::number(fDevTemperature2, 'f',1))
        .arg(queueSize)
        .arg(totalSaved)
        .arg(availableMemory);
    
    m_camera_statuesFps->setText(statusText);
    
    // 重置帧率计数
    g_read_fps[0] = 0;
    g_read_fps[1] = 0;
    g_disply_fps[0] = 0;
    g_disply_fps[1] = 0;
}


/*------ 图像显示刷新处理 ------*/

void MainWindow::Image_process(QImage img, int cameraIndex)
{
    // 检查对应的线程是否正在退出
    if (cameraIndex >= 0 && cameraIndex < 2 && m_thread[cameraIndex] && m_thread[cameraIndex]->quit)
    {
        return;
    }

    if (cameraIndex == 0) {
        // 处理第一个相机的图像显示在 gvMain
        if (m_image_item)
        {
            m_scene->removeItem(m_image_item);
            delete m_image_item;
            m_image_item = 0;
        }

        // 获取gvMain的显示区域大小
        QSize viewSize = ui->gvMain->viewport()->size();
        // 按比例缩放图像，保持宽高比，提高显示速度
        QImage scaledImg = img.scaled(viewSize, Qt::KeepAspectRatio, Qt::FastTransformation);
		scaledImg = scaledImg.mirrored(false, true); // 上下翻转图像

        m_image_item = m_scene->addPixmap(QPixmap::fromImage(scaledImg));
        m_scene->setSceneRect(0, 0, scaledImg.width(), scaledImg.height());

        g_disply_fps[0]++;
    } else if (cameraIndex == 1) {
        // 处理第二个相机的图像显示在 gvMain_2
        if (m_image_item2)
        {
            m_scene2->removeItem(m_image_item2);
            delete m_image_item2;
            m_image_item2 = 0;
        }

        // 获取gvMain_2的显示区域大小
        QSize viewSize = ui->gvMain_2->viewport()->size();
        // 按比例缩放图像，保持宽高比，提高显示速度
        QImage scaledImg = img.scaled(viewSize, Qt::KeepAspectRatio, Qt::FastTransformation);
		scaledImg = scaledImg.mirrored(false, true); // 上下翻转图像

        m_image_item2 = m_scene2->addPixmap(QPixmap::fromImage(scaledImg));
        m_scene2->setSceneRect(0, 0, scaledImg.width(), scaledImg.height());

        g_disply_fps[1]++;
    }
}

void MainWindow::updateCoordinateDisplay()
{
    ui->label_X1->setText(QString::number(g_coordinate[0], 'f', 4)); // 显示刀轴点X坐标
	ui->label_Y1->setText(QString::number(g_coordinate[1], 'f', 4)); // 显示刀轴点Y坐标
	ui->label_Z1->setText(QString::number(g_coordinate[2], 'f', 4)); // 显示刀轴点Z坐标
	ui->label_X2->setText(QString::number(g_coordinate[3], 'f', 4)); // 显示工件点X坐标
	ui->label_Y2->setText(QString::number(g_coordinate[4], 'f', 4)); // 显示工件点Y坐标
	ui->label_Z2->setText(QString::number(g_coordinate[5], 'f', 4)); // 显示工件点Z坐标
}


/*------ SDK等初始化操作 ------*/

int MainWindow::init_SDK()
{

    int                     iCameraCounts = 4;
    int                     iStatus = -1;
    tSdkCameraDevInfo       tCameraEnumList[4];

    //sdk初始化  0 English 1中文
    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    CameraEnumerateDevice(tCameraEnumList, &iCameraCounts);

    //没有连接设备
    if (iCameraCounts == 0) {
		qDebug() << "No camera connected!";
        return -1;
    }

    g_cameraCount = (iCameraCounts > 2) ? 2 : iCameraCounts;

    if (g_cameraCount==1)
    {
        //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        iStatus = CameraInit(&tCameraEnumList[0], -1, -1, &g_hCamera[0]);
        if (iStatus != CAMERA_STATUS_SUCCESS) return -1;
    }
    else if (g_cameraCount == 2)
    {
		//获取设备名tCameraEnumList[0].acFriendlyName,根据设备名确定相机的索引,CameraX的索引为0,CameraY的索引为1
        if (strcmp(tCameraEnumList[0].acFriendlyName, "CameraX") == 0)
        {
            iStatus = CameraInit(&tCameraEnumList[0], -1, -1, &g_hCamera[0]);
            if (iStatus != CAMERA_STATUS_SUCCESS) return -1;
			iStatus = CameraInit(&tCameraEnumList[1], -1, -1, &g_hCamera[1]);
            if (iStatus != CAMERA_STATUS_SUCCESS) return -1;
        }
        else if (strcmp(tCameraEnumList[0].acFriendlyName, "CameraY") == 0)
        {
            iStatus = CameraInit(&tCameraEnumList[1], -1, -1, &g_hCamera[0]);
			if (iStatus != CAMERA_STATUS_SUCCESS) return -1;
            iStatus = CameraInit(&tCameraEnumList[0], -1, -1, &g_hCamera[1]);
            if (iStatus != CAMERA_STATUS_SUCCESS) return -1;
        }
        else
        {
			qDebug() << "Unknown camera name: " << tCameraEnumList[0].acFriendlyName;
            return -1;
        }
    }
    else return -1;

    for (int i = 0; i < g_cameraCount; i++) {

        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(g_hCamera[i], &g_tCapability[i]);

        g_pMono8Buffer[i] = (unsigned char*)malloc(g_tCapability[i].sResolutionRange.iHeightMax * g_tCapability[i].sResolutionRange.iWidthMax);
        g_readBuf[i] = (unsigned char*)malloc(g_tCapability[i].sResolutionRange.iHeightMax * g_tCapability[i].sResolutionRange.iWidthMax);

        //让SDK进入工作模式，开始接收来自相机发送的图像数据。
        //如果当前相机是触发模式，则需要接收到触发帧以后才会更新图像。    
        CameraPlay(g_hCamera[i]);

        //设置图像处理的输出格式，彩色黑白均支持RGB24位
        if (g_tCapability[i].sIspCapacity.bMonoSensor) {
            CameraSetIspOutFormat(g_hCamera[i], CAMERA_MEDIA_TYPE_MONO8);
        }
        else {
            CameraSetIspOutFormat(g_hCamera[i], CAMERA_MEDIA_TYPE_RGB8);
        }
    }
    
    return 0;
}



/*------ QT界面初始化 ------*/

int  MainWindow::GUI_init_parameter(int hCamera, tSdkCameraCapbility* pCameraInfo)
{
    tSdkImageResolution*    pImageSizeDesc = pCameraInfo->pImageSizeDesc;// 预设分辨率选择
    tSdkImageResolution     sResolution;  //获取当前设置的分辨率

    //获得当前预设的分辨率。
    CameraGetImageResolution(hCamera, &sResolution);

    // 找到对应的相机索引
    int cameraIndex = -1;
    for (int i = 0; i < g_cameraCount; i++) {
        if (g_hCamera[i] == hCamera) {
            cameraIndex = i;
            break;
        }
    }
    
    if (cameraIndex >= 0) {
        g_W_H_INFO[cameraIndex].sensor_width = pImageSizeDesc[sResolution.iIndex].iWidth;
        g_W_H_INFO[cameraIndex].sensor_height = pImageSizeDesc[sResolution.iIndex].iHeight;
        g_W_H_INFO[cameraIndex].buffer_size = g_W_H_INFO[cameraIndex].sensor_width * g_W_H_INFO[cameraIndex].sensor_height;
    }

    GUI_init_exposure(hCamera, pCameraInfo);
    GUI_init_Trigger(hCamera, pCameraInfo);

    ui->snap_path_lineEdit->setText(QString("./"));

    g_SaveImage_type = 3;

    return 0;
}


int  MainWindow::GUI_init_Trigger(int hCamera, tSdkCameraCapbility* pCameraInfo)
{
    int  pbySnapMode;
    int StrobeMode = 0;
    int  uPolarity = 0;

    //获得相机的触发模式
    CameraGetTriggerMode(hCamera, &pbySnapMode);

    //设置相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
    switch (pbySnapMode) {
    case 0:
        ui->radioButton_collect->setChecked(true);
        ui->software_trigger_once_button->setEnabled(false);

        break;
    case 1:
        ui->radioButton_software_trigger->setChecked(true);
        ui->software_trigger_once_button->setEnabled(true);

        break;

    default:
        ui->radioButton_collect->setChecked(true);
        ui->software_trigger_once_button->setEnabled(false);
        break;
    }

    return 1;
}

int  MainWindow::GUI_init_exposure(int hCamera, tSdkCameraCapbility* pCameraInfo)
{

    BOOL            AEstate = FALSE;    //默认手动曝光
    int             pbyAeTarget;
    double          pfExposureTime;
    int             pusAnalogGain;
    BOOL            FlickEnable = FALSE;
    int             piFrequencySel;
    double	        m_fExpLineTime = 0;//当前的行曝光时间，单位为us
    tSdkExpose* SdkExpose = &pCameraInfo->sExposeDesc;

    //设置相机默认手动曝光模式。
    CameraSetAeState(hCamera, false);

    // 设置初始相机默认曝光时间为10μs
    CameraSetExposureTime(hCamera, 10);

    // 设置初始相机默认模拟增益为最小值
    CameraSetAnalogGain(hCamera, 6);
    
    //获得相机当前的曝光模式。
    //CameraGetAeState(hCamera, &AEstate);

    //获得自动曝光的亮度目标值。
    CameraGetAeTarget(hCamera, &pbyAeTarget);

    //获得自动曝光时抗频闪功能的使能状态。
    CameraGetAntiFlick(hCamera, &FlickEnable);

    //获得相机的曝光时间。
    CameraGetExposureTime(hCamera, &pfExposureTime);

    //获得图像信号的模拟增益值。
    CameraGetAnalogGain(hCamera, &pusAnalogGain);

    //获得自动曝光时，消频闪的频率选择。
    CameraGetLightFrequency(hCamera, &piFrequencySel);

    /*
        获得一行的曝光时间。对于CMOS传感器，其曝光
        的单位是按照行来计算的，因此，曝光时间并不能在微秒
        级别连续可调。而是会按照整行来取舍。这个函数的
        作用就是返回CMOS相机曝光一行对应的时间。
    */
    CameraGetExposureLineTime(hCamera, &m_fExpLineTime);

    ui->spinBox_gain->setMinimum(SdkExpose->uiAnalogGainMin);
    ui->spinBox_gain->setMaximum(SdkExpose->uiAnalogGainMax);
    ui->spinBox_gain->setValue(pusAnalogGain);

    ui->doubleSpinBox_exposure_time->setMinimum(SdkExpose->uiExposeTimeMin);
    ui->doubleSpinBox_exposure_time->setMaximum(SdkExpose->uiExposeTimeMax);
    ui->doubleSpinBox_exposure_time->setValue(pfExposureTime);

    return 1;

}

/*------ QT界面按钮等操作 ------*/

//相机选择下拉框操作
void MainWindow::on_comboBox_camera_select_activated(int index)
{
    if (index >= 0 && index < g_cameraCount) {
        m_currentCamera = index;
        updateCurrentCameraSettings();
    }
}

// 更新当前选中相机的设置到UI
void MainWindow::updateCurrentCameraSettings()
{
    if (m_currentCamera >= 0 && m_currentCamera < g_cameraCount) {
        int hCamera = g_hCamera[m_currentCamera];
        
        // 获取当前相机的增益值
        int pusAnalogGain;
        CameraGetAnalogGain(hCamera, &pusAnalogGain);
        ui->spinBox_gain->setValue(pusAnalogGain);
        
        // 获取当前相机的曝光时间
        double pfExposureTime;
        CameraGetExposureTime(hCamera, &pfExposureTime);
        ui->doubleSpinBox_exposure_time->setValue(pfExposureTime);
        
        // 获取当前相机的触发模式
        int pbySnapMode;
        CameraGetTriggerMode(hCamera, &pbySnapMode);
        
        switch (pbySnapMode) {
        case 0:
            ui->radioButton_collect->setChecked(true);
            ui->software_trigger_once_button->setEnabled(false);
            break;
        case 1:
            ui->radioButton_software_trigger->setChecked(true);
            ui->software_trigger_once_button->setEnabled(true);
            break;
        default:
            ui->radioButton_collect->setChecked(true);
            ui->software_trigger_once_button->setEnabled(false);
            break;
        }
    }
}


//增益值设置
void MainWindow::on_spinBox_gain_valueChanged(int value)
{
    if (m_currentCamera >= 0 && m_currentCamera < g_cameraCount) {
        int pusAnalogGain = 0;
        int hCamera = g_hCamera[m_currentCamera];

        CameraSetAnalogGain(hCamera, value);
        CameraGetAnalogGain(hCamera, &pusAnalogGain);
        ui->spinBox_gain->setValue(pusAnalogGain);
    }
}

//曝光时间设置
void MainWindow::on_doubleSpinBox_exposure_time_valueChanged(double value)
{
    if (m_currentCamera >= 0 && m_currentCamera < g_cameraCount) {
        double m_fExpTime = 0;     //当前的曝光时间，单位为us
        int hCamera = g_hCamera[m_currentCamera];

        /*
        设置曝光时间。单位为微秒。对于CMOS传感器，其曝光
        的单位是按照行来算的，因此，曝光时间并不能在微秒
        级别连续可调。而是会按照整行来取舍。在调用
        函数设定曝光时间后，建议再调用CameraGetExposureTime
        来获得实际设定的值。
        */
        CameraSetExposureTime(hCamera, value);
        CameraGetExposureTime(hCamera, &m_fExpTime);
        ui->doubleSpinBox_exposure_time->setValue(m_fExpTime);
    }
}

//连续采集模式
void MainWindow::on_radioButton_collect_clicked(bool checked)
{
    ui->radioButton_collect->setChecked(true);
    if (checked)
    {
        // 设置所有相机的触发模式
        for (int i = 0; i < g_cameraCount; i++) {
            CameraSetTriggerMode(g_hCamera[i], 0);
        }

        ui->radioButton_collect->setChecked(true);
        ui->software_trigger_once_button->setEnabled(false);
    }
}

//软触发模式
void MainWindow::on_radioButton_software_trigger_clicked(bool checked)
{
    ui->radioButton_software_trigger->setChecked(true);
    if (checked)
    {
        // 设置所有相机的触发模式
        for (int i = 0; i < g_cameraCount; i++) {
            CameraSetTriggerMode(g_hCamera[i], 1);
        }

        //获得相机的触发模式。0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
        ui->radioButton_software_trigger->setChecked(true);
        ui->software_trigger_once_button->setEnabled(true);
    }
}

//软触发一次操作
void MainWindow::on_software_trigger_once_button_clicked()
{
    // 同时触发所有相机
    for (int i = 0; i < g_cameraCount; i++) {
        //执行一次软触发。执行后，会触发由CameraSetTriggerCount指定的帧数。
        CameraSoftTrigger(g_hCamera[i]);
    }
}

//保存图片路径设置
void MainWindow::on_pushButton_snap_path_released()
{
    QFileDialog* openFilePath = new QFileDialog(this, "Select Folder", "");     //��һ��Ŀ¼ѡ��Ի���
    openFilePath->setFileMode(QFileDialog::Directory);
    if (openFilePath->exec() == QDialog::Accepted)
    {
        QString path = openFilePath->selectedFiles()[0];

        ui->snap_path_lineEdit->setText(path);
    }
    delete openFilePath;

}

//保存图片按钮确认
void MainWindow::on_pushButton_snap_catch_released()
{
    QString path = ui->snap_path_lineEdit->text();
    char* dir;
    QByteArray tmp = path.toLatin1();
    dir = tmp.data();

    // 同时抓拍所有相机的图像
    for (int i = 0; i < g_cameraCount; i++) {
        tSdkFrameHead tFrameHead;
        BYTE* pbyBuffer;
        BYTE* pbImgBuffer;
        char filename[512] = { 0 };

        sprintf_s(filename, sizeof(filename), "%s/camera%d_test", dir, i + 1);

        //CameraSnapToBuffer抓取一帧图像保存到buffer中
        // !!!!!!注意：CameraSnapToBuffer 可能会直接阻塞住，速度较慢，实时任务请用CameraGetImageBuffer来获取图像或回调方式
        if (CameraSnapToBuffer(g_hCamera[i], &tFrameHead, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            pbImgBuffer = (unsigned char*)malloc(g_tCapability[i].sResolutionRange.iHeightMax * g_tCapability[i].sResolutionRange.iWidthMax);

            /*
            将获得的原始输出图像数据进行处理，叠加饱和度、
            颜色增益和校正、降噪等等功能，最后得到RGB888
            格式的图像数据。
            */
            CameraImageProcess(g_hCamera[i], pbyBuffer, pbImgBuffer, &tFrameHead);

            //将图像缓冲区的数据保存成图片文件。
            CameraSaveImage(g_hCamera[i], filename, pbImgBuffer, &tFrameHead, FILE_BMP_8BIT, 100);
            //释放由CameraGetImageBuffer获得的缓冲区。
            CameraReleaseImageBuffer(g_hCamera[i], pbImgBuffer);
            free(pbImgBuffer);
        }
    }
}

//采集ROI图像路径设置
void MainWindow::on_pushButton_captureROI_path_released()
{
    QFileDialog* openFilePath = new QFileDialog(this, "Select Folder", "");     //��һ��Ŀ¼ѡ��Ի���
    openFilePath->setFileMode(QFileDialog::Directory);
    if (openFilePath->exec() == QDialog::Accepted)
    {
        g_captureROI_path = openFilePath->selectedFiles()[0];

        ui->captureROI_path_lineEdit->setText(g_captureROI_path);
    }
    delete openFilePath;
}

//采集ROI图像数量设置
void MainWindow::on_spinBox_captureROI_num_valueChanged(int value)
{
}

//采集ROI图像开始按钮
void MainWindow::on_pushButton_captureROI_start_released()
{
    if (g_captureROI_path.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please set the save path first!");
        return;
    }
    g_captureROI_flag = true;
}

//采集ROI图像停止按钮
void MainWindow::on_pushButton_captureROI_stop_released()
{
    g_captureROI_flag = false;
}



void MainWindow::on_pushButton_mask_clicked()
{
    // 同时获取所有相机的图像mask
    for (int i = 0; i < g_cameraCount; i++) {
        tSdkFrameHead tFrameHead;
        BYTE* pRawBuffer;
        BYTE* pProcessedBuffer;


        // 获取一帧图像
        if (CameraGetImageBuffer(g_hCamera[i], &tFrameHead, &pRawBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            // 分配处理后的图像缓冲区
            BYTE* pProcessedBuffer = new BYTE[tFrameHead.iWidth * tFrameHead.iHeight];

            // 图像处理
            if (CameraImageProcess(g_hCamera[i], pRawBuffer, pProcessedBuffer, &tFrameHead) == CAMERA_STATUS_SUCCESS) {
                // 创建OpenCV Mat对象
                cv::Mat imageMat(tFrameHead.iHeight, tFrameHead.iWidth, CV_8UC1, pProcessedBuffer);

                cv::Mat scaledMat;
                    
                cv::resize(imageMat, scaledMat, cv::Size(512, 512), 0, 0, cv::INTER_LINEAR);

                // 克隆图像数据
                g_mask[i] = scaledMat < 250;

                // 释放缓冲区
                //delete[] pProcessedBuffer;
                //CameraReleaseImageBuffer(g_hCamera[i], pRawBuffer);

            }
            // 释放缓冲区
            delete[] pProcessedBuffer;
            CameraReleaseImageBuffer(g_hCamera[i], pRawBuffer);
        }

    }
    qDebug() << "掩膜更新完成";
    g_mask_flag = true;
}

