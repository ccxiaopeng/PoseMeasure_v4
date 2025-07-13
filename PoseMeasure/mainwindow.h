#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include <QMessageBox>
#include "capturethread.h"
#include "savethread.h"
#include "memorypool.h"
#include "ui_mainwindow.h"


#include <windows.h>
#include "CameraApi.h"
#include <opencv2\opencv.hpp> 

typedef struct _WIDTH_HEIGHT {
    int     display_width;
    int     display_height;
    int     xOffsetFOV;
    int     yOffsetFOV;
    int     sensor_width;
    int     sensor_height;
    int     buffer_size;
}Width_Height;


class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget* parent = 0);
    ~MainWindow();
    void closeEvent(QCloseEvent*);
    int status;

protected:
    void changeEvent(QEvent* e);

private:
    Ui::MainWindowClass* ui;
    int  init_SDK();
    int  GUI_init_parameter(int hCamera, tSdkCameraCapbility* pCameraInfo);
    int  GUI_init_exposure(int hCamera, tSdkCameraCapbility* pCameraInfo);
    int  GUI_init_Trigger(int hCamera, tSdkCameraCapbility* pCameraInfo);
    void updateCurrentCameraSettings();

private slots:
    void on_comboBox_camera_select_activated(int index);

    void on_spinBox_gain_valueChanged(int value);
    void on_doubleSpinBox_exposure_time_valueChanged(double value);
    
    void on_radioButton_collect_clicked(bool checked);
    void on_radioButton_software_trigger_clicked(bool checked);
    void on_software_trigger_once_button_clicked();

    void on_pushButton_snap_path_released();
    void on_pushButton_snap_catch_released();
    
    void on_pushButton_captureROI_path_released();
    void on_spinBox_captureROI_num_valueChanged(int value);
    void on_pushButton_captureROI_start_released();
    void on_pushButton_captureROI_stop_released();    
    void Image_process(QImage img, int cameraIndex);
    void updateCoordinateDisplay();
    void camera_statues();

    void on_pushButton_mask_clicked(); // 添加mask按钮槽函数

private:

    QTimer* m_timer;
    QLabel* m_camera_statuesFps;

    QGraphicsScene* m_scene;
    QGraphicsScene* m_scene2; // 第二个相机的场景
    QGraphicsPixmapItem* m_image_item;
    QGraphicsPixmapItem* m_image_item2; // 第二个相机的图像项
    QRadioButton* radioButton_speed[3];

    CaptureThread* m_thread[2];  // 两个采集线程，每个对应一个相机
    
    // 异步保存系统
    SaveThread* m_saveThread;
    MemoryPool* m_memoryPool;

    int m_currentCamera; // 当前选中的相机索引 (0或1)

    //QImage m_maskImages[2];  // 存储两个相机的mask图像
    //bool m_maskCreated[2];   // 标记mask是否已创建
};

#endif // MAINWINDOW_H
