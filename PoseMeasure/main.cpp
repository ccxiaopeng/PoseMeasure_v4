#include "mainwindow.h"
#include <QtWidgets/QApplication>
#include <QDesktopWidget>
#include <opencv2\opencv.hpp> 

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication a(argc, argv);
    MainWindow w;
    if (!w.status) {
        return 1;
    }
    w.show();
    w.move((QApplication::desktop()->width() - w.width()) / 2, (QApplication::desktop()->height() - w.height()) / 2 -20);
    return a.exec();
}
