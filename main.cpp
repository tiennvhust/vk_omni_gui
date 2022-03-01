#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ros::init(argc, argv, "Qnode");
    MainWindow w;

//    ros::spin();
    w.showFullScreen();
    return a.exec();
}
