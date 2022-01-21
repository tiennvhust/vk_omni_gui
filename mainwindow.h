#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include <ros/ros.h>
#include <qpublish.h>
#include <qsubscribe.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void velSignal(geometry_msgs::Twist);
    void subscribeSignal();

private slots:
    void on_backward_pressed();

    void on_backward_released();

    void on_forward_pressed();

    void on_forward_released();

    void on_right_pressed();

    void on_right_released();

    void on_left_pressed();

    void on_left_released();

    void on_forward_right_pressed();

    void on_forward_right_released();

    void on_backward_right_pressed();

    void on_backward_right_released();

    void on_backward_left_pressed();

    void on_backward_left_released();

    void on_forward_left_pressed();

    void on_forward_left_released();

    void setVelocityText(const sensor_msgs::JointState::ConstPtr &msg);

    void setOdomText(const nav_msgs::Odometry::ConstPtr &msg);

private:
    Ui::MainWindow *ui;

    ros::NodeHandle nh;

    QPublish* p_QPublish;
    QSubscribe* p_QSubscribe;

    QThread p_QPublish_thread;
    QThread p_QSubscribe_thread;
};

#endif // MAINWINDOW_H
