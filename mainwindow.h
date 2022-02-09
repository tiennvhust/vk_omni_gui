#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>

#include <ros/ros.h>
#include <qpublish.h>
#include <qsubscribe.h>
#include <joystick.h>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

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
    void velSignal(array<double, 3>);

    void subscribeSignal();

    void upSpinSignal(const bool*);

    void downSpinSignal(const bool*);

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

    void onStatusUpdate(robot_status);

    void onBatteryUpdate(int);

    void on_battery_status_valueChanged(int value);

    void on_speedup_pressed();

    void on_speeddown_pressed();

    void on_speedup_released();

    void on_speeddown_released();

private:
    Ui::MainWindow *ui;

    ros::NodeHandle nh;

    QPublish* p_QPublish;
    QSubscribe* p_QSubscribe;
    JoyStick* p_JoyStick;

    QThread QPublish_thread;
    QThread QSubscribe_thread;
    QThread JoyStick_thread;

    bool speedSpin;

    QPalette normal_label, protective_label, emergency_label;
};

#endif // MAINWINDOW_H
