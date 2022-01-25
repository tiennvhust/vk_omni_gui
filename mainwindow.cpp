#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QtMath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    p_QPublish(new QPublish(nh)),
    p_QSubscribe(new QSubscribe(nh))
{
    qRegisterMetaType<sensor_msgs::JointState::ConstPtr>("sensor_msgs::JointState::ConstPtr");
    qRegisterMetaType<geometry_msgs::Twist>("geometry_msgs::Twist");
    qRegisterMetaType<nav_msgs::Odometry::ConstPtr>("nav_msgs::Odometry::ConstPtr");

    normal_label.setColor(QPalette::Window, Qt::green);
//    normal.setColor(QPalette::WindowText, Qt::white);
    protective_label.setColor(QPalette::Window, Qt::yellow);
    emergency_label.setColor(QPalette::Window, Qt::red);

    ui->setupUi(this);

    ui->speedSpinBox->setRange(-1.0, 1.0);
    ui->speedSpinBox->setSingleStep(0.05);
    ui->speedSpinBox->setValue(0.0);

    ui->speedSlider->setMinimum(0);
    ui->speedSlider->setValue(0);
    ui->speedSlider->setMaximum(100);
    ui->speedSlider->setSingleStep(1);

    ui->joy_stick->setStyleSheet("QGroupBox { \
                                    background-color: linen; \
                                    border: 2px solid gray; \
                                    border-radius: 5px; \
                                    color: white; \
                                    margin-top: 1ex;} /* leave space at the top for the title */ \
                                QGroupBox::title { \
                                    subcontrol-origin: margin; \
                                    subcontrol-position: top center; /* position at the top center */ \
                                    padding: 0 3px; \
                                    background-color: gray;} \
                                ");

    ui->speed_group->setStyleSheet("QGroupBox { \
                                   background-color: linen; \
                                   border: 2px solid gray; \
                                   border-radius: 5px; \
                                   color: white; \
                                   margin-top: 1ex;} /* leave space at the top for the title */ \
                               QGroupBox::title { \
                                   subcontrol-origin: margin; \
                                   subcontrol-position: top center; /* position at the top center */ \
                                   padding: 0 3px; \
                                   background-color: gray;} \
                               ");

    ui->odom_group->setStyleSheet("QGroupBox { \
                                  background-color: linen; \
                                  border: 2px solid gray; \
                                  border-radius: 5px; \
                                  color: white; \
                                  margin-top: 1ex;} /* leave space at the top for the title */ \
                              QGroupBox::title { \
                                  subcontrol-origin: margin; \
                                  subcontrol-position: top center; /* position at the top center */ \
                                  padding: 0 3px; \
                                  background-color: gray;} \
                              ");

    ui->forward->setStyleSheet("QPushButton \
                                    {background-color: #FF6A64; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC5450; \
                                    border-style: inset;} \
                                ");

    ui->backward->setStyleSheet("QPushButton \
                                    {background-color: #FF6A64; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC5450; \
                                    border-style: inset;} \
                                ");

    ui->left->setStyleSheet("QPushButton \
                                    {background-color: #FF6A64; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC5450; \
                                    border-style: inset;} \
                                ");

    ui->right->setStyleSheet("QPushButton \
                                    {background-color: #FF6A64; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC5450; \
                                    border-style: inset;} \
                                ");

    ui->forward_left->setStyleSheet("QPushButton \
                                    {background-color: steelblue; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #325D80; \
                                    border-style: inset;} \
                                ");

    ui->forward_right->setStyleSheet("QPushButton \
                                    {background-color: steelblue; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #325D80; \
                                    border-style: inset;} \
                                ");

    ui->backward_left->setStyleSheet("QPushButton \
                                    {background-color: steelblue; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #325D80; \
                                    border-style: inset;} \
                                ");
    
    ui->backward_right->setStyleSheet("QPushButton \
                                    {background-color: steelblue; \
                                    height: 50px; \
                                    border-style: outset; \
                                    border-radius: 10px; \
                                    border-width: 6px; \
                                    border-color: silver; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #325D80; \
                                    border-style: inset;} \
                                ");

    ui->speedSlider->setStyleSheet("QSlider::groove:horizontal { \
                                       height: 14px; /* the groove expands to the size of the slider by default. by giving it a height, it has a fixed size */ \
                                       background: lightblue; \
                                       margin: 2px 0;} \
                                   QSlider::handle:horizontal { \
                                       border: 1px solid gray; \
                                       background: gray; \
                                       margin: 0 -2px; \
                                       border-radius: 6px; \
                                       width: 18px;} \
                               ");

    ui->status_label->setText("Robot Status");
    ui->status_label->setAutoFillBackground(true);
//    ui->status_label->setPalette(normal_label);

    ui->battery_status->setValue(80);

    connect(this, SIGNAL(velSignal(geometry_msgs::Twist)),
            p_QPublish, SLOT(Publish(geometry_msgs::Twist)));

    connect(this, SIGNAL(subscribeSignal()),
            p_QSubscribe, SLOT(Subscribe()));

    //Velocity reference connections

    connect(ui->speedSpinBox, SIGNAL(valueChanged(double)),
            p_QPublish, SLOT(setVelocityReference(double)));

    connect(p_QPublish, SIGNAL(velRefSignal(double)),
            ui->speedSpinBox, SLOT(setValue(double)));

    connect(ui->speedSlider, SIGNAL(sliderMoved(int)),
            p_QPublish, SLOT(setVelocityReference(int)));

    connect(p_QPublish, SIGNAL(velRefSignal(int)),
            ui->speedSlider, SLOT(setValue(int)));

    //Wheel speed data connections
    connect(p_QSubscribe, SIGNAL(velWheelSignal(sensor_msgs::JointState::ConstPtr)),
            this, SLOT(setVelocityText(sensor_msgs::JointState::ConstPtr)));

    //Odometry data connections
    connect(p_QSubscribe, SIGNAL(odomSignal(nav_msgs::Odometry::ConstPtr)),
            this, SLOT(setOdomText(nav_msgs::Odometry::ConstPtr)));

    p_QPublish->moveToThread(&p_QPublish_thread);
    p_QSubscribe->moveToThread(&p_QSubscribe_thread);

    p_QPublish_thread.start();
    p_QSubscribe_thread.start();

    emit subscribeSignal();
}

MainWindow::~MainWindow()
{
    p_QPublish_thread.quit();
    p_QSubscribe_thread.quit();
    delete this->p_QPublish;
    delete this->p_QSubscribe;
    delete ui;
}

//Buttons commands
void MainWindow::on_backward_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(1, 0, 0));
}

void MainWindow::on_backward_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_forward_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(-1, 0, 0));
}

void MainWindow::on_forward_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_right_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(0, 1, 0));
}

void MainWindow::on_right_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_left_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(0, -1, 0));
}

void MainWindow::on_left_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_forward_right_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(-qSqrt(2)/2, qSqrt(2)/2, 0));
}

void MainWindow::on_forward_right_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_backward_right_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(qSqrt(2)/2, qSqrt(2)/2, 0));
}

void MainWindow::on_backward_right_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_backward_left_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(qSqrt(2)/2, -qSqrt(2)/2, 0));
}

void MainWindow::on_backward_left_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}

void MainWindow::on_forward_left_pressed()
{
    emit velSignal(this->p_QPublish->twistReference(-qSqrt(2)/2, -qSqrt(2)/2, 0));
}

void MainWindow::on_forward_left_released()
{
    emit velSignal(this->p_QPublish->twistReference(0, 0, 0));
}


//Display velocity data
void MainWindow::setVelocityText(const sensor_msgs::JointState::ConstPtr &msg)
{
    ui->left_wheel->setText(QString::number(msg->velocity[0]));
    ui->right_wheel->setText(QString::number(msg->velocity[1]));
    ui->rear_wheel->setText(QString::number(msg->velocity[2]));
}

//Display odometry data
void MainWindow::setOdomText(const nav_msgs::Odometry::ConstPtr &msg)
{
    ui->linear_x->setText(QString::number(msg->twist.twist.linear.x));
    ui->linear_y->setText(QString::number(msg->twist.twist.linear.y));
    ui->angular->setText(QString::number(msg->twist.twist.angular.z));
    ui->pose_x->setText(QString::number(msg->pose.pose.position.x));
    ui->pose_y->setText(QString::number(msg->pose.pose.position.y));
}

//Display robot status
void MainWindow::onStatusUpdate(robot_status msg)
{
    switch (msg)
    {
        case normal:
            ui->status_label->setText("Normal");
            ui->status_label->setPalette(normal_label);
            break;
        case protective:
            ui->status_label->setText("Protective");
            ui->status_label->setPalette(protective_label);
            break;
        case emergency:
            ui->status_label->setText("Emergency");
            ui->status_label->setPalette(emergency_label);
            break;
    }
}

void MainWindow::onBatteryUpdate(int msg)
{
    ui->battery_status->setValue(msg);
}

void MainWindow::on_battery_status_valueChanged(int value)
{
    if (value >= 0 && value < 50)
        ui->battery_status->setStyleSheet("QProgressBar \
                                          {border: 2px solid grey; \
                                          border-radius: 5px; \
                                          text-align: center; \
                                          } \
                                          QProgressBar::chunk \
                                          {background-color: #65E629; \
                                          width: 10px; \
                                          margin: 0.5px; \
                                          }");
    else
        ui->battery_status->setStyleSheet("QProgressBar \
                                          {border: 2px solid grey; \
                                          border-radius: 5px; \
                                          text-align: center; \
                                          } \
                                          QProgressBar::chunk \
                                          {background-color: #05B8CC; \
                                          width: 10px; \
                                          margin: 0.5px; \
                                          }");
}
