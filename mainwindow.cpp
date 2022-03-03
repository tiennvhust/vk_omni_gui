#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QtMath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    p_QPublish(new QPublish(nh)),
    p_QSubscribe(new QSubscribe(nh)),
    p_JoyStick(new JoyStick()),
    QPublish_thread(),
    QSubscribe_thread(),
    JoyStick_thread()
{
    qRegisterMetaType<std_msgs::Float64MultiArray::ConstPtr>("std_msgs::Float64MultiArray::ConstPtr");
    qRegisterMetaType<geometry_msgs::Twist>("geometry_msgs::Twist");
    qRegisterMetaType<nav_msgs::Odometry::ConstPtr>("nav_msgs::Odometry::ConstPtr");
    qRegisterMetaType<array<double,3>>("array<double,3>");
    qRegisterMetaType<plcCommand>("plcCommand");
    qRegisterMetaType<robot_status>("robot_status");

    QPixmap up(":/up.png");
    QPixmap down(":/down.png");
    QPixmap left(":/left.png");
    QPixmap left_up(":/left_up.png");
    QPixmap left_down(":/left_down.png");
    QPixmap right(":/right.png");
    QPixmap stop(":/stop.png");
    QPixmap right_up(":/right_up.png");
    QPixmap right_down(":/right_down.png");
    QPixmap plus(":/plus.png");
    QPixmap minus(":/minus.png");
    QPixmap ready(":/tick.png");

    QIcon up_icon(up);
    QIcon down_icon(down);
    QIcon left_icon(left);
    QIcon left_up_icon(left_up);
    QIcon left_down_icon(left_down);
    QIcon rigth_icon(right);
    QIcon right_up_icon(right_up);
    QIcon right_down_icon(right_down);
    QIcon stop_icon(stop);
    QIcon plus_icon(plus);
    QIcon minus_icon(minus);
    QIcon ready_icon(ready);

    normal_label.setColor(QPalette::Window, Qt::green);
//    normal.setColor(QPalette::WindowText, Qt::white);
    protective_label.setColor(QPalette::Window, Qt::yellow);
    emergency_label.setColor(QPalette::Window, Qt::red);

    ui->setupUi(this);

    ui->speedSpinBox->setRange(-1.0, 1.0);
    ui->speedSpinBox->setSingleStep(0.05);
    ui->speedSpinBox->setValue(0.0);

    ui->speedSlider->setMinimum(-100);
    ui->speedSlider->setValue(0);
    ui->speedSlider->setMaximum(100);
    ui->speedSlider->setSingleStep(1);

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
                                    {background-color: lightsalmon; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC8062; \
                                    border-style: inset;} \
                                ");
    ui->forward->setIcon(up_icon);
    ui->forward->setIconSize(QSize(30, 30));

    ui->backward->setStyleSheet("QPushButton \
                                    {background-color: lightsalmon; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC8062; \
                                    border-style: inset;} \
                                ");
    ui->backward->setIcon(down_icon);
    ui->backward->setIconSize(QSize(30, 30));

    ui->left->setStyleSheet("QPushButton \
                                    {background-color: lightsalmon; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC8062; \
                                    border-style: inset;} \
                                ");
    ui->left->setIcon(left_icon);
    ui->left->setIconSize(QSize(30, 30));

    ui->right->setStyleSheet("QPushButton \
                                    {background-color: lightsalmon; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CC8062; \
                                    border-style: inset;} \
                                ");
    ui->right->setIcon(rigth_icon);
    ui->right->setIconSize(QSize(30, 30));

    ui->forward_left->setStyleSheet("QPushButton \
                                    {background-color: lightsteelblue; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #8797AB; \
                                    border-style: inset;} \
                                ");
    ui->forward_left->setIcon(left_up_icon);
    ui->forward_left->setIconSize(QSize(30, 30));

    ui->forward_right->setStyleSheet("QPushButton \
                                    {background-color: lightsteelblue; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #8797AB; \
                                    border-style: inset;} \
                                ");
    ui->forward_right->setIcon(right_up_icon);
    ui->forward_right->setIconSize(QSize(30, 30));

    ui->backward_left->setStyleSheet("QPushButton \
                                    {background-color: lightsteelblue; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #8797AB; \
                                    border-style: inset;} \
                                ");

    ui->backward_left->setIcon(left_down_icon);
    ui->backward_left->setIconSize(QSize(30, 30));
    
    ui->backward_right->setStyleSheet("QPushButton \
                                    {background-color: lightsteelblue; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #8797AB; \
                                    border-style: inset;} \
                                ");
    ui->backward_right->setIcon(right_down_icon);
    ui->backward_right->setIconSize(QSize(30, 30));

    ui->break_button->setStyleSheet("QPushButton \
                                    {background-color: lightcoral; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #BD6464; \
                                    border-style: inset;} \
                                ");
    ui->break_button->setIcon(stop_icon);
    ui->break_button->setIconSize(QSize(30, 30));

    ui->speedup->setStyleSheet("QPushButton \
                                    {background-color: ghostwhite; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #C6C6CC; \
                                    border-style: inset;} \
                                ");
    ui->speedup->setIcon(plus_icon);
    ui->speedup->setIconSize(QSize(30, 30));

    ui->speeddown->setStyleSheet("QPushButton \
                                    {background-color: ghostwhite; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #C6C6CC; \
                                    border-style: inset;} \
                                ");
    ui->speeddown->setIcon(minus_icon);
    ui->speeddown->setIconSize(QSize(30, 30));

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

    ui->resetButton->setStyleSheet("QPushButton \
                                    {background-color: bisque; \
                                    border-style: solid; \
                                    border-radius: 10px; \
                                    border-width: 4px; \
                                    border-color: lightgray; \
                                    padding: 6px; \
                                    color: white;} \
                                QPushButton:pressed \
                                    {background-color: #CCB79D; \
                                    border-style: inset;} \
                                ");
    ui->resetButton->setIcon(ready_icon);
    ui->resetButton->setIconSize(QSize(60, 60));

    ui->status_label->setText("Robot Status");
    ui->status_label->setAutoFillBackground(true);
//    ui->status_label->setPalette(normal_label);

    ui->battery_status->setValue(80);

    connect(this, SIGNAL(subscribeSignal()),
            p_QSubscribe, SLOT(Subscribe()));

    connect(p_JoyStick, SIGNAL(velRefPublishSignal(geometry_msgs::Twist)),
            p_QPublish, SLOT(velocityPublish(geometry_msgs::Twist)));

    //Joystick connections
    connect(ui->speedSpinBox, SIGNAL(valueChanged(double)),
            p_JoyStick, SLOT(setVelRef(double)));

    connect(p_JoyStick, SIGNAL(velRefSignal(double)),
            ui->speedSpinBox, SLOT(setValue(double)));

    connect(ui->speedSlider, SIGNAL(sliderMoved(int)),
            p_JoyStick, SLOT(setVelRef(int)));

    connect(p_JoyStick, SIGNAL(velRefSignal(int)),
            ui->speedSlider, SLOT(setValue(int)));

    connect(this, SIGNAL(upSpinSignal(const bool*)),
            p_JoyStick, SLOT(upSpinVelRef(const bool*)));

    connect(this, SIGNAL(downSpinSignal(const bool*)),
            p_JoyStick, SLOT(downSpinVelRef(const bool*)));

    connect(this, SIGNAL(velSignal(array<double, 3>)),
            p_JoyStick, SLOT(setTwist(array<double, 3>)));

    //Wheel speed data connections
    connect(p_QSubscribe, SIGNAL(velWheelSignal(sensor_msgs::JointState::ConstPtr)),
            this, SLOT(setVelocityText(sensor_msgs::JointState::ConstPtr)));

    //Odometry data connections
    connect(p_QSubscribe, SIGNAL(odomSignal(nav_msgs::Odometry::ConstPtr)),
            this, SLOT(setOdomText(nav_msgs::Odometry::ConstPtr)));

    //PLC connections
    connect(p_QSubscribe, SIGNAL(statusSignal(robot_status)),
            this, SLOT(onStatusUpdate(robot_status)));

    connect(this, SIGNAL(plcCommandSignal(plcCommand)),
            p_QPublish, SLOT(plcCommandPublish(plcCommand)));

    p_QPublish->moveToThread(&QPublish_thread);
    p_QSubscribe->moveToThread(&QSubscribe_thread);
    p_JoyStick->moveToThread(&JoyStick_thread);

    QPublish_thread.start();
    QSubscribe_thread.start();
    JoyStick_thread.start();

    emit subscribeSignal();

    ui->cover_02->hide();
}

MainWindow::~MainWindow()
{
    QPublish_thread.quit();
    QSubscribe_thread.quit();
    JoyStick_thread.quit();
    delete this->p_QPublish;
    delete this->p_QSubscribe;
    delete this->p_JoyStick;
    delete ui;
}

//Buttons commands
void MainWindow::on_backward_clicked()
{
    emit velSignal({1, 0, 0});
}

void MainWindow::on_forward_clicked()
{
    emit velSignal({-1, 0, 0});
}

void MainWindow::on_right_clicked()
{
    emit velSignal({0, 1, 0});
}

void MainWindow::on_left_clicked()
{
    emit velSignal({0, -1, 0});
}

void MainWindow::on_forward_right_clicked()
{
    emit velSignal({-qSqrt(2)/2, qSqrt(2)/2, 0});
}

void MainWindow::on_backward_right_clicked()
{
    emit velSignal({qSqrt(2)/2, qSqrt(2)/2, 0});
}

void MainWindow::on_backward_left_clicked()
{
    emit velSignal({qSqrt(2)/2, -qSqrt(2)/2, 0});
}

void MainWindow::on_forward_left_clicked()
{
    emit velSignal({-qSqrt(2)/2, -qSqrt(2)/2, 0});
}

void MainWindow::on_break_button_clicked()
{
    emit velSignal({0, 0, 0});
}


//Display velocity data
void MainWindow::setVelocityText(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    int left_speed = static_cast<int>(msg->data[0] * 100);
    int right_speed = static_cast<int>(msg->data[1] * 100);
    int rear_speed = static_cast<int>(msg->data[2] * 100);

    ui->left_wheel->setText(QString::number(static_cast<double>(left_speed / 100.0)));
    ui->right_wheel->setText(QString::number(static_cast<double>(right_speed / 100.0)));
    ui->rear_wheel->setText(QString::number(static_cast<double>(rear_speed / 100.0)));
}

//Display odometry data
void MainWindow::setOdomText(const nav_msgs::Odometry::ConstPtr &msg)
{
    int twist_x = static_cast<int>(msg->twist.twist.linear.x * 100);
    int twist_y = static_cast<int>(msg->twist.twist.linear.y * 100);
    int twist_z = static_cast<int>(msg->twist.twist.angular.z * 100);
    int pose_x = static_cast<int>(msg->pose.pose.position.x * 100);
    int pose_y = static_cast<int>(msg->pose.pose.position.y * 100);

    ui->linear_x->setText(QString::number(static_cast<double>(twist_x / 100.0)));
    ui->linear_y->setText(QString::number(static_cast<double>(twist_y / 100.0)));
    ui->angular->setText(QString::number(static_cast<double>(twist_z / 100.0)));
    ui->pose_x->setText(QString::number(static_cast<double>(pose_x / 100.0)));
    ui->pose_y->setText(QString::number(static_cast<double>(pose_y / 100.0)));
}

//Display robot status
void MainWindow::onStatusUpdate(robot_status msg)
{
    switch (msg)
    {
        case normal:
            ui->status_label->setText("Normal");
            ui->status_label->setPalette(normal_label);
            ui->cover_01->hide();
            ui->cover_02->hide();
            break;
        case protective:
            ui->status_label->setText("Protective");
            ui->status_label->setPalette(protective_label);
            ui->cover_01->show();
            break;
        case emergency:
            ui->status_label->setText("Emergency");
            ui->status_label->setPalette(emergency_label);
            ui->cover_02->show();
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

void MainWindow::on_speedup_pressed()
{
    speedSpin = true;
    emit upSpinSignal(&speedSpin);
}

void MainWindow::on_speeddown_pressed()
{
    speedSpin = true;
    emit downSpinSignal(&speedSpin);
}

void MainWindow::on_speedup_released()
{
    speedSpin = false;
}

void MainWindow::on_speeddown_released()
{
    speedSpin = false;
}

void MainWindow::on_resetButton_clicked()
{
    emit plcCommandSignal(reset);
}
