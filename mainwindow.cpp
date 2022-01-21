#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QString>
#include <QtMath>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    p_Qnode(new Qnode(nh))
{
    ui->setupUi(this);

    ui->speedSpinBox->setRange(-1.0, 1.0);
    ui->speedSpinBox->setSingleStep(0.05);
    ui->speedSpinBox->setValue(0.0);

    ui->speedSlider->setMinimum(0);
    ui->speedSlider->setValue(0);
    ui->speedSlider->setMaximum(100);
    ui->speedSlider->setSingleStep(1);

    connect(this, SIGNAL(velSignal(geometry_msgs::Twist)),
            p_Qnode, SLOT(Publish(geometry_msgs::Twist)));

    //Velocity reference connections

    connect(ui->speedSpinBox, SIGNAL(valueChanged(double)),
            p_Qnode, SLOT(setVelocityReference(double)));

    connect(p_Qnode, SIGNAL(velRefSignal(double)),
            ui->speedSpinBox, SLOT(setValue(double)));

    connect(ui->speedSlider, SIGNAL(sliderMoved(int)),
            p_Qnode, SLOT(setVelocityReference(int)));

    connect(p_Qnode, SIGNAL(velRefSignal(int)),
            ui->speedSlider, SLOT(setValue(int)));

    //Wheel speed data connections
    connect(p_Qnode, SIGNAL(velWheelSignal(const sensor_msgs::JointState::ConstPtr&)),
            this, SLOT(setVelocityText(const sensor_msgs::JointState::ConstPtr&)));
//    connect(this->p_Qnode, SIGNAL(velLeft(const QString&)),
//            ui->left_wheel, SLOT(setText(const QString&)));

//    connect(this->p_Qnode, SIGNAL(velRight(const QString&)),
//            ui->right_wheel, SLOT(setText(const QString&)));

//    connect(this->p_Qnode, SIGNAL(velRear(const QString&)),
//            ui->rear_wheel, SLOT(setText(const QString&)));

    //Odometry data connections
    connect(p_Qnode, SIGNAL(odomSignal(const nav_msgs::Odometry::ConstPtr&)),
            this, SLOT(setOdomText(const nav_msgs::Odometry::ConstPtr&)));
//    connect(this->p_Qnode, SIGNAL(velLinearX(const Qstring&)),
//            ui->linear_x, SLOT(setText(const Qstring&)));

//    connect(this->p_Qnode, SIGNAL(velLinearY(const Qstring&)),
//            ui->linear_y, SLOT(setText(const Qstring&)));

//    connect(this->p_Qnode, SIGNAL(velAngular(const Qstring&)),
//            ui->angular, SLOT(setText(const Qstring&)));

//    connect(this->p_Qnode, SIGNAL(posePointX(const Qstring&)),
//            ui->pose_x, SLOT(setText(const Qstring&)));

//    connect(this->p_Qnode, SIGNAL(posePointY(const Qstring&)),
//            ui->pose_y, SLOT(setText(const Qstring&)));

}

MainWindow::~MainWindow()
{
    delete this->p_Qnode;
    delete ui;
}

//Buttons commands
void MainWindow::on_backward_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(1, 0, 0));
}

void MainWindow::on_backward_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_forward_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(-1, 0, 0));
}

void MainWindow::on_forward_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_right_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(0, 1, 0));
}

void MainWindow::on_right_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_left_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(0, -1, 0));
}

void MainWindow::on_left_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_forward_right_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(-qSqrt(2)/2, qSqrt(2)/2, 0));
}

void MainWindow::on_forward_right_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_backward_right_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(qSqrt(2)/2, qSqrt(2)/2, 0));
}

void MainWindow::on_backward_right_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_backward_left_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(qSqrt(2)/2, -qSqrt(2)/2, 0));
}

void MainWindow::on_backward_left_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_forward_left_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(-qSqrt(2)/2, -qSqrt(2)/2, 0));
}

void MainWindow::on_forward_left_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
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
