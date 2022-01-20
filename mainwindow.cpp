#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    p_Qnode(new Qnode(nh))
{
    ui->setupUi(this);

    ui->speedSpinBox->setRange(-1.0, 1.0);
    ui->speedSpinBox->setSingleStep(0.05);
    connect(this, SIGNAL(velSignal(geometry_msgs::Twist)), p_Qnode, SLOT(Publish(geometry_msgs::Twist)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

//Update reference velocity
void MainWindow::on_speedSpinBox_valueChanged(double data)
{
    this->p_Qnode->velocityReferenceUpdate(data);
}

//Buttons commands
void MainWindow::on_backward_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(0, -1, 0));
}

void MainWindow::on_backward_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_forward_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(0, 1, 0));
}

void MainWindow::on_forward_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_right_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(1, 0, 0));
}

void MainWindow::on_right_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}

void MainWindow::on_left_pressed()
{
    emit velSignal(this->p_Qnode->twistReference(-1, 0, 0));
}

void MainWindow::on_left_released()
{
    emit velSignal(this->p_Qnode->twistReference(0, 0, 0));
}
