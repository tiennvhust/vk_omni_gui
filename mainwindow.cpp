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
    ui->speedSpinBox->setValue(0.0);

    ui->speedSlider->setMinimum(0);
    ui->speedSlider->setValue(0);
    ui->speedSlider->setMaximum(100);
    ui->speedSlider->setSingleStep(1);

    connect(this, SIGNAL(velSignal(geometry_msgs::Twist)),
            p_Qnode, SLOT(Publish(geometry_msgs::Twist)));

    connect(ui->speedSpinBox, SIGNAL(valueChanged(double)),
            p_Qnode, SLOT(setVelocityReference(double)));

    connect(p_Qnode, SIGNAL(velRefSignal(double)),
            ui->speedSpinBox, SLOT(setValue(double)));

    connect(ui->speedSlider, SIGNAL(sliderMoved(int)),
            p_Qnode, SLOT(setVelocityReference(int)));

    connect(p_Qnode, SIGNAL(velRefSignal(int)),
            ui->speedSlider, SLOT(setValue(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
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
