#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QObject>
#include <QThread>

#include <geometry_msgs/Twist.h>
using namespace std;

class JoyStick : public QObject
{
    Q_OBJECT
public:
    explicit JoyStick(QObject *parent = nullptr);

signals:
    void velRefSignal(double);

    void velRefSignal(int);

    void velRefPublishSignal(geometry_msgs::Twist);

public slots:
    void setVelRef(double);

    void setVelRef(int);

    void setTwist(array<double, 3>);

    void upSpinVelRef(const bool*);

    void downSpinVelRef(const bool*);

private:
    double vel_ref;

    array<double, 3> twist;

    geometry_msgs::Twist getTwist();
};

#endif // JOYSTICK_H
