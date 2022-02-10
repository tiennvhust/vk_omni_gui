#include "joystick.h"

JoyStick::JoyStick(QObject *parent) : QObject(parent)
{
    vel_ref = 0;
}

void JoyStick::setVelRef(double value)
{
    if (value != vel_ref)
    {
        vel_ref = value;
        emit velRefSignal(static_cast<int>(vel_ref * 100));
        if(static_cast<bool>(twist[0]) || static_cast<bool>(twist[1]) || static_cast<bool>(twist[2])) emit velRefPublishSignal(getTwist());
    }
}

void JoyStick::setVelRef(int value)
{
    if (value != static_cast<int>(vel_ref * 100))
    {
        vel_ref = static_cast<double>(value) / 100;
        emit velRefSignal(vel_ref);
        if(static_cast<bool>(twist[0]) || static_cast<bool>(twist[1]) || static_cast<bool>(twist[2])) emit velRefPublishSignal(getTwist());
    }
}

void JoyStick::setTwist(array<double, 3> data)
{
    twist = data;
    emit velRefPublishSignal(getTwist());
}

void JoyStick::upSpinVelRef(const bool* spin)
{
    while(*spin)
    {
        vel_ref += 0.01;
        emit velRefSignal(vel_ref);
        emit velRefSignal(static_cast<int>(vel_ref * 100));
        if(static_cast<bool>(twist[0]) || static_cast<bool>(twist[1]) || static_cast<bool>(twist[2])) emit velRefPublishSignal(getTwist());
        QThread::msleep(20);
    }
}

void JoyStick::downSpinVelRef(const bool* spin)
{
    while(*spin)
    {
        vel_ref -= 0.01;
        emit velRefSignal(vel_ref);
        emit velRefSignal(static_cast<int>(vel_ref * 100));
        if(static_cast<bool>(twist[0]) || static_cast<bool>(twist[1]) || static_cast<bool>(twist[2])) emit velRefPublishSignal(getTwist());
        QThread::msleep(20);
    }
}

geometry_msgs::Twist JoyStick::getTwist()
{
    geometry_msgs::Twist buffer;

    buffer.linear.x = twist[0] * vel_ref;
    buffer.linear.y = twist[1] * vel_ref;
    buffer.linear.z = twist[2] * vel_ref;

    return buffer;
}
