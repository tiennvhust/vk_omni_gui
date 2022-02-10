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
        if(static_cast<bool>(twist.linear.x) || static_cast<bool>(twist.linear.y) || static_cast<bool>(twist.linear.z)) emit velRefPublishSignal(twist);
    }
}

void JoyStick::setVelRef(int value)
{
    if (value != static_cast<int>(vel_ref * 100))
    {
        vel_ref = static_cast<double>(value) / 100;
        emit velRefSignal(vel_ref);
        if(static_cast<bool>(twist.linear.x) || static_cast<bool>(twist.linear.y) || static_cast<bool>(twist.linear.z)) emit velRefPublishSignal(twist);
    }
}

void JoyStick::setTwist(array<double, 3> data)
{
    twist.linear.x = data[0] * vel_ref;
    twist.linear.y = data[1] * vel_ref;
    twist.linear.z = data[2] * vel_ref;
    emit velRefPublishSignal(twist);
}

void JoyStick::upSpinVelRef(const bool* spin)
{
    while(*spin)
    {
        vel_ref += 0.01;
        emit velRefSignal(vel_ref);
        emit velRefSignal(static_cast<int>(vel_ref * 100));
        if(static_cast<bool>(twist.linear.x) || static_cast<bool>(twist.linear.y) || static_cast<bool>(twist.linear.z)) emit velRefPublishSignal(twist);
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
        if(static_cast<bool>(twist.linear.x) || static_cast<bool>(twist.linear.y) || static_cast<bool>(twist.linear.z)) emit velRefPublishSignal(twist);
        QThread::msleep(20);
    }
}
