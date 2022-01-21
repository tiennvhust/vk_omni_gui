#include <qpublish.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

QPublish::QPublish(ros::NodeHandle& nh)
{
    this->nh = &nh;
}

QPublish::~QPublish() {}

void QPublish::Publish(geometry_msgs::Twist msg)
{
    velocityPublisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
    velocityPublisher.publish(msg);
}

void QPublish::setVelocityReference(double value)
{
    if (value != velocityReference)
    {
        velocityReference = value;
        emit velRefSignal(static_cast<int>(velocityReference * 100));
    }
}

void QPublish::setVelocityReference(int value)
{
    if (value != static_cast<int>(velocityReference * 100))
    {
        velocityReference = static_cast<double>(value) / 100;
        emit velRefSignal(velocityReference);
    }
}

//Update reference velocity
void QPublish::velocityReferenceUpdate(double value)
{
    velocityReference = value;
}

//Return twist value
geometry_msgs::Twist QPublish::twistReference(double x, double y, double z)
{
    geometry_msgs::Twist buffer;
    buffer.linear.x = x*velocityReference;
    buffer.linear.y = y*velocityReference;
    buffer.linear.z = z*velocityReference;
    return buffer;
}
