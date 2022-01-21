#include <qnode.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

Qnode::Qnode(ros::NodeHandle& nh_)
{
    nh = &nh_;
    velocityPublisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
    velocitySubsciber = nh->subscribe<sensor_msgs::JointState>("/robot_kist/joint_states", 10, &Qnode::velocityCallback, this);
    odomSubscriber = nh->subscribe<nav_msgs::Odometry>("odom", 10, &Qnode::odomCallback, this);
}

Qnode::~Qnode() {}

void Qnode::Publish(geometry_msgs::Twist msg)
{
    velocityPublisher.publish(msg);
}

void Qnode::setVelocityReference(double value)
{
    if (value != velocityReference)
    {
        velocityReference = value;
        emit velRefSignal(static_cast<int>(velocityReference * 100));
    }
}

void Qnode::setVelocityReference(int value)
{
    if (value != static_cast<int>(velocityReference * 100))
    {
        velocityReference = static_cast<double>(value) / 100;
        emit velRefSignal(velocityReference);
    }
}

//Update reference velocity
void Qnode::velocityReferenceUpdate(double value)
{
    velocityReference = value;
}

//Return twist value
geometry_msgs::Twist Qnode::twistReference(double x, double y, double z)
{
    geometry_msgs::Twist buffer;
    buffer.linear.x = x*velocityReference;
    buffer.linear.y = y*velocityReference;
    buffer.linear.z = z*velocityReference;
    return buffer;
}

//Callback function for joint state subscriber
void Qnode::velocityCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    emit velWheelSignal(msg);
}

//Callback function for odometry subscriber
void Qnode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    emit odomSignal(msg);
}
