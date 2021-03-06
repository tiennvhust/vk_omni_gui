#include <qsubscribe.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

QSubscribe::QSubscribe(ros::NodeHandle& nh)
{
    this->nh = &nh;
}

QSubscribe::~QSubscribe()
{
    ros::shutdown();
}

void QSubscribe::Subscribe()
{
    velocitySubsciber = nh->subscribe<std_msgs::Float64MultiArray>("wheels_speed", 10, &QSubscribe::velocityCallback, this);
    odomSubscriber = nh->subscribe<nav_msgs::Odometry>("odom", 10, &QSubscribe::odomCallback, this);
    statusSubscriber = nh->subscribe("robot_status", 10, &QSubscribe::statusCallback, this);
    //  run ROS loop
    ros::spin();
}

//Callback function for joint state subscriber
void QSubscribe::velocityCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    emit velWheelSignal(msg);
}

//Callback function for odometry subscriber
void QSubscribe::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    emit odomSignal(msg);
}

//Callback function for status subscriber
void QSubscribe::statusCallback(std_msgs::UInt8 msg)
{
    emit statusSignal(static_cast<robot_status>(msg.data));
}
