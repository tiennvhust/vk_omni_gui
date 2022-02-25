#include <qpublish.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

QPublish::QPublish(ros::NodeHandle& nh)
{
    this->nh = &nh;
    velocityPublisher = this->nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
    plcCommandPublisher = this->nh->advertise<std_msgs::UInt8>("safety_command", 1);
}

QPublish::~QPublish() {}

void QPublish::velocityPublish(geometry_msgs::Twist msg)
{
    velocityPublisher.publish(msg);
}

void QPublish::plcCommandPublish(plcCommand msg)
{
    std_msgs::UInt8 cmd;
    cmd.data = static_cast<uint8_t>(msg);
    plcCommandPublisher.publish(cmd);
}
