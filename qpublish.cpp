#include <qpublish.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

QPublish::QPublish(ros::NodeHandle& nh)
{
    this->nh = &nh;
    velocityPublisher = this->nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
}

QPublish::~QPublish() {}

void QPublish::Publish(geometry_msgs::Twist msg)
{
    velocityPublisher.publish(msg);
}
