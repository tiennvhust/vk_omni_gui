#include <qsubscribe.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

QSubscribe::QSubscribe(ros::NodeHandle& nh)
{
    this->nh = &nh;
}

QSubscribe::~QSubscribe()
{
}

void QSubscribe::Subscribe()
{
    velocitySubsciber = nh->subscribe<sensor_msgs::JointState>("/robot_kist/joint_states", 10, &QSubscribe::velocityCallback, this);
    odomSubscriber = nh->subscribe<nav_msgs::Odometry>("odom", 10, &QSubscribe::odomCallback, this);
    //run ROS loop
    ros::Rate loop_rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

//Callback function for joint state subscriber
void QSubscribe::velocityCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    emit velWheelSignal(msg);
}

//Callback function for odometry subscriber
void QSubscribe::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    emit odomSignal(msg);
}
