#ifndef QSUBSCRIBE_H
#define QSUBSCRIBE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <QObject>
#include <std_msgs/UInt8.h>

enum robot_status {normal = 12, protective = 11, emergency = 10};

class QSubscribe : public QObject
{
    Q_OBJECT
public:
    QSubscribe(ros::NodeHandle& nh);
    ~QSubscribe();

public slots:
    void Subscribe();

signals:
    void velWheelSignal(sensor_msgs::JointState::ConstPtr);
    void odomSignal(nav_msgs::Odometry::ConstPtr);
    void statusSignal(robot_status);

private:
    ros::NodeHandle* nh;
    ros::Subscriber velocitySubsciber;
    ros::Subscriber odomSubscriber;
    ros::Subscriber statusSubscriber;

    void velocityCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void statusCallback(std_msgs::UInt8 msg);
};

#endif // QSUBSCRIBE_H
