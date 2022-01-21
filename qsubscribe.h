#ifndef QSUBSCRIBE_H
#define QSUBSCRIBE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <QObject>

class QSubscribe : public QObject
{
    Q_OBJECT
public:
    QSubscribe(ros::NodeHandle& nh);
    ~QSubscribe();

    void velocityReferenceUpdate(double data);
    geometry_msgs::Twist twistReference(double x, double y, double z);


public slots:
    void Subscribe();

signals:
    void velWheelSignal(sensor_msgs::JointState::ConstPtr);
    void odomSignal(nav_msgs::Odometry::ConstPtr);

private:
    ros::NodeHandle* nh;
    ros::Subscriber velocitySubsciber;
    ros::Subscriber odomSubscriber;

    void velocityCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif // QSUBSCRIBE_H
