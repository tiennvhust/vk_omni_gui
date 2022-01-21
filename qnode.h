#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <QObject>
#include <QString>


class Qnode : public QObject
{
    Q_OBJECT
public:
    Qnode(ros::NodeHandle& nh_);
    ~Qnode();

    void velocityReferenceUpdate(double data);
    geometry_msgs::Twist twistReference(double x, double y, double z);


public slots:
    void Publish(geometry_msgs::Twist);
    void setVelocityReference(double);
    void setVelocityReference(int);

signals:
    void velRefSignal(double);
    void velRefSignal(int);

    void velLeft(const QString&);
    void velRight(const QString&);
    void velRear(const QString&);

    void velLinearX(const QString&);
    void velLinearY(const QString&);
    void velAngular(const QString&);

    void posePointX(const QString&);
    void posePointY(const QString&);

private:
    ros::NodeHandle* nh;
    ros::Publisher velocityPublisher;
    ros::Subscriber velocitySubsciber;
    ros::Subscriber odomSubscriber;

    double velocityReference;

    void velocityCallback(sensor_msgs::JointState);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

#endif // QNODE_H
