#ifndef QNODE_H
#define QNODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <QObject>

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

private:
    ros::NodeHandle* nh;
    ros::Publisher velocityPublisher;

    double velocityReference;
};

#endif // QNODE_H
