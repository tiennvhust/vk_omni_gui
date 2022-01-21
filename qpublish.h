#ifndef QPublish_H
#define QPublish_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <QObject>
#include <QString>


class QPublish : public QObject
{
    Q_OBJECT
public:
    QPublish(ros::NodeHandle& nh);
    ~QPublish();

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

#endif // QPublish_H
