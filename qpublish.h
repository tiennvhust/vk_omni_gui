#ifndef QPublish_H
#define QPublish_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <QObject>
#include <QString>


class QPublish : public QObject
{
    Q_OBJECT
public:
    QPublish(ros::NodeHandle& nh);
    ~QPublish();

public slots:
    void Publish(geometry_msgs::Twist);

private:
    ros::NodeHandle* nh;
    ros::Publisher velocityPublisher;
};

#endif // QPublish_H
