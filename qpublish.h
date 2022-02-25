#ifndef QPublish_H
#define QPublish_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <QObject>
#include <QString>

enum plcCommand {reset = 22, status = 23};

class QPublish : public QObject
{
    Q_OBJECT
public:
    QPublish(ros::NodeHandle& nh);
    ~QPublish();

public slots:
    void velocityPublish(geometry_msgs::Twist);

    void plcCommandPublish(plcCommand);

private:
    ros::NodeHandle* nh;
    ros::Publisher velocityPublisher;
    ros::Publisher plcCommandPublisher;
};

#endif // QPublish_H
