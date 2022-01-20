#include <qnode.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

Qnode::Qnode(ros::NodeHandle& nh_)
{
    nh = &nh_;
    velocityPublisher = nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
}

Qnode::~Qnode() {}

void Qnode::Publish(geometry_msgs::Twist msg)
{
    velocityPublisher.publish(msg);
}

//Update reference velocity
void Qnode::velocityReferenceUpdate(double data)
{
    velocityReference = data;
}

//Return twist value
geometry_msgs::Twist Qnode::twistReference(double x, double y, double z)
{
    geometry_msgs::Twist buffer;
    buffer.linear.x = x*velocityReference;
    buffer.linear.y = y*velocityReference;
    buffer.linear.z = z*velocityReference;
    return buffer;
}

