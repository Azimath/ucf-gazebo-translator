#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/BodyRequest.h>

#include <algorithm>
//More thrust reverse
//Minimum thrust side to side
//More downwards

//Top thuster to bottom thruster 0.1633
//Side thuster to side thuster 0.1632
//Front thruster to back thruster 0.216

//Maximum bidirectional thrust/torque a single thruster can provide
#define MAX_THRUST 29.42

#define MAX_TORQUE_ROLL MAX_THRUST * 0.1633
#define MAX_TORQUE_YAW MAX_THRUST * 0.1632
#define MAX_TORQUE_PITCH MAX_THRUST * 0.216

class GazeboInterface
{
public:
    GazeboInterface();

    ~GazeboInterface();

    ros::NodeHandle nh_;
    ros::Subscriber commandSub;
    ros::ServiceClient gazeboWrenchCaller;
    ros::ServiceClient gazeboStopCaller;

    void commandCb(const geometry_msgs::Twist &msg);

};
