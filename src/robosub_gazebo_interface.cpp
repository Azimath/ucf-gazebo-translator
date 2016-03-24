#include <robosub_gazebo_interface.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gate_finder");
    GazeboInterface gi;
    ros::spin();
    return 0;
}

template <typename T> int sgn(T val) {  //http://stackoverflow.com/a/4609795
    return (T(0) < val) - (val < T(0));
}

GazeboInterface::GazeboInterface()
{
    commandSub = nh_.subscribe("/robosubsim/thusters", 1000, &GazeboInterface::commandCb, this);
    gazeboWrenchCaller = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazeboStopCaller = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/clear_body_wrenches");
}

GazeboInterface::~GazeboInterface()
{

}

void GazeboInterface::commandCb(const geometry_msgs::Twist &msg)
{
    gazebo_msgs::BodyRequest stop;
    stop.request.body_name = "ucf_submarine_simple";

    gazebo_msgs::ApplyBodyWrench wrench;
    wrench.request.body_name = "ucf_submarine_simple::body";
    wrench.request.reference_frame = "ucf_submarine_simple::body";

    gazebo_msgs::ApplyBodyWrench bouyancyWrench;
    bouyancyWrench.request.body_name = "ucf_submarine_simple::body";
    bouyancyWrench.request.reference_frame = "world";

    gazebo_msgs::ApplyBodyWrench weightWrench;
    weightWrench.request.body_name = "ucf_submarine_simple::body";
    weightWrench.request.reference_frame = "world";

    //You can't deliver maximum thrust and maximum torque on the same axis, so mix things to represent this
    //+x forward, +z down, +y right
    float tFrontUp, tRearUp, tLeftForward, tRightForward, tTopStrafe, tBottomStrafe;

    tFrontUp = std::max(-1.0, std::min(1.0, msg.linear.z + msg.angular.y));
    tRearUp = std::max(-1.0, std::min(1.0, msg.linear.z - msg.angular.y));

    tLeftForward = std::max(-1.0, std::min(1.0, msg.linear.x - msg.angular.z));
    tRightForward = std::max(-1.0, std::min(1.0, msg.linear.x + msg.angular.z));

    tTopStrafe = std::max(-1.0, std::min(1.0, msg.linear.y - msg.angular.x));
    tBottomStrafe = std::max(-1.0, std::min(1.0, msg.linear.y + msg.angular.x));

    wrench.request.wrench.force.x = (tLeftForward + tRightForward) * MAX_THRUST;
    wrench.request.wrench.force.y = (tTopStrafe + tBottomStrafe) * MAX_THRUST;
    wrench.request.wrench.force.z = (tFrontUp + tRearUp) * MAX_THRUST;

    wrench.request.wrench.torque.x = (tBottomStrafe - tTopStrafe) * MAX_TORQUE_ROLL;
    wrench.request.wrench.torque.y = (tFrontUp - tRearUp) * MAX_TORQUE_PITCH;
    wrench.request.wrench.torque.z = (tRightForward - tBottomStrafe) * MAX_TORQUE_YAW;

    wrench.request.duration.sec = -1;
    wrench.request.start_time.sec = 0;

    weightWrench.request.wrench.force.z = -10.0;
    weightWrench.request.reference_point.z = 1.0;

    weightWrench.request.duration.sec = -1;
    weightWrench.request.start_time.sec = 0;

    bouyancyWrench.request.wrench.force.z = 10.0;
    bouyancyWrench.request.reference_point.z = 0.0;

    bouyancyWrench.request.duration.sec = -1;
    bouyancyWrench.request.start_time.sec = 0;

    gazeboStopCaller.call(stop);
    gazeboWrenchCaller.call(wrench);
    //gazeboWrenchCaller.call(bouyancyWrench);
    //gazeboWrenchCaller.call(weightWrench);

}
