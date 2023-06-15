#ifndef SET_VELOCITY_NODE
#define SET_VELOCITY_NODE

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

class SetVelocity
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher target_pub;
    void initPublishers();

    public:
    SetVelocity(ros::NodeHandle* nodehandle);
    ~SetVelocity(){};
    void wait_for_vel();
};


#endif