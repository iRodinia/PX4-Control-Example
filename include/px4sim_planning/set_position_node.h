#ifndef SET_POSITION_NODE
#define SET_POSITION_NODE

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

class SetPosition
{
    private:
    ros::NodeHandle nh_;
    ros::Publisher target_pub;
    void initPublishers();

    public:
    SetPosition(ros::NodeHandle* nodehandle);
    ~SetPosition(){};
    void wait_for_pos();
};


#endif