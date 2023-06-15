#include "px4sim_planning/set_position_node.h"

using namespace std;

SetPosition::SetPosition(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initPublishers();
}

void SetPosition::initPublishers(){
    target_pub = nh_.advertise<geometry_msgs::PoseStamped>("pos_cmd", 1);
}

void SetPosition::wait_for_pos(){
    ROS_INFO("Input target position in x y z:");
    float x, y, z;
    cin >> x >> y >> z;
    geometry_msgs::PoseStamped _pose;
    _pose.header.stamp = ros::Time::now();
    _pose.pose.position.x = x;
    _pose.pose.position.y = y;
    _pose.pose.position.z = z;
    target_pub.publish(_pose);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmder_node");
    ros::NodeHandle nh;
    SetPosition cmder(&nh);
    while(ros::ok()){
        cmder.wait_for_pos();
        ros::spinOnce();
    }
    return 0;
}
