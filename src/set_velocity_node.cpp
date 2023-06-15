#include "px4sim_planning/set_velocity_node.h"

using namespace std;

SetVelocity::SetVelocity(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initPublishers();
}

void SetVelocity::initPublishers(){
    target_pub = nh_.advertise<std_msgs::Float64>("vel_cmd", 1);
}

void SetVelocity::wait_for_vel(){
    ROS_INFO("Input target velocity:");
    float v;
    cin >> v;
    std_msgs::Float64 vt;
    vt.data = v;
    target_pub.publish(vt);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_vel_node");
    ros::NodeHandle nh;
    SetVelocity cmder(&nh);
    while(ros::ok()){
        cmder.wait_for_vel();
        ros::spinOnce();
    }
    return 0;
}