#ifndef COMMANDER_H
#define COMMANDER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "px4sim_planning/minimum_snap.h"

using namespace Eigen;

class TrajCommander
{
    private:
    ros::NodeHandle nh_;
    ros::Time t_now;
    MinimumSnap planner = MinimumSnap(4, 2.0, 1);
    ros::Time t_polynomial_start;
    VectorXd poly_times;
    MatrixXd poly_coeffs;

    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::Subscriber local_pos_sub;
    ros::Publisher local_vel_pub;
    ros::Subscriber local_vel_sub;
    ros::Subscriber target_pos_sub;
    ros::Subscriber target_vel_sub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    void initSubscribers();
    void initPublishers();
    void initServices();
    void initTimers();
    void subStateCb(const mavros_msgs::State::ConstPtr& msg);
    void subLocalPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subLocalVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void subTargetPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void subTargetVelCb(const std_msgs::Float64::ConstPtr& msg);

    Vector3d solvePolyPos(double t, VectorXd time_alloc, MatrixXd coeffs);

    public:
    ros::Timer timer1;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pos;
    geometry_msgs::TwistStamped current_vel;
    bool takeoff;
    bool poly_avaliable;
    Vector3d start_hover_pos;
    int at_pos_count;

    TrajCommander(ros::NodeHandle* nodehandle);
    ~TrajCommander(){};

    void timer1Cb(const ros::TimerEvent&);
    
};


#endif