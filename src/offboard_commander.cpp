#include "px4sim_planning/offboard_commander.h"

#include <iostream>

TrajCommander::TrajCommander(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    initSubscribers();
    initPublishers();
    initServices();
    initTimers();
    takeoff = false;
    poly_avaliable = false;
    start_hover_pos = Vector3d(0.0, 0.0, 2.0);
    at_pos_count = 0;
}

void TrajCommander::initSubscribers(){
    state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &TrajCommander::subStateCb, this);
    local_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &TrajCommander::subLocalPosCb, this);
    local_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, &TrajCommander::subLocalVelCb, this);
    target_pos_sub = nh_.subscribe<geometry_msgs::PoseStamped>("pos_cmd", 2, &TrajCommander::subTargetPosCb, this);
    target_vel_sub = nh_.subscribe<std_msgs::Float64>("vel_cmd", 2, &TrajCommander::subTargetVelCb, this);
}

void TrajCommander::initPublishers(){
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    local_vel_pub = nh_.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
}

void TrajCommander::initServices(){
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
}

void TrajCommander::initTimers(){
    timer1 = nh_.createTimer(ros::Rate(20), &TrajCommander::timer1Cb, this);
}

void TrajCommander::subStateCb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void TrajCommander::subLocalPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}

void TrajCommander::subLocalVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_vel = *msg;
}

void TrajCommander::subTargetVelCb(const std_msgs::Float64::ConstPtr& msg){
    double vel = (*msg).data;
    if(vel <= 0){
        ROS_INFO("Invalid velocity!");
    }
    else{
        planner = MinimumSnap(4, vel, 1);
        ROS_INFO("Set average velocity to %f", vel);
    }
}

void TrajCommander::subTargetPosCb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    geometry_msgs::PoseStamped target_pos = *msg;
    MatrixXd way_pts(2,3);
    way_pts << current_pos.pose.position.x, current_pos.pose.position.y, current_pos.pose.position.z,
                target_pos.pose.position.x, target_pos.pose.position.y, target_pos.pose.position.z;
    VectorXd time_alloc = planner.AllocateTime(way_pts);  
    // VectorXd start_vel = Vector3d(current_vel.twist.linear.x, current_vel.twist.linear.y, current_vel.twist.linear.z);
    // VectorXd end_vel = Vector3d::Zero();
    MatrixXd start_vel(1,3);
    start_vel << current_vel.twist.linear.x, current_vel.twist.linear.y, current_vel.twist.linear.z;
    MatrixXd end_vel(1,3);
    end_vel << 0, 0, 0;

    MatrixXd coefs = planner.SolveQPClosedForm(way_pts, time_alloc, start_vel, end_vel);

    std::cout << "Polynomial coefficients:";
    std::cout << coefs << std::endl;
    

    poly_times = time_alloc;
    poly_coeffs = coefs;
    t_polynomial_start = ros::Time::now();
    poly_avaliable = true;
}

Vector3d TrajCommander::solvePolyPos(double t, VectorXd time_alloc, MatrixXd coeffs){
    double t_quire = t;
    const unsigned int poly_order = planner.GetPolyOrder();
    Vector3d result = Vector3d::Zero();
    for(int i=0; i<time_alloc.cols(); i++){
        if(t_quire <= time_alloc(i)){
             for(int j=0; j<=poly_order; j++){
                result += pow(t_quire, j) * coeffs.row(i*(poly_order+1)+j).transpose();
             }
             return result;
        }
        else{
            t_quire = t_quire - time_alloc(i);
        }
    }
    for(int j=0; j<=poly_order; j++){
        result += pow(time_alloc(time_alloc.cols()-1), j) * coeffs.row((time_alloc.cols()-1)*(poly_order+1)+j).transpose();
    }
    return result;
}

void TrajCommander::timer1Cb(const ros::TimerEvent&){
    t_now = ros::Time::now();
    if(current_state.connected){
        geometry_msgs::PoseStamped pos_cmd;
        if(current_state.mode=="OFFBOARD" && current_state.armed){
            if(!takeoff){
                pos_cmd.pose.position.x = start_hover_pos(0);
                pos_cmd.pose.position.y = start_hover_pos(1);
                pos_cmd.pose.position.z = start_hover_pos(2);
                if(pow(current_pos.pose.position.x-pos_cmd.pose.position.x,2) +
                    pow(current_pos.pose.position.y-pos_cmd.pose.position.y,2) +
                    pow(current_pos.pose.position.z-pos_cmd.pose.position.z,2) <= 0.01){
                    if(at_pos_count > 150){
                        at_pos_count = 0;
                        takeoff = true;
                    }
                    else{
                        at_pos_count++;
                    }
                }
                else{
                    at_pos_count = 0;
                }
            }
            else if(!poly_avaliable){
                pos_cmd.pose.position.x = start_hover_pos(0);
                pos_cmd.pose.position.y = start_hover_pos(1);
                pos_cmd.pose.position.z = start_hover_pos(2);
                ROS_INFO_ONCE("Ready for offboard control!");
            }
            else{
                Vector3d cmd_pos = solvePolyPos((t_now-t_polynomial_start).toSec(), poly_times, poly_coeffs);
                pos_cmd.pose.position.x = cmd_pos(0);
                pos_cmd.pose.position.y = cmd_pos(1);
                pos_cmd.pose.position.z = cmd_pos(2);
                ROS_INFO("Commanded position is: %f %f %f", cmd_pos(0), cmd_pos(1), cmd_pos(2));
            }
        }
        else{
            pos_cmd.pose.position.x = start_hover_pos(0);
            pos_cmd.pose.position.y = start_hover_pos(1);
            pos_cmd.pose.position.z = start_hover_pos(2);
            ROS_INFO_ONCE("Sending meaningless target...");
        }
        local_pos_pub.publish(pos_cmd);
    }
    else{
        ROS_INFO("Unconnected!");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh;
    TrajCommander cmder(&nh);

    ros::spin();
    return 0;
}