/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <fstream>
#include <vector>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <cmath>

#define _USE_MATH_DEFINES

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_arming_node");
    ros::NodeHandle nh;

    // default values for linear speed and angular yaw rate(angular speed).
    float g_speed;
    float g_maxAngularSpeed;

    if(!ros::param::get("/Inspection_Planner/rotorcraft/maxSpeed", g_speed)){
        ROS_ERROR("MISSING PARAMETER: Max Linear Speed - MPC_XY_VEL_MAX");
        g_speed = 12.0;
        ROS_INFO("Setting Default values : MPC_XY_VEL_MAX: %d", g_speed);
    }
        
    if(!ros::param::get("/Inspection_Planner/rotorcraft/maxAngularSpeed", g_maxAngularSpeed)){
        ROS_ERROR("MISSING PARAMETER: Max Angular Speed - MC_YAWRATE_MAX");
        g_maxAngularSpeed = 3.490; // In rad
        ROS_INFO("Setting Default values : MC_YAWRATE_MAX: %d", g_maxAngularSpeed);
    }

    g_maxAngularSpeed = g_maxAngularSpeed * (180.0/M_PI);
        
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    auto set_param = [](std::string name, auto value) {
        mavros_msgs::ParamSet p;
        mavros_msgs::ParamValue v;
        p.request.param_id = name;
        v.real             = value;
        p.request.value    = v;
        if (ros::service::call("mavros/param/set", p)) {
            ROS_INFO_STREAM("Set Parameter " << name << " to " << value);
        }
    };
    set_param("MPC_XY_VEL_MAX", g_speed);
    set_param("MC_YAWRATE_MAX", g_maxAngularSpeed);


    geometry_msgs::PoseStamped p1;
    p1.pose.position.x = 0;
    p1.pose.position.y = 0;
    p1.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(p1);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
   
    while(ros::ok()){
	if(current_state.mode == "OFFBOARD" && current_state.armed == true){
		ROS_INFO("Spinning now!");
		break;
	}
        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        local_pos_pub.publish(p1);
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    
    return 0;
}

