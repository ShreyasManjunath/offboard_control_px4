#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <list>
#include <cmath>
#include "offboard_control_px4/GeographicOperations.hpp"
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Geometry> 



mavros_msgs::State current_state;
geometry_msgs::PoseArray computed_trajectory_posearray;
Eigen::Vector3d LLA_HOME = Eigen::Vector3d(49.015752, 8.426293, 115.00);


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
	ROS_INFO("%s", armed ? "" : "DisArmed");
}

void convertPoseArrayToWPformat(mavros_msgs::WaypointPush& wp_push_srv);
void goHome(mavros_msgs::WaypointPush& wp_push_srv);
void takeOff(mavros_msgs::WaypointPush& wp_push_srv);


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_mode_node");
    ros::NodeHandle nh;
    
    mavros_msgs::SetMode mission_mode;
    mission_mode.request.custom_mode = "AUTO.MISSION";
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;
    boost::shared_ptr<geometry_msgs::PoseArray const> computed_trajectory_posearray_ptr;
    computed_trajectory_posearray_ptr = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/computed_trajectory_posearray",nh);
    
    if(computed_trajectory_posearray_ptr != NULL){
      computed_trajectory_posearray = *computed_trajectory_posearray_ptr;
    }

    takeOff(wp_push_srv);

    convertPoseArrayToWPformat(wp_push_srv);

    goHome(wp_push_srv);

    ROS_INFO("Waypoints vector size: %i", wp_push_srv.request.waypoints.size());

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to PX4!");
    // ARM
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

    // Send WPs to Vehicle
    if (wp_client.call(wp_push_srv)) {
        ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
        if (current_state.mode != "AUTO.MISSION") {
            if( set_mode_client.call(mission_mode) &&
                mission_mode.response.mode_sent){
                ROS_INFO("AUTO.MISSION enabled");
            }
        }
    }
    else
        ROS_ERROR("Send waypoints FAILED.");

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void convertPoseArrayToWPformat(mavros_msgs::WaypointPush& wp_push_srv){
    for(auto& pose : computed_trajectory_posearray.poses){
        double x,y,z;
        double roll,pitch,yaw;
        double lat, lon, alt;
        alt = pose.position.z
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d rx = q.toRotationMatrix();
        Eigen::Vector3d euler = rx.eulerAngles(2,1,0);
        yaw = euler[0]; pitch = euler[1]; roll = euler[2];
        Eigen::Vector3d enu_position(pose.position.x, pose.position.y, pose.position.z);
        Eigen::Vector3d position_in_lla = enu2lla(LLA_HOME, enu_position);
        mavros_msgs::Waypoint wp;
        wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
        wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
        wp.is_current     = false;
        wp.autocontinue   = true;
        wp.param4         = yaw; 
        wp.x_lat          = position_in_lla[0];
        wp.y_long         = position_in_lla[1];
        wp.z_alt          = alt;
        wp_push_srv.request.waypoints.push_back(wp);
    }
}


void goHome(mavros_msgs::WaypointPush& wp_push_srv){
    mavros_msgs::Waypoint wp;
    wp.frame          = mavros_msgs::Waypoint::FRAME_MISSION;
    wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);
}

void takeOff(mavros_msgs::WaypointPush& wp_push_srv){
    mavros_msgs::Waypoint wp;
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = 49.015752;
    wp.y_long         = 8.426293;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);
}
