#ifndef WAYPOINTCONTROL_H_
#define WAYPOINTCONTROL_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <geometry_msgs/PoseArray.h>


class WaypointControl {  
private:
	ros::NodeHandle nh;
	
	ros::Subscriber state_sub;
	ros::Subscriber local_pos_sub;
	ros::Subscriber init_local_pos_sub;
	ros::Publisher local_pos_pub;
	ros::ServiceClient arming_client;
	ros::ServiceClient set_mode_client;
	ros::Subscriber computed_trajectory_posearray_sub;

	mavros_msgs::State current_state;
	geometry_msgs::PoseStamped current_pose;
	std::vector<geometry_msgs::PoseStamped> waypoint_pose;
	bool init_local_pose_check;
	int waypoint_count;
	bool obtained_waypoints;
	bool flight_complete;
	
	int num_waypoint;
	std::vector<double> x_pos;
	std::vector<double> y_pos;
	std::vector<double> z_pos;
	std::vector<geometry_msgs::PoseStamped> computed_trajectory_posearray;

	ros::WallTime start_time;
	ros::WallTime end_time;
	double duration_of_flight;

public:
	WaypointControl(ros::NodeHandle& n);
	void publishWaypoint();
	void getWaypoint();
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	void currentPosecallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void initPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void subscribeToWaypointsFromSIP(const geometry_msgs::PoseArrayConstPtr& posearray);
	
};

#endif
