#include "offboard_control_px4/WaypointControl.h"

WaypointControl::WaypointControl(ros::NodeHandle& n):nh(nh),init_local_pose_check(true),waypoint_count(0), obtained_waypoints(false)
{
    ROS_INFO("waypoint_control_node Started..");
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &WaypointControl::stateCallback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &WaypointControl::currentPosecallback, this);
    init_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &WaypointControl::initPoseCallback, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    computed_trajectory_posearray_sub = nh.subscribe("/computed_trajectory_posearray", 1, &WaypointControl::subscribeToWaypointsFromSIP, this);

    ros::Rate rate(20.0);
    
    if(obtained_waypoints){
        
        //getWaypoint();
    }
		

}

void WaypointControl::stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void WaypointControl::currentPosecallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

void WaypointControl::initPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    publishWaypoint();

    ros::Rate rate(20.0);
    rate.sleep();
}

void WaypointControl::getWaypoint() {
    if (ros::param::get("waypoint_control_node/num_waypoint", num_waypoint)) {}
    else {
        ROS_WARN("Didn't find num_waypoint");
    }
    if (ros::param::get("waypoint_control_node/x_pos", x_pos)) {}
    else {
        ROS_WARN("Didn't find x_pos");
    }
    if (ros::param::get("waypoint_control_node/y_pos", y_pos)) {}
    else {
        ROS_WARN("Didn't find y_pos");
    }
    if (ros::param::get("waypoint_control_node/z_pos", z_pos)) {}
    else {
        ROS_WARN("Didn't find z_pos");
    }

    if (x_pos.size() != num_waypoint) {
        ROS_WARN("Wrong x_pos values.");
    }
    if (y_pos.size() != num_waypoint) {
        ROS_WARN("Wrong y_pos values.");
    }
    if (z_pos.size() != num_waypoint) {
        ROS_WARN("Wrong z_pos values.");
    }
    obtained_waypoints = true;
}

void WaypointControl::subscribeToWaypointsFromSIP(const geometry_msgs::PoseArrayConstPtr& posearray){
    if(obtained_waypoints){
        geometry_msgs::PoseArray tempPoseArray = *posearray;
        num_waypoint = tempPoseArray.poses.size();
        for(geometry_msgs::Pose& pose : tempPoseArray.poses){
            geometry_msgs::PoseStamped tempPose;
            tempPose.pose = pose;
            computed_trajectory_posearray.push_back(tempPose);
        }
        init_local_pose_check = false;
        obtained_waypoints = true;
    }
    
}

void WaypointControl::publishWaypoint() {
    if (!init_local_pose_check) {
        local_pos_pub.publish(computed_trajectory_posearray[waypoint_count]);
        
        if (abs(current_pose.pose.position.x - computed_trajectory_posearray[waypoint_count].pose.position.x) < 0.5 && 
            abs(current_pose.pose.position.y - computed_trajectory_posearray[waypoint_count].pose.position.y) < 0.5 &&
            abs(current_pose.pose.position.z - computed_trajectory_posearray[waypoint_count].pose.position.z) < 0.5) {

            waypoint_count += 1;

            if (waypoint_count >= num_waypoint) {
                waypoint_count--;
            }

        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_control_node");
    ros::NodeHandle nh;
    ROS_INFO("waypoint_control_node Initialized..");
    ros::Duration(11).sleep();
    WaypointControl wpc(nh);

    ros::spin();


    return 0;
}
