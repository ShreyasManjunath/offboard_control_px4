#include "offboard_control_px4/WaypointControl.h"

WaypointControl::WaypointControl(ros::NodeHandle& n):nh(nh),init_local_pose_check(true),waypoint_count(0), obtained_waypoints(false)
{
    ROS_INFO("waypoint_control_node Started..");
    flight_complete = false;
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &WaypointControl::stateCallback, this);
    local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &WaypointControl::currentPosecallback, this);
    init_local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &WaypointControl::initPoseCallback, this);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    computed_trajectory_posearray_sub = nh.subscribe("/computed_trajectory_posearray", 1, &WaypointControl::subscribeToWaypointsFromSIP, this);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");
    landing_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/land");
    
    if(!obtained_waypoints){
        
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

    if(!flight_complete)
        publishWaypoint();

    ros::Rate rate(20.0);
    rate.sleep();
    if(flight_complete){
        bool flag = false;
        flag = landVehicle();
        if(flag){
            if(disarmVehicle())
                ROS_INFO("Mission complete.");
            
            ros::shutdown();
        }
            
        
    }
        
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
    if(!obtained_waypoints){
        geometry_msgs::PoseArray tempPoseArray = *posearray;
        num_waypoint = tempPoseArray.poses.size();
        ROS_INFO("No of Waypoints: [%i]", num_waypoint);
        for(geometry_msgs::Pose& pose : tempPoseArray.poses){
            geometry_msgs::PoseStamped tempPose;
            tempPose.pose = pose;
            computed_trajectory_posearray.push_back(tempPose);
        }
        ROS_INFO("Waypoints recvd.");
        init_local_pose_check = false;
        obtained_waypoints = true;
        start_time = ros::WallTime::now();
    }
    
}

void WaypointControl::publishWaypoint() {
    if (!init_local_pose_check) {
        local_pos_pub.publish(computed_trajectory_posearray[waypoint_count]);
        
        if (abs(current_pose.pose.position.x - computed_trajectory_posearray[waypoint_count].pose.position.x) < 0.2 && 
            abs(current_pose.pose.position.y - computed_trajectory_posearray[waypoint_count].pose.position.y) < 0.2 &&
            abs(current_pose.pose.position.z - computed_trajectory_posearray[waypoint_count].pose.position.z) < 0.2) {

            waypoint_count += 1;
            // Waits at the waypoint for specified time.
            waitAtWaypoint(0.5);
            ROS_INFO("Next Waypoint ID: [%i]", waypoint_count);
            if (waypoint_count >= num_waypoint) {
                waypoint_count--;
                flight_complete = true;
                end_time = ros::WallTime::now();
                //duration_of_flight = (end_time - start_time).toNSec() * 1e-6;
                //ROS_INFO("Duration of Flight: %d seconds.", duration_of_flight);
                return;
            }

        }
    }
}

bool WaypointControl::landVehicle(){

    mavros_msgs::CommandTOL land_cmd;
    ros::Rate rate(20.0);
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;   
    while (!(landing_client.call(land_cmd) &&
            land_cmd.response.success)){
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Vehicle Landed.");
    return land_cmd.response.success;
}

bool WaypointControl::disarmVehicle(){
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    while(current_state.armed){
        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
            ROS_INFO("Vehicle disarmed.");
        }
    }
    return arm_cmd.response.success;
}

void WaypointControl::waitAtWaypoint(double waitTime){
    ros::Rate rate(10.0); // 10 hz or 0.1 sec
    int iter = waitTime/0.1; // Number of iterations fulfilling the waitTime.
    for(int i = 0; i < iter && ros::ok(); i++){
        local_pos_pub.publish(computed_trajectory_posearray[waypoint_count]);
        rate.sleep();
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
