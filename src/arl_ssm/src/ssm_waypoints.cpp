// ROS Includes
#include <ros/ros.h>
#include <ros/package.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//C++ Includes
#include <fstream>
#include <vector>
#include <utility>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "ssm_waypoints");

	//Create vector of pairs that will hold the waypoints
	std::vector< std::pair<double, double> > waypoints;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
				
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO_STREAM("Waiting for the move_base action server to come up");
	}

	//Open file stream with the correct path obtained through call to getPath from roslib.
	std::string file_path = ros::package::getPath("arl_ssm");
	file_path += "/src/waypointsSSM_pioneer2.txt";
	double x, y;
	ROS_INFO_STREAM("This is default path to file: " << file_path);
	std::ifstream infile(file_path.c_str());

	if (!infile){
		ROS_ERROR_STREAM("Could not open file!");
	}

	//If file is opened succesfully begin reading pairs of x and y numbers
	//for our current code we assume numbers are two in a line sepereted by a space.
	//push back pairs into vector "waypoints"
	else{
		ROS_INFO_STREAM("File opened successfully!");
		int count = 0;
		while (infile >> x >> y){
			waypoints.push_back(std::make_pair(x, y));
			ROS_INFO_STREAM("Inserted waypoint X: " << waypoints[count].first << " Y: " << waypoints[count].second);
			count++;
		}
		
		infile.close();
		ROS_INFO_STREAM("Read Complete.");
	
		move_base_msgs::MoveBaseGoal goal;
		
		//Next, iterate thorugh the vector sending the individual points to move_base.
		int number_of_waypoints = waypoints.size();
		for (int i = 0; i < number_of_waypoints; i++){
			//we'll send a goal to the robot to the waypoints
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
				
			goal.target_pose.pose.position.x = waypoints[i].first;
			goal.target_pose.pose.position.y = waypoints[i].second;
			goal.target_pose.pose.orientation.w = 1.0;
				
			ROS_INFO("Sending goal");
			ac.sendGoal(goal);
				
			ac.waitForResult();
				
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO_STREAM("Hooray, the base moved to waypoint number: " << i);
			else
				ROS_INFO_STREAM("The base failed to move to waypoint for some reason");
		}
	}	
	return 0;
}
