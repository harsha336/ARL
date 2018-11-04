// ROS Includes
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

//C++ Includes
#include <string>
//#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_navigation_goals");

	//Create vector of pairs that will hold the waypoints
	std::vector< std::pair<double, double> > waypoints;

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);
				
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO_STREAM("Waiting for the move_base action server to come up");
	}

	//Open file stream
	double x, y;
	std::ifstream infile("waypointsSSM_pionner1.txt");

	if (!infile){
		ROS_ERROR_STREAM("Could not open file!");
	}

	else{
	
		std::string line;
		while (std::getline(infile, line)){
			std::size_t delim_pos = line.find(" ");
			double x = atof(line.substr(0, delim_pos).c_str());
			double y = atof(line.substr(delim_pos + 1).c_str());
			
			waypoints.push_back(std::make_pair(x, y));
		}
		
		infile.close();
	
		int ref = waypoints.size();
		for(int i = 0; i < x; i++){
			ROS_INFO_STREAM("X: " << waypoints[i].first << " Y: " << waypoints[i].second);
		}
		// Just for testing purposes
		return 0;
		move_base_msgs::MoveBaseGoal goal;
			
		//we'll send a goal to the robot to move 1 meter forward
		goal.target_pose.header.frame_id = "base_link";
		goal.target_pose.header.stamp = ros::Time::now();
				
		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;
				
		ROS_INFO("Sending goal");
		ac.sendGoal(goal);
				
		ac.waitForResult();
				
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved 1 meter forward");
		else
			ROS_INFO("The base failed to move forward 1 meter for some reason");
	}	
	return 0;
}
