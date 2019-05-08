#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool projectSubmission = true; // project only requires 1 pickup


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Location 1:
  goal.target_pose.pose.position.x = 6.0;
  goal.target_pose.pose.position.y = 1.3;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending pickup location 1...");
  ac.sendGoal(goal);
  ac.waitForResult();

  // Check if the robot reached goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Pickup location 1 reached.");
  else
    ROS_INFO("Could not reach pickup location.");

  sleep(5);

  if(!projectSubmission) // Additional goal location
  {

      // Location 2:
      goal.target_pose.pose.position.x = -1.1;
      goal.target_pose.pose.position.y = -5.6;
      goal.target_pose.pose.orientation.w = 1.0;
      ROS_INFO("Sending pickup location 2...");
      ac.sendGoal(goal);
      ac.waitForResult();

      // Check if the robot reached goal 2
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Pickup location 2 reached.");
      else
        ROS_INFO("Could not reach pickup location.");

    }

    // Back home:
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Going home...");
    ac.sendGoal(goal);
    ac.waitForResult();

    // Check if the robot reached goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("I'm home!");
    else
      ROS_INFO("Could not get home.");

    sleep(5);

  ros::shutdown();
  return 0;
}
