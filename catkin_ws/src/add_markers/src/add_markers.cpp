/*  This node does the following:
 *  - add marker at pickup location
 *  - listen to move_base/status.status (1=in_transit, 3=goal reached)
 *  - delete first marker when goal reached
 *  - listen to move_base/status.status again
 *  - add marker at dropoff location when goal reached
 */

// ROS Libraries
#include "ros/ros.h"
#include "actionlib_msgs/GoalStatusArray.h" // Goal status
#include "visualization_msgs/Marker.h" // Goal marker

#include <iostream>

// Publisher & Subscriber
ros::Subscriber goal_status_subscriber;
ros::Publisher marker_pub;

//uint32_t shape = visualization_msgs::Marker::CUBE;
bool packagePlaced = false;
bool robotMoved = false; // prevents "goal reached" actions without the robot having traveled
bool pickedPackage = false;
bool udacityCorridor = false; // select map

visualization_msgs::Marker marker;
uint32_t shape = visualization_msgs::Marker::CUBE;
//setMarkerParam();

using namespace std;

// Set marker parameters (except for position timestamp and action)
void setMarkerParam()
{
  marker.header.frame_id = "/map"; // COS is world frame
  marker.ns = "basic_shapes"; // marker namespace
  marker.id = 0;
  marker.type = shape;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0; // transparency
  marker.lifetime = ros::Duration();

}


void goal_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{

  if(!msg->status_list.empty()) // Prevent segmentation fault when no goal given
  {
    actionlib_msgs::GoalStatusArray status_array = *msg; // get status message

    if(!packagePlaced)
    {
      marker.header.stamp = ros::Time::now();
      setMarkerParam(); // general parameters

      if(udacityCorridor)
      {
        marker.pose.position.x = -7.0;
        marker.pose.position.y = 1.5;
        marker.pose.position.z = 0.15;
        marker.action = visualization_msgs::Marker::ADD;
      }

      else // custom world
      {
        marker.pose.position.x = 6.0;
        marker.pose.position.y = 1.3;
        marker.pose.position.z = 0.15;
        marker.action = visualization_msgs::Marker::ADD;
      }
      packagePlaced = true;
    }

    // check if robot moved before success status
    if(status_array.status_list[0].status == 1) // status 1 = moving
      robotMoved = true;

    // Delete goal marker when goal reached
    // status 3 = goal reached
    if(status_array.status_list[0].status == 3 && robotMoved && !pickedPackage)
    {
      ROS_INFO("Picking up package.");
      marker.action = visualization_msgs::Marker::DELETE;

      robotMoved = false;
      pickedPackage = true;
    }

    // Reaching dropoff location
    if(status_array.status_list[0].status == 3 && robotMoved && pickedPackage)
    {

      ROS_INFO("Dropping off package.");
      marker.header.stamp = ros::Time::now();
      setMarkerParam();

      marker.pose.position.x = 0.0;
      marker.pose.position.y = 0.5;
      marker.pose.position.z = 0.15;
      marker.action = visualization_msgs::Marker::ADD;
      robotMoved = false;

    }

    marker_pub.publish(marker);

  }

}

int main(int argc, char** argv)
{

  // Initialize Node
  ros::init(argc, argv, "add_markers_node");
  ROS_INFO("Marker node initialized.");

  // NodeHandle object
  ros::NodeHandle n;

  // Refresh rate (Hz)
  ros::Rate r(5);

  // Subscribe to goal status, call goal_callback function when receiving
  goal_status_subscriber = n.subscribe("/move_base/status", 100, goal_callback);

  // marker Publisher
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

}
