/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/22
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "item_collector_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
  ros::init(argc, argv, "item_collector");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("topic", topic, string("item_collector"));

  // Create a new ScriptInteractionNode object.
  ItemCollector* item_collector = new ItemCollector("item_collector", n);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  delete item_collector;

  return 0;
}