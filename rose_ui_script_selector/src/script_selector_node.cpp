/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/20
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "rose_ui_script_selector/script_selector_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
  ros::init(argc, argv, "script_selector");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  std::string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("topic", topic, std::string("script_selector"));

  // Create a new ScriptInteractionNode object.
  ScriptSelector* scripts_table = new ScriptSelector("script_selector", n);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (n.ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  delete scripts_table;

  return 0;
}