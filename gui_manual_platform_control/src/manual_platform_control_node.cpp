/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*  Author: Mathijs de Langen
*  Date  : 2014/04/15
*     - File created.
*
* Description:
*  description
* 
***********************************************************************************/
#include "manual_platform_control_node.hpp"

int main( int argc, char **argv )
{

// Set up ROS.
  ros::init(argc, argv, "manual_platform_control");
  ros::NodeHandle n;

  // Declare variables that can be modified by launch file or command line.
  int rate;
  std::string topic;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("rate", rate, int(10));
  private_node_handle_.param("topic", topic, std::string("manual_platform_control"));

  // Create a new ScriptInteractionNode object.
  ManualPlatformControl manual_platform_control("manual_platform_control", n);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Monitor the low-level platform controller alarm state
  SharedVariable<bool> sh_platform_controller_alarm_(SharedVariable<bool>("platform_controller_alarm"));
  sh_platform_controller_alarm_.connect(ros::Duration(1.0));

  // Main loop.
  while (n.ok())
  {
    if(sh_platform_controller_alarm_)
      manual_platform_control.stopMovement();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}