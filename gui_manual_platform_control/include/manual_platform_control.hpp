/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/04/15
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef MANUAL_PLATFORM_CONTROL_HPP
#define MANUAL_PLATFORM_CONTROL_HPP

#include <ros/ros.h>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "shared_variables/shared_variable.hpp"

using namespace shared_variables;

class ManualPlatformControl
{
public:
	ManualPlatformControl( std::string name, ros::NodeHandle n );
	~ManualPlatformControl();

	void stopMovement();
	
private:
	#define MAX_ANGLE 			0.75
	#define MAX_MOVEMENT_SPEED 	0.60
	#define MAX_STRAFE_SPEED 	0.15

	void CB_increaseForwardSpeed  ( const std_msgs::Empty& );
	void CB_decreaseForwardSpeed  ( const std_msgs::Empty& );
	void CB_increaseStafeLeftSpeed( const std_msgs::Empty& );
	void CB_decreaseStafeLeftSpeed( const std_msgs::Empty& );
	void CB_increateLeftRotation  ( const std_msgs::Empty& );
	void CB_decreaseLeftRotation  ( const std_msgs::Empty& );
	void CB_stopMovement  		  ( const std_msgs::Empty& );

	void publishMovement();

	ros::NodeHandle 		n_;
	std::string 			name_;

    double 					current_speed_forward_;
    double 					current_speed_left_;
    double 					current_speed_angle_;

	ros::Publisher      	wheel_publisher_;

   	ros::Subscriber 		move_forwards_sub_;
    ros::Subscriber 		move_backwards_sub_;
    ros::Subscriber 		strafe_left_sub_;
    ros::Subscriber 		stafe_right_sub_;
    ros::Subscriber 		rotate_left_sub_;
    ros::Subscriber 		rotate_right_sub_;
    ros::Subscriber 		stop_sub_;
};

#endif // MANUAL_PLATFORM_CONTROL_HPP