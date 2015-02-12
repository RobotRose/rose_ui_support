/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/04/15
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#include "manual_platform_control.hpp"

ManualPlatformControl::ManualPlatformControl( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
    , current_speed_forward_ ( 0 )
    , current_speed_left_ ( 0 )
    , current_speed_angle_ ( 0 )
{      
    // Subscribers
    move_forwards_sub_ 	    = n_.subscribe( "/manual_platform_control/move_forward",  1, &ManualPlatformControl::CB_increaseForwardSpeed,   this );
    move_backwards_sub_     = n_.subscribe( "/manual_platform_control/move_backward", 1, &ManualPlatformControl::CB_decreaseForwardSpeed,   this );
    strafe_left_sub_        = n_.subscribe( "/manual_platform_control/strafe_left",   1, &ManualPlatformControl::CB_increaseStafeLeftSpeed, this );
    stafe_right_sub_        = n_.subscribe( "/manual_platform_control/stafe_right",   1, &ManualPlatformControl::CB_decreaseStafeLeftSpeed, this );
    rotate_left_sub_        = n_.subscribe( "/manual_platform_control/rotate_left",   1, &ManualPlatformControl::CB_increateLeftRotation,   this );
    rotate_right_sub_       = n_.subscribe( "/manual_platform_control/rotate_right",  1, &ManualPlatformControl::CB_decreaseLeftRotation,   this );
    stop_sub_               = n_.subscribe( "/manual_platform_control/stop",          1, &ManualPlatformControl::CB_stopMovement,   this );

    // Publishers
    wheel_publisher_        = n_.advertise<geometry_msgs::Twist>("/manual_cmd_vel", 1);
}

ManualPlatformControl::~ManualPlatformControl()
{

}

void ManualPlatformControl::stopMovement()
{
    current_speed_forward_  = 0.0;
    current_speed_left_     = 0.0;
    current_speed_angle_    = 0.0;
    publishMovement();
}

void ManualPlatformControl::CB_increaseForwardSpeed( const std_msgs::Empty& )
{
    current_speed_forward_  = std::min(current_speed_forward_+0.015, MAX_MOVEMENT_SPEED);
    publishMovement();
}

void ManualPlatformControl::CB_decreaseForwardSpeed( const std_msgs::Empty& )
{
    current_speed_forward_  = std::max(current_speed_forward_-0.015, -MAX_MOVEMENT_SPEED);
    publishMovement();
}

void ManualPlatformControl::CB_increaseStafeLeftSpeed( const std_msgs::Empty& )
{
    current_speed_left_     = std::min(current_speed_left_+0.015, MAX_STRAFE_SPEED);
    publishMovement();
}

void ManualPlatformControl::CB_decreaseStafeLeftSpeed( const std_msgs::Empty& )
{
    current_speed_left_     = std::max(current_speed_left_-0.015, -MAX_STRAFE_SPEED);
    publishMovement();
}

void ManualPlatformControl::CB_increateLeftRotation  ( const std_msgs::Empty& )
{
    current_speed_angle_    = std::min(current_speed_angle_+0.03, MAX_ANGLE);
    publishMovement();
}

void ManualPlatformControl::CB_decreaseLeftRotation  ( const std_msgs::Empty& )
{
    current_speed_angle_    = std::max(current_speed_angle_-0.03, -MAX_ANGLE);
    publishMovement();
}

void ManualPlatformControl::CB_stopMovement  ( const std_msgs::Empty& )
{
    stopMovement();
}

void ManualPlatformControl::publishMovement()
{
    geometry_msgs::Twist command_vel;

    command_vel.linear.x    = current_speed_forward_;
    command_vel.linear.y    = current_speed_left_;
    command_vel.angular.z   = current_speed_angle_;

    wheel_publisher_.publish(command_vel);
    ROS_INFO("Movement pulished: %f, %f, %f", current_speed_forward_, current_speed_left_, current_speed_angle_);
}