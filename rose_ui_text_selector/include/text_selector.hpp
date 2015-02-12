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
#ifndef TEXT_SELECTOR_HPP
#define TEXT_SELECTOR_HPP

#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

#include "bounding_box_factory/bounding_box_factory.hpp"
#include "rose_datamanager_api/datamanager_api.hpp"

#include "rose_parameter_manager/parameterAction.h"
#include "rose_parameter_manager/parameterGoal.h"
#include "rose_parameter_manager/parameterFeedback.h"
#include "rose_parameter_manager/parameterResult.h"

#include "parameter_request_message.hpp"

#include "server_multiple_client/server_multiple_client.hpp"

#include "std_msgs/String.h"

class TextSelector
{
public:
	typedef rose_parameter_manager::parameterAction ParameterAction;
  	typedef ServerMultipleClient<ParameterAction> SMC;

	TextSelector( std::string name, ros::NodeHandle n );
	~TextSelector();
	
private:
	void CB_serverCancel( SMC* smc );
	void CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc );
	void sendResult( bool succes );

	void sendBoundingBoxResult( std::string item_id, std::string bounding_box );
	void sendNameResult( std::string name );
	void sendWaypointNameResult( std::string item_id, std::string name );

	void CB_receiveTextInput( const std_msgs::String text );

	SMC*				smc_;
	DatamanagerAPI* 	datamanager_;
	
	ros::NodeHandle 	n_;
	std::string 		name_;
	
	ros::Publisher      text_pub_;
	ros::Publisher      text_cancel_pub_;
	
	ros::Subscriber 	text_entered_sub_;
};

#endif // TEXT_SELECTOR_HPP