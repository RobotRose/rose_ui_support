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
#ifndef SCRIPT_SELECTOR_HPP
#define SCRIPT_SELECTOR_HPP

#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>

#include "rose_datamanager_api/datamanager_api.hpp"

#include "rose_script_manager/execute_scriptAction.h"
#include "rose_script_manager/execute_scriptGoal.h"
#include "rose_script_manager/execute_scriptFeedback.h"
#include "rose_script_manager/execute_scriptResult.h"

#include "rose_ui_script_selector/script_selected.h"
#include "rose_ui_script_selector/scripts.h"

#include "script/script.hpp"
#include "std_msgs/Bool.h"

using namespace std;
using namespace rose_script_manager;

typedef actionlib::SimpleActionClient<rose_script_manager::execute_scriptAction> Client;

class ScriptSelector
{
public:
	ScriptSelector( string name, ros::NodeHandle n );
	~ScriptSelector();
	
private:
	static bool compareScripts(Script lhs, Script rhs);

	void activateScript( Script script );
	void getScripts();
	void publishScripts();

	void CB_scriptSelected( const rose_ui_script_selector::script_selected::ConstPtr& selection );
	void CB_scriptCancelled ( const std_msgs::Bool& cancelled );

	void CB_receiveServerActive();
	void CB_receiveFeedback( const execute_scriptFeedbackConstPtr& feedback );
	void CB_receiveResult( const actionlib::SimpleClientGoalState& state, const execute_scriptResultConstPtr& result );

	Client				client_;
	DatamanagerAPI* 	datamanager_;
	
	ros::NodeHandle 	n_;
	ros::Publisher      scripts_pub_;

	ros::Subscriber 	script_cancelled_sub_;
	ros::Subscriber		script_selected_sub_;

	string 				name_;
	vector<Script> 		scripts_;
};

#endif // SCRIPT_SELECTOR_HPP