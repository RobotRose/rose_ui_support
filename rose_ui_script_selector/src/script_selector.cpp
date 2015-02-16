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
#include "rose_ui_script_selector/script_selector.hpp"

ScriptSelector::ScriptSelector( string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
	, client_ ( "script_manager", true )
{
	datamanager_ = new DatamanagerAPI();

    // Publishers
    scripts_pub_       	 = n_.advertise<rose_ui_script_selector::scripts>( "/script_selector/scripts", 1, true );
    getScripts();
    publishScripts();

    // Subscribers
    script_cancelled_sub_ 	= n_.subscribe( "/script_selector/script_cancelled", 1, &ScriptSelector::CB_scriptCancelled, this );
    script_selected_sub_ 	= n_.subscribe( "/script_selector/script_selected", 1, &ScriptSelector::CB_scriptSelected, this );

    client_.waitForServer();
}

ScriptSelector::~ScriptSelector()
{

}

void ScriptSelector::getScripts()
{
	while ( scripts_.empty() )
		scripts_ = datamanager_->getAll<Script>();

	for ( auto s = scripts_.begin() ; s != scripts_.end() ; s++ )
	{
		ROS_INFO("Script id: %s", (*s).get_id().c_str());
		ROS_INFO("Script name: %s", (*s).get_name().c_str());
		ROS_INFO("Script file: %s", (*s).get_filename().c_str());
	}

	std::sort(scripts_.begin(), scripts_.end(), &ScriptSelector::compareScripts);
}

bool ScriptSelector::compareScripts(Script lhs, Script rhs)
{
	return lhs.get_name() < rhs.get_name();
}

void ScriptSelector::publishScripts()
{
	rose_ui_script_selector::scripts scripts;
    for ( auto script = scripts_.begin() ; script != scripts_.end() ; script++ )
		scripts.scripts.push_back((*script).get_name());
	
	scripts_pub_.publish( scripts );
}

void ScriptSelector::activateScript( Script script )
{
	ROS_INFO("ScriptSelector::activateScript");
	execute_scriptGoal goal;
	goal.script_id = script.get_filename();

	ROS_INFO("acivating script %s", script.get_name().c_str());

	client_.sendGoal(goal, 
		boost::bind(&ScriptSelector::CB_receiveResult, this, _1, _2), 
		boost::bind(&ScriptSelector::CB_receiveServerActive, this), 
		boost::bind(&ScriptSelector::CB_receiveFeedback, this, _1)
	);
}
void ScriptSelector::CB_scriptCancelled ( const std_msgs::Bool& cancelled )
{
	ROS_INFO("ScriptSelector::CB_scriptCancelled");
	client_.cancelGoal();
}

void ScriptSelector::CB_scriptSelected( const rose_ui_script_selector::script_selected::ConstPtr& selection )
{
	ROS_INFO("ScriptSelector::CB_scriptSelected");

    int row_nr   = selection->row_nr;

    activateScript( scripts_.at(row_nr) );
}

void ScriptSelector::CB_receiveServerActive()
{
	ROS_INFO("ScriptSelector::CB_receiveServerActive");
}

void ScriptSelector::CB_receiveFeedback( const execute_scriptFeedbackConstPtr& feedback )
{
	ROS_INFO("ScriptSelector::CB_receiveFeedback");
}

void ScriptSelector::CB_receiveResult( const actionlib::SimpleClientGoalState& state,
	const execute_scriptResultConstPtr& result )
{
	ROS_INFO("ScriptSelector::CB_receiveResult");
}