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
#include "rose_ui_item_collector/item_collector.hpp"

ItemCollector::ItemCollector( string name, ros::NodeHandle n ) 
	: name_ ( name )
	, n_ ( n )
	, smc_ (n_, name_, boost::bind(&ItemCollector::CB_serverWork, this, _1, _2),
                       boost::bind(&ItemCollector::CB_serverCancel, this, _1))
{
	// Add client
	smc_.addClient<rose_ui_item_selector::itemsAction>("item_selector", boost::bind(&ItemCollector::CB_custom_success, this, _1, _2),
																	boost::bind(&ItemCollector::CB_custom_fail, this, _1, _2),
															  		boost::bind(&ItemCollector::CB_custom_active, this),
															  		boost::bind(&ItemCollector::CB_custom_feedback, this, _1));	
	smc_.startServer();
}

ItemCollector::~ItemCollector()
{

}

// -------------- Client callback functions
void ItemCollector::CB_custom_success( const actionlib::SimpleClientGoalState& state, const itemsResultConstPtr& result )
{
	ROS_INFO("ItemCollector::CB_custom_success");
	received_result_ = result;
	sendResult();
}

void ItemCollector::CB_custom_fail( const actionlib::SimpleClientGoalState& state, const itemsResultConstPtr& result )
{
	ROS_INFO("ItemCollector::CB_custom_fail");
}

void ItemCollector::CB_custom_active()
{
	ROS_INFO("ItemCollector::CB_custom_active");
}

void ItemCollector::CB_custom_feedback( const itemsFeedbackConstPtr& feedback )
{
	ROS_INFO("ItemCollector::CB_custom_feedback");
}

void ItemCollector::sendGoalToClients( vector<string> names, vector< vector<string> > types )
{
	ROS_INFO("ItemCollector::itemSelectorSendGoal");
	itemsGoal goal;
	goal.items 	= names;
	for ( int i = 0 ; i < types.size() ; i++ )
	{
		roscomm::stringlist types_for_name;
		types_for_name.values = types.at(i);
		goal.item_types.push_back(types_for_name);
	}

	smc_.sendGoal<rose_ui_item_selector::itemsAction>(goal);
}

// -------------- Server callback functions
void ItemCollector::sendResult()
{
	result_.item_ids.clear();
	roscomm::stringlist stringlist;
	vector<string> item_ids;

	vector<roscomm::stringlist> current_selection = received_result_->current_selection;

	for ( auto parameter = current_selection.begin() ; parameter != current_selection.end() ; parameter++ )
	{
		vector<string> item_ids = (*parameter).values;
		stringlist.values.clear();
		
		for ( auto item_id = item_ids.begin() ; item_id != item_ids.end() ; item_id ++ )
			stringlist.values.push_back(*item_id);

		result_.item_ids.push_back(stringlist);
	}

	smc_.sendServerResult( true, result_ );
}

void ItemCollector::CB_serverCancel( SMC* smc )
{
	ROS_INFO("ItemCollector::CB_serverCancel()");
}

void ItemCollector::CB_serverWork( const rose_ui_item_collector::get_itemsGoalConstPtr &goal, SMC* smc )
{
	ROS_INFO("ItemCollector::CB_receiveGoal");

	vector<string> names = goal->names;
	vector< vector<string> > types;

	for ( int i = 0 ; i < goal->types.size() ; i++ )
	{
		vector<string> types_for_name = goal->types.at(i).values;
		types.push_back( types_for_name );
	}

	sendGoalToClients( names, types );
}
