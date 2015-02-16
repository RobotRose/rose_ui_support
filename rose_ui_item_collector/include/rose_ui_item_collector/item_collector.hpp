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
#ifndef ITEM_COLLECTOR_HPP
#define ITEM_COLLECTOR_HPP

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "rose_ui_item_collector/get_itemsAction.h"

#include "rose_ui_item_selector/itemsAction.h"
#include "rose_ui_item_selector/itemsGoal.h"
#include "rose_ui_item_selector/itemsFeedback.h"
#include "rose_ui_item_selector/itemsResult.h"

#include "server_multiple_client/server_multiple_client.hpp"
#include "roscomm/stringlist.h"

typedef ServerMultipleClient<rose_ui_item_collector::get_itemsAction> SMC;

using namespace std;
using namespace rose_ui_item_selector;

class ItemCollector
{
public:
	ItemCollector( string name, ros::NodeHandle n );
	~ItemCollector();

private:
	// Client functions
	void CB_custom_success( const actionlib::SimpleClientGoalState& state, const itemsResultConstPtr& result );
	void CB_custom_fail( const actionlib::SimpleClientGoalState& state, const itemsResultConstPtr& result );
	void CB_custom_active();
	void CB_custom_feedback( const itemsFeedbackConstPtr& feedback );
	void sendGoalToClients( vector<string> names, vector< vector<string> > types );

	// Server functions
	void sendResult();
	void CB_serverCancel( SMC* smc );
	void CB_serverWork( const rose_ui_item_collector::get_itemsGoalConstPtr &goal, SMC* smc );

	rose_ui_item_collector::get_itemsResult 	result_;
	rose_ui_item_selector::itemsResultConstPtr 	received_result_;

	ros::NodeHandle 	n_;
	string				name_;
	SMC					smc_;
};

#endif //ITEM_COLLECTOR_HPP