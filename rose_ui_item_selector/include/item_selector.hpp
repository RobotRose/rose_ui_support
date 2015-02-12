/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/20
* 		- File created.
*
* Description:
*	Thread that communcates with the script manager node.
* 
***********************************************************************************/
#ifndef ITEM_SELECTOR_HPP
#define ITEM_SELECTOR_HPP

#include <ros/ros.h>

#include "rose_datamanager_api/datamanager_api.hpp"

#include "gui_item_selector/item_selected.h"
#include "gui_item_selector/item_selection.h"
#include "gui_item_selector/items.h"

#include "gui_item_selector/itemsAction.h"
#include "gui_item_selector/itemsGoal.h"
#include "gui_item_selector/itemsFeedback.h"
#include "gui_item_selector/itemsResult.h"

#include "server_multiple_client/server_multiple_client.hpp"

#include "selectable_item.hpp"
#include "selectable_item_table.hpp"

#include "std_msgs/String.h"

using namespace std;

typedef ServerMultipleClient<gui_item_selector::itemsAction> SMC;

class ItemSelector
{

public:
	ItemSelector(string name, ros::NodeHandle n);
	~ItemSelector();

    int  getLastTableSelection();

private:
    void CB_serverWork( const gui_item_selector::itemsGoalConstPtr &goal, SMC* smc );
    void CB_serverCancel( SMC* smc );

    void sendFeedback();
    void sendResult();

    void CB_itemActivated( const gui_item_selector::item_selected::ConstPtr& selection );
    void CB_selectionFinished( const gui_item_selector::item_selection::ConstPtr& message );
    void CB_removeItem( const gui_item_selector::item_selected::ConstPtr& selection );
    void CB_removeSelection( const gui_item_selector::item_selection::ConstPtr& message );

    void publishTables();
    void publishEmptyTables();
    
    void checkForAndStoreNewItems( SelectableItemTable* itemtable, vector<int> item_ids );
    void removeItemFromDatabase( const std::string id );
    void removeItems( SelectableItemTable* itemlist, vector<int> items );
    
    template<class T>
    static bool compare(T lhs, T rhs) { return lhs.get_name() < rhs.get_name(); };

    ros::NodeHandle     n_;
    string              name_;
    DatamanagerAPI*     datamanager_;
    SMC                 smc_;

    ros::Publisher      items_pub_;
    ros::Subscriber     item_selected_sub_;
    ros::Subscriber     item_selection_finished_sub_;
    ros::Subscriber     remove_item_sub_;
    ros::Subscriber     remove_items_sub_;

    ros::Publisher      redraw_waypoints_pub_;

    SelectableItemTable*    item_table_list_;

    gui_item_selector::itemsFeedback    feedback_;
    gui_item_selector::itemsResult      result_;

    boost::mutex::scoped_lock   done_;

    int last_selected_item_;
};

#endif //ITEM_SELECTOR_HPP
