/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/20
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "item_selector.hpp"

ItemSelector::ItemSelector(string name, ros::NodeHandle n)
    : n_ ( n )
    , name_ ( name )
    , last_selected_item_ ( 0 )
    , smc_ (n_, name_, boost::bind(&ItemSelector::CB_serverWork, this, _1, _2),
                       boost::bind(&ItemSelector::CB_serverCancel, this, _1))
{  
    datamanager_ = new DatamanagerAPI();

    // Publishers
    items_pub_       = n_.advertise<gui_item_selector::items>(name + "/items", 1, true);

    // Subscribers
    remove_items_sub_            = n_.subscribe(name + "/remove_selection", 1, &ItemSelector::CB_removeSelection, this);
    remove_item_sub_             = n_.subscribe(name + "/remove_selected", 1, &ItemSelector::CB_removeItem, this);
    item_selected_sub_           = n_.subscribe(name + "/item_selected", 1, &ItemSelector::CB_itemActivated, this);
    item_selection_finished_sub_ = n_.subscribe(name + "/item_selection_finished", 1, &ItemSelector::CB_selectionFinished, this);

    redraw_waypoints_pub_ = n_.advertise<std_msgs::Empty>("/map_display/redraw_waypoints", 1, true);

    publishEmptyTables();

    smc_.startServer();
}

ItemSelector::~ItemSelector()
{

}

void ItemSelector::CB_serverWork( const gui_item_selector::itemsGoalConstPtr &goal, SMC* smc )
{
    ROS_INFO("ItemSelector::CB_serverWork()");

    item_table_list_ = new SelectableItemTable();

    ROS_INFO("Received %i tables", (int) goal->items.size());

    if ( goal->items.size() == 0 )
    {
        SelectableItem* parameter = new SelectableItem("no parameters needed", "none");
        item_table_list_->addItem(parameter);
        item_table_list_->set_selection_mode( eNoSelection );
    }
    else 
    {
    // Fill in the new tables
        for ( int i = 0 ; i < goal->items.size() ; i++ )
        {
            // Fill in name, and get all types for
            SelectableItem* parameter = new SelectableItem(goal->items.at(i), "none");
    
            for ( int j = 0 ; j < goal->item_types.at(i).values.size() ; j++ )
                parameter->add_type(goal->item_types.at(i).values.at(j));
    
            // Get possible choices for item (name and id)
            SelectableItemTable* choice_table = new SelectableItemTable();
    
            // Add cross-reference
            parameter->set_reference(choice_table);
            choice_table->set_reference(parameter);
            choice_table->set_selection_mode( eMultiSelection );
    
            // Get items from the datamanager
            for ( int j = 0 ; j < parameter->get_types().size() ; j++ )
            {
                if ( parameter->get_types().at(j) == "items")
                {
                    std::vector<Item> items = datamanager_->getAll<Item>();

                    // Create standard 'empty' option for item
                    Item empty_item(" New item");
                    items.push_back(empty_item);

                    ROS_INFO_NAMED(ROS_NAME, "Found the following items:");
                    for ( auto& item : items )
                        ROS_INFO_NAMED(ROS_NAME, "id: %s, name: %s", item.get_id().c_str(), item.get_name().c_str());
                    // empty_item = datamanager_->store<Item>(empty_item);

                    choice_table->addItems(items);
                }
                if ( parameter->get_types().at(j) == "waypoints")
                {
                    std::vector<Waypoint> waypoints = datamanager_->getAll<Waypoint>();

                    // Create standard 'empty' option for item
                    Waypoint empty_waypoint(" New waypoint");
                    waypoints.push_back(empty_waypoint);

                    // empty_waypoint = datamanager_->store<Waypoint>(empty_waypoint);
                    //! @todo MdL: We do not want to show waypoint 10 (reserved waypoint).
                    ROS_DEBUG_NAMED(ROS_NAME, "I have found %d waypoints", (int)waypoints.size());
                    
                    auto wp = std::find_if(waypoints.begin(), waypoints.end(), [](Waypoint& wp){return (wp.get_id() == "waypoint10");});
                    waypoints.erase(wp);

                    ROS_DEBUG_NAMED(ROS_NAME, "%d waypoints after filtering", (int)waypoints.size());

                    choice_table->addItems(waypoints);
                }
                if ( parameter->get_types().at(j) == "persons")
                {
                    std::vector<Person> persons = datamanager_->getAll<Person>();

                    // Create standard 'empty' option for item
                    Person empty_person(" New person");
                    persons.push_back(empty_person);

                    // empty_person = datamanager_->store<Person>(empty_person);

                    choice_table->addItems(persons);
                }
            }
            // Nice sorting :)
            choice_table->sortItems();
           
            item_table_list_->addItem(parameter);
        }

        // First item is selected
        item_table_list_->itemSelected(0);
    }

    ROS_INFO("size of first table: %i", (int)item_table_list_->get_itemlist().size());

    publishTables();
}

void ItemSelector::CB_serverCancel( SMC* smc )
{
    ROS_INFO("ItemSelector::CB_serverCancel");
    publishEmptyTables();
}

void ItemSelector::publishTables()
{
    ROS_INFO("ItemSelector::publishTables()");

    gui_item_selector::items visible_tables;

    // To fill the message currectly
    roscomm::stringlist table;
    roscomm::intlist current_selection;

    // Get first table
    // Items
    table.values = item_table_list_->toString();
    visible_tables.tables.push_back(table);
    ROS_INFO("First table names filled");

    // Selection mode
    visible_tables.selection_mode.push_back( item_table_list_->get_selection_mode() );
    ROS_INFO("First table selection filled");

    // Current selection
    int selected_item = -1;
    if ( item_table_list_->getSelectedItemPositions().size() > 0 )
    {
        selected_item = item_table_list_->getSelectedItemPositions().front();
        current_selection.values.push_back(selected_item);
    }
    visible_tables.current_selection.push_back(current_selection);

    ROS_INFO("First table filled");

    if ( selected_item >= 0  && item_table_list_->getSelectableItem(selected_item)->get_reference() != NULL)
    {
        ROS_INFO("Table has reference");
        // Get second table
        SelectableItemTable* itemlist = item_table_list_->getSelectableItem(selected_item)->get_reference();
        
        // Items
        table.values = itemlist->toString();
        visible_tables.tables.push_back(table);

        // Selection mode
        visible_tables.selection_mode.push_back(itemlist->get_selection_mode() );

        // Current selection
        current_selection.values = itemlist->getSelectedItemPositions();
        visible_tables.current_selection.push_back(current_selection);
    }
    else
    {
        ROS_INFO("No reference");
        roscomm::stringlist empty_table;
        roscomm::intlist empty_selection;

        // Items
        visible_tables.tables.push_back(empty_table);

        // Current selection
        visible_tables.current_selection.push_back(empty_selection);

        // Selection mode
        visible_tables.selection_mode.push_back(0);
    }

    visible_tables.last_selected_item = last_selected_item_;

    ROS_INFO("ItemSelector: publishing tables");

    items_pub_.publish(visible_tables);
}

void ItemSelector::publishEmptyTables()
{
    gui_item_selector::items visible_tables;
    roscomm::stringlist table;
    roscomm::intlist current_selection;

    visible_tables.tables.push_back(table);
    visible_tables.tables.push_back(table);

    visible_tables.current_selection.push_back(current_selection);
    visible_tables.current_selection.push_back(current_selection);

    visible_tables.selection_mode.push_back(eNoSelection);
    visible_tables.selection_mode.push_back(eNoSelection);

    items_pub_.publish(visible_tables);
}

void ItemSelector::sendFeedback()
{
    ROS_INFO("ItemSelector::sendFeedback()");

    feedback_.current_selection.clear();
    smc_.sendServerFeedback(feedback_);
}

void ItemSelector::sendResult()
{
    ROS_INFO("ItemSelector::sendResult()");
    result_.current_selection.clear();

    roscomm::stringlist current_table_selection;

    vector<SelectableItem*> parameterlist = item_table_list_->get_itemlist();
    for ( auto parameter = parameterlist.begin() ; parameter != parameterlist.end() ; parameter++ )
    {
        current_table_selection.values.clear();

        if ((*parameter)->get_reference() != NULL )
        {
            SelectableItemTable* itemtable = (*parameter)->get_reference();
            checkForAndStoreNewItems( itemtable, itemtable->getSelectedItemPositions() );
            current_table_selection.values = itemtable->getSelectedItemIds();
        }

        result_.current_selection.push_back(current_table_selection);
    }

    publishEmptyTables();

    smc_.sendServerResult(true, result_);
}

// Callback functions
void ItemSelector::CB_itemActivated( const gui_item_selector::item_selected::ConstPtr& selection )
{
    int table_nr = selection->column;
    int row_nr   = selection->row;

    ROS_INFO("ItemSelector::itemActivated(int table_nr, int column_nr): table_nr: %i, row_nr: %i", table_nr, row_nr);

    if ( table_nr == 0 )
    {
        item_table_list_->itemSelected( row_nr );

        last_selected_item_ = 0;
    }

    if ( table_nr == 1 )
    {
        vector<SelectableItem*> parameterlist = item_table_list_->get_itemlist();
        for ( auto parameter = parameterlist.begin() ; parameter != parameterlist.end() ; parameter++ )
            if ((*parameter)->get_selected())
                (*parameter)->get_reference()->itemSelected( row_nr );

        last_selected_item_ = row_nr;
    }

    publishTables();
}

void ItemSelector::CB_removeItem( const gui_item_selector::item_selected::ConstPtr& selection )
{
    int table_nr = selection->column;
    int row_nr   = selection->row;

    ROS_INFO("ItemSelector::removeItem(int table_nr, int column_nr): table_nr: %i, row_nr: %i", table_nr, row_nr);

    if ( table_nr == 0 )
        ROS_ERROR("This cannot be removed");

    SelectableItem* selected_item;
    SelectableItem* selected_table;

    if ( table_nr == 1 )
    {
        vector<SelectableItem*> parameterlist = item_table_list_->get_itemlist();
        for ( auto parameter = parameterlist.begin() ; parameter != parameterlist.end() ; parameter++ )
            if ((*parameter)->get_selected())
                selected_table = (*parameter);
    }

    selected_item = selected_table->get_reference()->getSelectableItem( row_nr );
    removeItemFromDatabase(selected_item->get_id());

    selected_table->get_reference()->removeItem( row_nr );

    publishTables();
}

void ItemSelector::CB_selectionFinished( const gui_item_selector::item_selection::ConstPtr& message )
{
    if (smc_.hasActiveGoal() )
        sendResult();
}

void ItemSelector::CB_removeSelection( const gui_item_selector::item_selection::ConstPtr& message )
{
    //! @todo MdL: Check message.

    // Current selection
    int selected_item = -1;
    if ( item_table_list_->getSelectedItemPositions().size() > 0 )
        selected_item = item_table_list_->getSelectedItemPositions().front();

    ROS_INFO("First table filled");

    if ( selected_item >= 0  && item_table_list_->getSelectableItem(selected_item)->get_reference() != NULL)
    {
        ROS_INFO("Table has reference");
        // Get second table
        SelectableItemTable* itemlist = item_table_list_->getSelectableItem(selected_item)->get_reference();
        
        vector<int> current_selection = itemlist->getSelectedItemPositions();
        vector<string> current_selection_ids = itemlist->getSelectedItemIds();

        // Remove from current list
        removeItems(itemlist, current_selection);

        // Remove from database
        for ( auto& id : current_selection_ids )
            removeItemFromDatabase(id);
    }

    publishTables();
}

void ItemSelector::removeItems( SelectableItemTable* itemlist, vector<int> items )
{
    std::sort(items.begin(), items.end());
    for ( auto& item : items )
        ROS_INFO("I have to remove: %d", item);

    //Backwards through the array to not mess up the order before the removed items
    for ( int i = items.size()-1 ; i >= 0 ; i-- )
        itemlist->removeItem(items[i]);
}

void ItemSelector::checkForAndStoreNewItems( SelectableItemTable* itemtable, vector<int> item_ids )
{
    ROS_INFO_NAMED(ROS_NAME, "ItemSelector::checkForAndStoreNewItems");

    for ( auto& item_id : item_ids )
    {
        auto item = itemtable->get_itemlist()[item_id];

        ROS_INFO_NAMED(ROS_NAME, "Checking %s", item->get_id().c_str());

        if ( item->get_id() == "none")
        {
            ROS_INFO_NAMED(ROS_NAME, "Finding %s", item->get_name().c_str());
            if ( item->get_name() == " New item" )
            {
                Item empty_item("Stored item");
                empty_item = datamanager_->store<Item>(empty_item);
                item->set_id(empty_item.get_id());

                ROS_INFO_NAMED(ROS_NAME, "Stored %s", item->get_id().c_str());
            }

            if ( item->get_name() == " New person" )
            {
                Person empty_person("Stored person");
                empty_person = datamanager_->store<Person>(empty_person);
                item->set_id(empty_person.get_id());

                ROS_INFO_NAMED(ROS_NAME, "Stored %s", item->get_id().c_str());
            }

            if ( item->get_name() == " New waypoint" )
            {
                Waypoint empty_waypoint("Stored waypoint");
                empty_waypoint = datamanager_->store<Waypoint>(empty_waypoint);
                item->set_id(empty_waypoint.get_id());

                ROS_INFO_NAMED(ROS_NAME, "Stored %s", item->get_id().c_str());
            }
        }
    }
}

void ItemSelector::removeItemFromDatabase( const std::string id )
{
    if ( boost::starts_with(id, Item::IDENTIFIER) )
    {
        Item item = datamanager_->get<Item>(id);
        datamanager_->deleteObject<Item>(item);
    }
    else if ( boost::starts_with(id, Waypoint::IDENTIFIER) )
    {
        Waypoint item = datamanager_->get<Waypoint>(id);
        datamanager_->deleteObject<Waypoint>(item);

        redraw_waypoints_pub_ = n_.advertise<std_msgs::Empty>("/map_display/redraw_waypoints", 1, true);
        std_msgs::Empty empty;
        redraw_waypoints_pub_.publish(empty);
    }
    else if ( boost::starts_with(id, Person::IDENTIFIER) )
    {
        Person item = datamanager_->get<Person>(id);
        datamanager_->deleteObject<Person>(item);;
    }
    else 
    {
        ROS_ERROR("Could not delete item with id %s", id.c_str());
        return;
    }

    ROS_DEBUG_NAMED(ROS_NAME, "Removed object with id %s", id.c_str());
}