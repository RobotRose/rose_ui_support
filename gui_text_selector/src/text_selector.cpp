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
#include "text_selector.hpp"

TextSelector::TextSelector( std::string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
{
	datamanager_ = new DatamanagerAPI();
    smc_ = new SMC(n_, name_, boost::bind(&TextSelector::CB_serverWork, this, _1, _2),
                              boost::bind(&TextSelector::CB_serverCancel, this, _1));

    // Publishers
    text_pub_       	 = n_.advertise<std_msgs::String>( "/text_selector/request", 1, true );
    text_cancel_pub_     = n_.advertise<std_msgs::String>( "/text_selector/request_cancelled", 1, false );

    // Subscribers
    text_entered_sub_ 	= n_.subscribe( "/text_selector/text_input", 1, &TextSelector::CB_receiveTextInput, this );

    smc_->startServer();
}

TextSelector::~TextSelector()
{

}

void TextSelector::CB_receiveTextInput( const std_msgs::String text )
{
    if (not smc_->hasActiveGoal() )
        return;

    rose_parameter_manager::parameterGoalConstPtr goal = smc_->getLastGoal();

    switch (goal->parameter)
    {
        // case PARAMETER_REQUEST::BOUNDING_BOX:
        //     sendBoundingBoxResult( goal->item_id, text.data );
        //     break;
        // case PARAMETER_REQUEST::MAP_LOCATION:
        //     text.data = "Map location ";// + goal->item_id;
        //     text_pub_.publish( text );
            // break;
        case PARAMETER_REQUEST::ITEM_NAME:
            sendNameResult( text.data );
            break;        
        case PARAMETER_REQUEST::WAYPOINT_NAME:
            sendWaypointNameResult( goal->item_id, text.data );
            break;
        default:
            sendResult( false );
    }
}

void TextSelector::sendBoundingBoxResult( std::string item_id, std::string bounding_box )
{
    BoundingBoxFactory bb_fac;
    BoundingBox bb = bb_fac.createBoundingBox( bounding_box );

    Item item = datamanager_->get<Item>(item_id);
    item.set_bounding_box(bb);

    datamanager_->store<Item>(item);

    sendResult( true );
}

void TextSelector::sendWaypointNameResult( std::string item_id, std::string name )
{
    Waypoint waypoint = datamanager_->get<Waypoint>(item_id);
    waypoint.set_name(name);

    datamanager_->store<Waypoint>(waypoint);

    sendResult( true );
}

void TextSelector::sendNameResult( std::string name )
{
    sendResult( true );
}

void TextSelector::sendResult( bool succes )
{
    rose_parameter_manager::parameterResult result;
    smc_->sendServerResult( succes, result );
}

void TextSelector::CB_serverCancel( SMC* smc )
{
    std_msgs::String text;
    text.data = "";
    text_cancel_pub_.publish(text);
}

void TextSelector::CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc )
{
    ROS_DEBUG_NAMED(ROS_NAME, "TextSelector::CB_serverWork");
    std_msgs::String text;

    switch (goal->parameter)
    {
        // case PARAMETER_REQUEST::BOUNDING_BOX:
        //     text.data = "Please select a bounding box.";// + goal->item_id;
        //     text_pub_.publish( text );
        //     break;
        // case PARAMETER_REQUEST::MAP_LOCATION:
        //     text.data = "Map location ";// + goal->item_id;
        //     text_pub_.publish( text );
            // break;
        case PARAMETER_REQUEST::ITEM_NAME:
            text.data = "Please enter a name.";// + goal->item_id;
            text_pub_.publish( text );
            break;
        case PARAMETER_REQUEST::WAYPOINT_NAME:
            text.data = "Please enter a name.";// + goal->item_id;
            text_pub_.publish( text );
            break;            
        default:
            sendResult( false );
    }
}
