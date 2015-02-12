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
#include "overview_camera.hpp"

OverviewCamera::OverviewCamera( string name, ros::NodeHandle n )
	: name_ ( name )
    , n_ ( n )
{
	datamanager_           = new DatamanagerAPI();
    arm_controller_helper_ = new ArmControllerHelper();
    operator_gui_          = new OperatorMessaging(n_);

    smc_ = new SMC(n_, name_, boost::bind(&OverviewCamera::CB_serverWork, this, _1, _2),
                              boost::bind(&OverviewCamera::CB_serverCancel, this, _1));
    // Publishers
    selection_request_pub_          = n_.advertise<std_msgs::String>("/overview_camera/request", 1, true);
    bounding_boxex_in_camera_pub_   = n_.advertise<gui_overview_camera::selections>("/overview_camera/bounding_boxes", 1, true);
    
    // Subscribers
    bounding_box_selected_sub_      = n_.subscribe("/overview_camera/selection",               1, &OverviewCamera::CB_bounding_box_selected, this);
    clicked_point_in_camera_sub_    = n_.subscribe("/overview_camera/point_selected",          1, &OverviewCamera::CB_pointClicked,          this);
    bounding_boxes_in_camera_sub_   = n_.subscribe("/bounding_box_finder/bounding_box_vector", 1, &OverviewCamera::CB_newBoundingBoxes,      this);

    bb_converter_service_           = n_.serviceClient<bounding_box_finder::convert_bb_to_uv>("/bounding_box_convert_srv/convert_bb_to_uv");
    toggle_bb_service_              = n_.serviceClient<bounding_box_finder::toggle>("/bounding_box_finder/toggle");
    find_point_service_             = n_.serviceClient<point_extractor::get_point>("/point_extractor/get_point");

    smc_->startServer();
}

OverviewCamera::~OverviewCamera()
{

}

void OverviewCamera::CB_bounding_box_selected( const gui_overview_camera::selection& selection )
{
    if (not smc_->hasActiveGoal() )
        return;

    ROS_INFO("OverviewCamera::CB_bounding_box_selected::begin");
    rose_parameter_manager::parameterGoalConstPtr goal = smc_->getLastGoal();
    
    // Find point in this rectangle
    point_extractor::get_point      get_point_msg;
    get_point_msg.request.x_min = selection.x1;
    get_point_msg.request.x_max = selection.x2;
    get_point_msg.request.y_min = selection.y1;
    get_point_msg.request.y_max = selection.y2;

    if ( not find_point_service_.call(get_point_msg) )
    {
        ROS_ERROR("Point could not be found in this rectangle. Please select again."); //! @todo MdL: This message should be shown in the UI.
        operator_gui_->action("No point found, select again");
        return;
    }

    // To create the bounding box
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header             = get_point_msg.response.point_stamped.header;
    pose_stamped.pose.position      = get_point_msg.response.point_stamped.point;
    pose_stamped.pose.orientation.w = 1.0;

    BoundingBox bb = BoundingBox(pose_stamped, 0, 0, 0);

    ROS_DEBUG_NAMED(ROS_NAME, "Received point (%f, %f, %f)", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);

    switch (goal->parameter)
    {
        case PARAMETER_REQUEST::BOUNDING_BOX:
        {
            Item item;
            item = datamanager_->get<Item>(goal->item_id);
            item.set_bounding_box(bb);
            datamanager_->store<Item>(item);

            ROS_INFO("OverviewCamera::CB_bounding_box_selected::end");
            sendResult( true );
            break;
        }

        case PARAMETER_REQUEST::PLACE_LOCATION:
        {
            Waypoint    waypoint;
            waypoint = datamanager_->get<Waypoint>(goal->item_id);

            waypoint.set(bb.getPoseStamped());
            datamanager_->store<Waypoint>(waypoint);
            auto pos = waypoint.getPoseStamped().pose.position;
            ROS_INFO("OverviewCamera::CB_bounding_box_selected::end: stored waypoint %s at (%2.3f, %2.3f, %2.3f)", waypoint.get_id().c_str(), pos.x, pos.y, pos.z);
            sendResult( true);
            break;
        }
    }
}

void OverviewCamera::CB_pointClicked( const gui_overview_camera::selection& selection )
{
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCamera::CB_pointClicked");
    if (not smc_->hasActiveGoal() )
        return;

    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCamera::CB_pointClicked::begin");
    rose_parameter_manager::parameterGoalConstPtr goal = smc_->getLastGoal();

    bounding_box_finder::BoundingBox bounding_box;
    if ( not getSelectedBoundingBox( selection.x1, selection.y1, bounding_box ))
    {
        ROS_ERROR("No valid bounding box selected");
        return;
    }

    BoundingBox bb = BoundingBox(bounding_box.pose_stamped, bounding_box.dimensions.y, bounding_box.dimensions.z, bounding_box.dimensions.x);
    ROS_DEBUG_NAMED(ROS_NAME, "Bounding box created");

    Item item = datamanager_->get<Item>(goal->item_id);
    item.set_bounding_box(bb);
    datamanager_->store<Item>(item);

    ROS_INFO("OverviewCamera::CB_pointClicked::end");

    sendResult( true );
}

bool OverviewCamera::getSelectedBoundingBox( const int x, const int y, bounding_box_finder::BoundingBox& bounding_box )
{
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCamera::getSelectedBoundingBox (%d, %d)", x, y);

    //! @todo MdL: From stored bb

    for ( int i = 0 ; i < rectangles_.boxes.size() ; i++ )
    {
        if (    rectangles_.boxes.at(i).x1 <= x  
             &&                    x <= rectangles_.boxes.at(i).x2  
             && rectangles_.boxes.at(i).y1 <= y  
             &&                    y <= rectangles_.boxes.at(i).y2
        )
        {
            bounding_box = bounding_boxes_.bounding_box_vector.at(i);
            return true;
        }
    }

    ROS_ERROR("No bounding box found");

    return false;
}

void OverviewCamera::sendResult( bool succes )
{
    requestBoundingBoxes( false );
    rose_parameter_manager::parameterResult result;
    smc_->sendServerResult( succes, result );
}

void OverviewCamera::CB_serverCancel( SMC* smc )
{
    requestBoundingBoxes( false );
}

void OverviewCamera::requestBoundingBoxes( const bool on )
{
    bounding_box_finder::toggle   toggle;
    toggle.request.on = on;

    toggle_bb_service_.call(toggle);
}

void OverviewCamera::CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc )
{
    std_msgs::String text;
    switch (goal->parameter)
    {
        case PARAMETER_REQUEST::BOUNDING_BOX:
            text.data = "Select bounding box";// + goal->item_id;
            requestBoundingBoxes( true );
            selection_request_pub_.publish( text );
            break;      
        case PARAMETER_REQUEST::PLACE_LOCATION:
            text.data = "Select placing location";// + goal->item_id;
            selection_request_pub_.publish( text );
            break;
        default:
            sendResult( false );
    }
}

void OverviewCamera::CB_newBoundingBoxes( const bounding_box_finder::BoundingBoxVector bb_vector )
{
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCamera::CB_newBoundingBoxes");

    ROS_DEBUG_NAMED(ROS_NAME, "LOCKED");
    bounding_boxes_ = bb_vector;

    sendNewBoundingBoxes();
}

void OverviewCamera::sendNewBoundingBoxes()
{
    ROS_DEBUG_NAMED(ROS_NAME, "sendNewBoundingBoxes");
    rectangles_     = rectangleSelectionFromBoundingBox(bounding_boxes_);

    bounding_boxex_in_camera_pub_.publish(rectangles_);
}

gui_overview_camera::selections OverviewCamera::rectangleSelectionFromBoundingBox( bounding_box_finder::BoundingBoxVector bounding_boxes )
{   
    ROS_DEBUG_NAMED(ROS_NAME, "OverviewCamera::rectangleSelectionFromBoundingBox");
    gui_overview_camera::selections         selections;
    bounding_box_finder::convert_bb_to_uv   convert_msg;
    convert_msg.request.bounding_boxes      = bounding_boxes;
        
    if (bb_converter_service_.call(convert_msg))
    {
        std::vector<bounding_box_finder::uv_bounding_box> uv_bounding_boxes = convert_msg.response.uv_bounding_box;

        // Update camera resolution
        selections.width    = convert_msg.response.camera_width;
        selections.height   = convert_msg.response.camera_height;

        gui_overview_camera::selection          selection;
        bounding_box_finder::uv_bounding_box    uv_bounding_box;
        for ( int i = 0 ; i < uv_bounding_boxes.size() ; i++ )
        {
            uv_bounding_box = uv_bounding_boxes.at(i);

            int min_x = selections.height + 1;
            int min_y = selections.height + 1; 
            int max_x = -1;
            int max_y = -1;
            for ( const auto& point : uv_bounding_box.corners )
            {
                min_x = std::min(min_x, (int)point.u);
                max_x = std::max(max_x, (int)point.u);
                min_y = std::min(min_y, (int)point.v);
                max_y = std::max(max_y, (int)point.v);
            }

            selection.x1        = min_x;
            selection.x2        = max_x;
            selection.y1        = min_y;
            selection.y2        = max_y;

            selection.reachable = arm_controller_helper_->reachable(bounding_boxes_.bounding_box_vector.at(i).pose_stamped);

            selections.boxes.push_back(selection);
        }
    }
    else
    {
        ROS_ERROR("Service could not be contacted");
    }

    return selections;
}
