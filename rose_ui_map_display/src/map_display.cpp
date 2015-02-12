/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2014/03/05
*         - File created.
*
* Description:
*    description
* 
***********************************************************************************/
#include "map_display.hpp"

MapDisplay::MapDisplay( string name, ros::NodeHandle n )
	: name_ ( name )
	, n_ ( n )
{
	datamanager_ = new DatamanagerAPI();

    smc_ = new SMC(n_, name_, boost::bind(&MapDisplay::CB_serverWork, this, _1, _2),
                              boost::bind(&MapDisplay::CB_serverCancel, this, _1));
    
    // Publishers
    selection_request_pub_      = n_.advertise<std_msgs::String>(                           "/map_display/request",         1, true );
    selection_cancel_pub_       = n_.advertise<std_msgs::Empty>(                            "/map_display/request_cancel",  1, true );
    navigation_path_pub_        = n_.advertise<nav_msgs::Path>(                             "/map_display/navigation_path", 1, true );
    robot_footprint_pub_        = n_.advertise<gui_map_display::colored_polygon_stamped>(   "/map_display/robot_footprint", 1, true );
    map_pub_                    = n_.advertise<nav_msgs::OccupancyGrid>(                    "/map_display/map",             1, true );
    waypoints_pub_              = n_.advertise<gui_map_display::waypoint_array>(            "/map_display/waypoints",       1, true );
    
    // Subscribers
    map_sub_                = n_.subscribe( "/map_planner",                             1, &MapDisplay::CB_map,               this );
    location_selected_sub_  = n_.subscribe( "/map_display/selection",                   1, &MapDisplay::CB_location_selected, this );
    navigation_sub_         = n_.subscribe( "/move_base/arc_local_planner/global_plan", 1, &MapDisplay::CB_nagivation,        this );
    robot_footprint_sub_    = n_.subscribe( "/move_base/local_costmap/laser_obstacle_layer_footprint/footprint_stamped",
                                                                                        1, &MapDisplay::CB_robotFootprint,    this );
    robot_footprint_sub_    = n_.subscribe( "/move_base/local_costmap/laser_obstacle_layer_footprint/footprint_stamped",
                                                                                        1, &MapDisplay::CB_robotFootprint,    this );
    bumpers_sub_            = n_.subscribe( "/lift_controller/bumpers/state",           1, &MapDisplay::CB_bumpers,           this );
    save_location_sub_      = n_.subscribe( "/map_display/save_current_location",       1, &MapDisplay::CB_saveCurrentLocation, this );
    redraw_waypoints_sub_   = n_.subscribe( "/map_display/redraw_waypoints",            1, &MapDisplay::CB_redrawWaypoints,   this );

    smc_->addClient<rose_parameter_manager::parameterAction>("parameter_manager");
    smc_->startServer();

    publishWaypoints();

    initializeFootprintAndBumperPolygon();
}

MapDisplay::~MapDisplay()
{

}

void MapDisplay::publishWaypoints()
{
    /*
        Get all waypoints
        for each waypoint,
            convert to msg
            add msg to array
        publish array
     */
    gui_map_display::waypoint_array msg_array;

    std::vector<Waypoint> waypoints = datamanager_->getAll<Waypoint>();
    ROS_INFO_NAMED(ROS_NAME, "There are %d waypoints", (int)waypoints.size());

    for (auto &wp : waypoints)
    {
        if (wp.get_name() != "waypoint10")
        msg_array.waypoints.push_back(waypointToMsg(wp));
    }

//    ROS_INFO_NAMED(ROS_NAME, "Publishing WP array");
    waypoints_pub_.publish(msg_array);
}

gui_map_display::waypoint MapDisplay::waypointToMsg(Waypoint wp)
{
    gui_map_display::waypoint msg;
    PoseStamped poseStamped = wp.getPoseStamped();

    bool success = rose_transformations::transformToFrame(tf_, "/map", poseStamped);
    if(success)
    {
        msg.x = poseStamped.pose.position.x;
        msg.y = poseStamped.pose.position.y;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(poseStamped.pose.orientation, quat);
        msg.theta = rose_conversions::quaternionToRPY(quat).z; //Yaw is rotation around z
    }
    else
    {
        ROS_ERROR_NAMED(ROS_NAME, "Could not transform waypoint %s to /map-frame", wp.get_name().c_str());
    }

    msg.name = wp.get_name();
    msg.id = wp.get_id();

    return msg;
}

void MapDisplay::CB_redrawWaypoints( const std_msgs::Empty& )
{
    publishWaypoints();
}

void MapDisplay::CB_location_selected( const gui_map_display::selection& selection )
{
    if (not smc_->hasActiveGoal() )
        return;

    rose_parameter_manager::parameterGoalConstPtr goal = smc_->getLastGoal();

    geometry_msgs::PoseStamped waypoint_pose;

    waypoint_pose.header.frame_id = "/map";
    waypoint_pose.header.seq      = 0;
    waypoint_pose.header.stamp    = ros::Time::now();

    waypoint_pose.pose.position.x = selection.x;
    waypoint_pose.pose.position.y = selection.y;
    waypoint_pose.pose.position.z = 0.0;

    waypoint_pose.pose.orientation = rose_conversions::RPYToQuaterion(0.0, 0.0, selection.angle);

    Waypoint waypoint = datamanager_->get<Waypoint>(goal->item_id);
    waypoint.set(waypoint_pose);
    waypoint.set_temp(true);
    datamanager_->store<Waypoint>(waypoint);

    sendResult( true );
    selection_cancel_pub_.publish(std_msgs::Empty());
}

void MapDisplay::sendResult( bool succes )
{
    rose_parameter_manager::parameterResult result;
    smc_->sendServerResult( succes, result );
}

void MapDisplay::CB_serverCancel( SMC* smc )
{

}

void MapDisplay::CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc )
{
    std_msgs::String text;
    switch (goal->parameter)
    {
        case PARAMETER_REQUEST::MAP_LOCATION:
            text.data = "Select map location";// + goal->item_id;
            selection_request_pub_.publish( text );
            break;
        default:
            sendResult( false );
    }
}

void MapDisplay::CB_nagivation( const nav_msgs::Path::ConstPtr &msg )
{
    ROS_INFO_NAMED(ROS_NAME, "MapDisplay::CB_nagivation");

    // Just republish for gui map display node
    navigation_path_pub_.publish(msg);
}

void MapDisplay::CB_map( const nav_msgs::OccupancyGrid::ConstPtr& msg )
{
    ROS_INFO_NAMED(ROS_NAME, "MapDisplay::CB_map");

    // Just republish for gui map display node
    map_pub_.publish(msg);
}

void MapDisplay::CB_bumpers( const rose20_platform::bumpers_state::ConstPtr& msg )
{
    ROS_DEBUG_NAMED(ROS_NAME, "Received %i bumpers", msg->bumper_count);
    latest_bumper_state_ = msg;

    publishFootprint();
}

geometry_msgs::Polygon MapDisplay::transformPolygonToMapFrame( const geometry_msgs::PolygonStamped::ConstPtr polygon )
{
    // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame");
    geometry_msgs::Polygon          result;

    geometry_msgs::PointStamped     point_stamped;
    geometry_msgs::PointStamped     converted_point_stamped;

    geometry_msgs::Point32 point;
    BOOST_FOREACH(point, polygon->polygon.points)
    {
        // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::foreach");
        point_stamped.header            = polygon->header;
        // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::rose_conversions::point32ToPoint-before");
        // ROS_DEBUG_NAMED(ROS_NAME, "point x: %f, y: %f, z: %f", point.x, point.y, point.z);
        point_stamped.point             = rose_conversions::point32ToPoint(point);
        // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::rose_conversions::point32ToPoint-after");
        // ROS_DEBUG_NAMED(ROS_NAME, "point x: %f, y: %f, z: %f", point_stamped.point.x, point_stamped.point.y, point_stamped.point.z);
        try
        {
            tf_.transformPoint("map", point_stamped, converted_point_stamped);
            // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::push back result");
            result.points.push_back(rose_conversions::pointToPoint32(converted_point_stamped.point));
            // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::push back result done");
            // ROS_DEBUG_NAMED(ROS_NAME, "point x: %f, y: %f, z: %f", converted_point_stamped.point.x, converted_point_stamped.point.y, converted_point_stamped.point.z);
        }
        catch( tf::TransformException& ex )
        {
            ROS_ERROR("Received an exception: %s", ex.what());
            return result;
        }
        // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::end foreach");
    }

    // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::transformPolygonToMapFrame::done");
    return result;
}

void MapDisplay::CB_robotFootprint( const geometry_msgs::PolygonStamped::ConstPtr &msg )
{
    ROS_INFO_NAMED(ROS_NAME, "MapDisplay::CB_robotFootprint");

    // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::lock");
    syncroot.lock();
    // Footprint is in odom, translate to map
    geometry_msgs::Polygon converted_msg = transformPolygonToMapFrame(msg);

    if ( converted_msg.points.size() != 4 )
    {
        syncroot.unlock();
        return;
    }

    // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::publish");
    robot_footprint_pub_.publish(converted_msg);
    // ROS_INFO_NAMED(ROS_NAME, "MapDisplay::unlock");
    syncroot.unlock();
}

void MapDisplay::publishFootprint()
{
    std_msgs::ColorRGBA base_color; //Green
    base_color.r = 0.0;
    base_color.g = 1.0;
    base_color.b = 0.0;
    base_color.a = 1.0;

    std_msgs::ColorRGBA collision_color; //Red
    collision_color.r = 1.0;
    collision_color.g = 0.0;
    collision_color.b = 0.0;
    collision_color.a = 1.0;

    gui_map_display::colored_polygon_stamped in_map(footprint_poly_);

    for(int i =0; i<(int)in_map.polygon.points.size(); i++)
    {
        auto point = in_map.polygon.points[i];
        geometry_msgs::PointStamped ps;
        ps.header.frame_id = in_map.header.frame_id; //is base_link because we copied from the already initialized footprint_poly_
        ps.point = rose_conversions::point32ToPoint(point);
        if(rose_transformations::transformToLatestFrame(tf_, "/map", ps, 0.1))
        {
            auto transformed_point = rose_conversions::pointToPoint32(ps.point);
            in_map.polygon.points[i] = transformed_point;
        }
        else
        {
            ROS_DEBUG_NAMED(ROS_NAME, "Could not transform point in base_link to map");
        }
    }
    in_map.header.frame_id = "/map";

    in_map.colors.clear();
    //First make everything the normal color
    for(auto point : in_map.polygon.points)
    {
        in_map.colors.push_back(base_color);
    }

    //Then iterate over all lines that correspond to a bumper and color those collision_color if they are in collision
    if(latest_bumper_state_)
    {
        for (auto& bumper_line : bumper_line_mapping_)
        {
            int bumper_index = bumper_line.first;
            int line_index = bumper_line.second;

//            ROS_DEBUG_NAMED(ROS_NAME, "Bumper %i = %s", bumper_index, (latest_bumper_state_->bumper_states[bumper_index]?"pressed":"released"));

            if(latest_bumper_state_->bumper_states[bumper_index]) //True means pressed
            {
                ROS_DEBUG_NAMED(ROS_NAME, "Bumper active for segment %i", line_index);
                in_map.colors[line_index] = collision_color;
            }
        }
    }
    else
    {
        ROS_WARN_ONCE("MapDisplay::publishFootprint No bumper states received yet");
    }

    robot_footprint_pub_.publish(in_map);
}

void MapDisplay::initializeFootprintAndBumperPolygon()
{
    /*
            S1      S2
             +######+
LF   F11 F12 |  7   |   F21 F22 RF
    +-+##+---+      +---+##+-+
    |   0   SL      SR    6  |
L22 +                        + R11
    #  1                     #
    #                     5  #
L21 +                        + R12
    |                        |
    |                        |
    |           *            |
    |                        |
    |                        |
L12 +                        + R21
    #  2                     #
    #                     4  #
L11 +                        + R22
    |  3                 3   |
    +-+##+--------------+##+-+ RB
  LB B22 B21          B12 B11

   Bumper order in hardware and message is this:
   0,1,2,3,4,5,6,7 = F1, L2, L2, B2, B1, R2, R1, F2, S
    */
    geometry_msgs::Point32 base_link;

    geometry_msgs::Point32 left_front(base_link);
    geometry_msgs::Point32 right_front(left_front);
    geometry_msgs::Point32 left_back(left_front);
    geometry_msgs::Point32 right_back(left_front);

    left_front.x += FOOTPRINT_LENGTH/2;
    left_front.y += FOOTPRINT_WIDTH/2;

    right_front.x += FOOTPRINT_LENGTH/2;
    right_front.y -= FOOTPRINT_WIDTH/2;

    left_back.x -= FOOTPRINT_LENGTH/2;
    left_back.y += FOOTPRINT_WIDTH/2;

    right_back.x -= FOOTPRINT_LENGTH/2;
    right_back.y -= FOOTPRINT_WIDTH/2;

    geometry_msgs::Point32 SL(left_front);
    geometry_msgs::Point32 S1(left_front);
    geometry_msgs::Point32 S2(left_front);
    geometry_msgs::Point32 SR(left_front);

    geometry_msgs::Point32 F11(left_front);
    geometry_msgs::Point32 F12(left_front);
    geometry_msgs::Point32 F21(left_front);
    geometry_msgs::Point32 F22(left_front);

    geometry_msgs::Point32 B11(right_back);
    geometry_msgs::Point32 B12(right_back);
    geometry_msgs::Point32 B21(right_back);
    geometry_msgs::Point32 B22(right_back);

    geometry_msgs::Point32 R11(right_front);
    geometry_msgs::Point32 R12(right_front);
    geometry_msgs::Point32 R21(right_front);
    geometry_msgs::Point32 R22(right_front);

    geometry_msgs::Point32 L11(left_back);
    geometry_msgs::Point32 L12(left_back);
    geometry_msgs::Point32 L21(left_back);
    geometry_msgs::Point32 L22(left_back);

    F11.y -= 0.03; //TODO determine value
    F12.y -= 0.18; //TODO determine value
    F21.y -= 0.42; //TODO determine value
    F22.y -= 0.57; //TODO determine value

    SL.y  -= 0.25; //TODO determine value
    SR.y  -= 0.35; //TODO determine value
    S1.y  -= 0.25; //TODO determine value
    S2.y  -= 0.35; //TODO determine value
    S1.x  += 0.10; //TODO determine value
    S2.x  += 0.10; //TODO determine value

    R11.x -= 0.03; //TODO determine value
    R12.x -= 0.18; //TODO determine value
    R21.x -= 0.62; //TODO determine value
    R22.x -= 0.77; //TODO determine value

    B11.y += 0.03; //TODO determine value
    B12.y += 0.18; //TODO determine value
    B21.y += 0.42; //TODO determine value
    B22.y += 0.57; //TODO determine value

    L11.x += 0.03; //TODO determine value
    L12.x += 0.18; //TODO determine value
    L21.x += 0.62; //TODO determine value
    L22.x += 0.77; //TODO determine value

    footprint_poly_ = gui_map_display::colored_polygon_stamped();
    footprint_poly_.header.frame_id = "/base_link";

    //Bumper drawing above is defined in clockwise order, but bumpers in embedded software are defined in counter-clockwise order:
    //0,1,2,3,4,5,6,7 = F1, L2, L1, B2, B1, R2, R1, F2, S

    //Bumper F1
    footprint_poly_.polygon.points.push_back(F12);
    bumper_line_mapping_[0] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(F11);

    footprint_poly_.polygon.points.push_back(left_front); //Has no line index

    //Bumper L2
    footprint_poly_.polygon.points.push_back(L22);
    bumper_line_mapping_[1] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(L21);

    //Bumper L1
    footprint_poly_.polygon.points.push_back(L12);
    bumper_line_mapping_[2] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(L11);

    footprint_poly_.polygon.points.push_back(left_back);

    //Bumper B2
    footprint_poly_.polygon.points.push_back(B22);
    bumper_line_mapping_[3] = (int)footprint_poly_.polygon.points.size()-1;
//    footprint_poly_.polygon.points.push_back(B21);

    //Bumper B1
//    footprint_poly_.polygon.points.push_back(B12);
//    bumper_line_mapping_[3] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(B11);

    footprint_poly_.polygon.points.push_back(right_back);

    //Bumper R2
    footprint_poly_.polygon.points.push_back(R22);
    bumper_line_mapping_[4] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(R21);

    //Bumper R1
    footprint_poly_.polygon.points.push_back(R12);
    bumper_line_mapping_[5] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(R11);

    footprint_poly_.polygon.points.push_back(right_front);

    //Bumper F2
    footprint_poly_.polygon.points.push_back(F22);
    bumper_line_mapping_[6] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(F21);

    footprint_poly_.polygon.points.push_back(SR);

    //Bumper S(canner)
    footprint_poly_.polygon.points.push_back(S2);
    bumper_line_mapping_[7] = (int)footprint_poly_.polygon.points.size()-1;
    footprint_poly_.polygon.points.push_back(S1);

    footprint_poly_.polygon.points.push_back(SL);

    footprint_poly_.polygon.points.push_back(footprint_poly_.polygon.points[0]); //Make a circle

}

void MapDisplay::CB_saveCurrentLocation( const std_msgs::Empty& )
{
    ROS_DEBUG_NAMED(ROS_NAME, "MapDisplay::CB_saveCurrentLocation");
    std::string location_name = "temp";   
    Waypoint current_location(location_name);

    // get current location on map
    PoseStamped robot_pose;
    if ( not rose_transformations::getFrameInFrame(tf_, "base_link", "map", robot_pose))
        return;

    current_location.set(robot_pose);

    current_location = datamanager_->store<Waypoint>(current_location);

    // Request name of the waypoint
    rose_parameter_manager::parameterGoal goal;
    goal.item_id        = current_location.get_id();
    goal.parameter      = PARAMETER_REQUEST::WAYPOINT_NAME;

    smc_->sendGoal<rose_parameter_manager::parameterAction>(goal, "parameter_manager");
    if (not smc_->waitForResult(ros::Duration(60.0)))
        datamanager_->deleteObject<Waypoint>(current_location);

    // publish new set of waypoints
    publishWaypoints();
}
