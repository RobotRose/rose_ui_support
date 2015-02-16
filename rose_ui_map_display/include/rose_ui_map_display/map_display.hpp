/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/03/05
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#ifndef MAP_DISPLAY_HPP
#define MAP_DISPLAY_HPP

#include <boost/thread/mutex.hpp>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <rose_ui_map_display/colored_polygon_stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "rose_datamanager_api/datamanager_api.hpp"

#include "rose_ui_map_display/selection.h"
#include "rose_ui_map_display/waypoint.h"
#include "rose_ui_map_display/waypoint_array.h"
#include "rose_parameter_manager/parameterAction.h"
#include "rose_parameter_manager/parameterGoal.h"
#include "rose_parameter_manager/parameterFeedback.h"
#include "rose_parameter_manager/parameterResult.h"

#include "parameter_request_message.hpp"

#include "server_multiple_client/server_multiple_client.hpp"
#include "rose_common/common.hpp"
#include "rose_conversions/conversions.hpp"
#include "rose_transformations/transformations.hpp"
#include "rose_geometry/geometry.hpp"
#include "std_msgs/String.h"
#include "rose20_platform/bumpers_state.h"

#define FOOTPRINT_WIDTH 0.60
#define FOOTPRINT_LENGTH 0.80

class MapDisplay
{
public:
	typedef rose_parameter_manager::parameterAction ParameterAction;
  	typedef ServerMultipleClient<ParameterAction> SMC;

	MapDisplay( string name, ros::NodeHandle n );
	~MapDisplay();
    void publishFootprint();
    void publishBumpers();
	
private:
	void CB_serverCancel( SMC* smc );
	void CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc );
	void sendResult( bool succes );

	void CB_location_selected( const rose_ui_map_display::selection& selection );
	void CB_robotFootprint( const geometry_msgs::PolygonStamped::ConstPtr &msg );
	void CB_nagivation( const nav_msgs::Path::ConstPtr &msg );
	void CB_map( const nav_msgs::OccupancyGrid::ConstPtr& msg );
    void CB_bumpers( const rose20_platform::bumpers_state::ConstPtr& msg);
    void CB_saveCurrentLocation( const std_msgs::Empty& );
    void CB_redrawWaypoints( const std_msgs::Empty& );

    void initializeFootprintAndBumperPolygon();

	void publishWaypoints();
	rose_ui_map_display::waypoint waypointToMsg(Waypoint wp);

	geometry_msgs::Polygon transformPolygonToMapFrame( const geometry_msgs::PolygonStamped::ConstPtr polygon );

	DatamanagerAPI* 	datamanager_;
	SMC*				smc_;

	ros::NodeHandle 	n_;
	std::string 		name_;

    rose20_platform::bumpers_state::ConstPtr latest_bumper_state_;
    rose_ui_map_display::colored_polygon_stamped footprint_poly_;
    std::map<int, int> bumper_line_mapping_; //TODO: Maybe make the values vectors, so one bumper can map to a collection of line segments
	
	ros::Publisher      selection_request_pub_;
    ros::Publisher      selection_cancel_pub_;
	ros::Publisher      navigation_path_pub_;
	ros::Publisher		robot_footprint_pub_;
	ros::Publisher		map_pub_;
	ros::Publisher		waypoints_pub_;

	ros::Subscriber 	location_selected_sub_;
	ros::Subscriber		robot_footprint_sub_;
	ros::Subscriber		navigation_sub_;
	ros::Subscriber		map_sub_;
    ros::Subscriber		bumpers_sub_;
    ros::Subscriber		save_location_sub_;
    ros::Subscriber		redraw_waypoints_sub_;

	tf::TransformListener  	tf_;
	boost::timed_mutex		syncroot;

};

#endif // MAP_DISPLAY_HPP
