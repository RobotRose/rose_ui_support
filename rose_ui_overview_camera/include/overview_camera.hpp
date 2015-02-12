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
#ifndef OVERVIEW_CAMERA_HPP
#define OVERVIEW_CAMERA_HPP

#include <iostream>
#include <mutex>          // std::mutex
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>

#include "arm_controller_helper.hpp"
#include "bounding_box_factory/bounding_box_factory.hpp"
#include "rose_datamanager_api/datamanager_api.hpp"

#include "bounding_box_finder/BoundingBox.h"
#include "bounding_box_finder/BoundingBoxVector.h"
#include "bounding_box_finder/convert_bb_to_uv.h"
#include "bounding_box_finder/toggle.h"
#include "bounding_box_finder/uv_point.h"
#include "bounding_box_finder/uv_bounding_box.h"

#include "gui_overview_camera/selection.h"
#include "gui_overview_camera/selections.h"
#include "point_extractor/get_point.h"

#include "rose_parameter_manager/parameterAction.h"
#include "rose_parameter_manager/parameterGoal.h"
#include "rose_parameter_manager/parameterFeedback.h"
#include "rose_parameter_manager/parameterResult.h"

#include "parameter_request_message.hpp"

#include "server_multiple_client/server_multiple_client.hpp"
#include "std_msgs/String.h"
#include "rose_common/common.hpp"
#include "operator_messaging/operator_messaging.hpp"

class OverviewCamera
{
public:
	typedef rose_parameter_manager::parameterAction ParameterAction;
  	typedef ServerMultipleClient<ParameterAction> SMC;

	OverviewCamera( string name, ros::NodeHandle n );
	~OverviewCamera();
	
private:
	void CB_serverCancel( SMC* smc );
	void CB_serverWork( const rose_parameter_manager::parameterGoalConstPtr& goal, SMC* smc );
    void sendResult( bool succes );

	void CB_bounding_box_selected( const gui_overview_camera::selection& selection );
	void CB_pointClicked( const gui_overview_camera::selection& selection );
	void CB_newBoundingBoxes( const bounding_box_finder::BoundingBoxVector bb_vector );

	void requestBoundingBoxes( const bool on );
	void sendNewBoundingBoxes();
	bool getSelectedBoundingBox( const int x, const int y, bounding_box_finder::BoundingBox& bounding_box );

	gui_overview_camera::selections 	rectangleSelectionFromBoundingBox( bounding_box_finder::BoundingBoxVector bounding_boxes );

	DatamanagerAPI* 	 datamanager_;
	ArmControllerHelper* arm_controller_helper_;
	SMC*				 smc_;
	
	ros::Publisher      selection_request_pub_;
	ros::Publisher      bounding_boxex_in_camera_pub_;

	ros::Subscriber 	bounding_box_selected_sub_;
	ros::Subscriber 	bounding_boxes_in_camera_sub_;
	ros::Subscriber 	clicked_point_in_camera_sub_;

	ros::ServiceClient  bb_converter_service_;
	ros::ServiceClient  find_point_service_;
	ros::ServiceClient  toggle_bb_service_;

	ros::NodeHandle 	n_;
	std::string 		name_;

	bounding_box_finder::BoundingBoxVector 		bounding_boxes_;
	gui_overview_camera::selections				rectangles_;
    OperatorMessaging*			                operator_gui_;
};

#endif // OVERVIEW_CAMERA_HPP
