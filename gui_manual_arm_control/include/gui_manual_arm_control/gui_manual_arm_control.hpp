/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Loy van Beek
*   Date  : 2014/09/03
*       - File created.
*
* Description:
*   description
* 
***********************************************************************************/
#ifndef GUI_MANUAL_ARM_CONTROL_HPP
#define GUI_MANUAL_ARM_CONTROL_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

#include "arm_controller.hpp"
#include "arm_controller/manipulateAction.h"
#include "arm_controller/manipulateGoal.h"
#include "arm_controller/manipulateResult.h"
#include "arm_controller/manipulateFeedback.h"
#include "arm_controller_helper.hpp"

#include "server_multiple_client/server_multiple_client.hpp"

class GuiManualArmControl
{    typedef ServerMultipleClient<arm_controller::manipulateAction> SMC;

  public:
    GuiManualArmControl( std::string name, ros::NodeHandle n );
    ~GuiManualArmControl();

  private:
    void addClients();

    void CB_serverCancel( SMC* smc );
    void CB_serverWork( const arm_controller::manipulateGoalConstPtr& goal, SMC* smc );
    void sendResult(bool succes);

  // Custom client succes/fail
    void CB_action_success( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );
    void CB_action_fail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result );

    void selectLeftCallback(const std_msgs::String::ConstPtr& msg);
//    void selectRightCallback(const std_msgs::String::ConstPtr& msg);
    void openWidthCallback(const std_msgs::Int32::ConstPtr& msg);
    void pressureCallback(const std_msgs::Int32::ConstPtr& msg);

    std::string                name_;
    ros::NodeHandle            n_;
    SMC*                       smc_;
    std::vector<std::string>   clients_;
    ArmControllerHelper*       arm_controller_helper_;

    int selectedArm_;

    ros::Subscriber selectLeftGripperSubscriber_;
//    ros::Subscriber selectRightGripperSubscriber_;
    ros::Subscriber gripperOpenWidthSubscriber_;
    ros::Subscriber gripperPressureSubscriber_;
};

#endif //GUI_MANUAL_ARM_CONTROL_HPP
