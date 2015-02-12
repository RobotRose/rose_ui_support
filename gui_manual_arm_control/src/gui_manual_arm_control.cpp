/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2014/02/14
*       - File created.
*
* Description:
*   description
* 
***********************************************************************************/
#include "gui_manual_arm_control/gui_manual_arm_control.hpp"

GuiManualArmControl::GuiManualArmControl( std::string name, ros::NodeHandle n )
    : name_ ( name )
    , n_ ( n )
{
    ROS_INFO("GuiManualArmControl::GuiManualArmControl()::begin");

    arm_controller_helper_ 	= new ArmControllerHelper();
    selectedArm_ = arm_controller_helper_->getArms()[0];

    smc_ = new SMC(n_, name_, boost::bind(&GuiManualArmControl::CB_serverWork, this, _1, _2),
                       boost::bind(&GuiManualArmControl::CB_serverCancel, this, _1));

    // add all clients
    clients_.push_back("/arms");
    addClients();

    smc_->startServer();

    selectLeftGripperSubscriber_ = n.subscribe("/gui/left_arm_select",   1,  &GuiManualArmControl::selectLeftCallback, this);
//    selectRightGripperSubscriber_= n.subscribe("/gui/right_arm_select",  1,  &GuiManualArmControl::selectRightCallback, this);
    gripperOpenWidthSubscriber_  = n.subscribe("/gui/gripper_width",     1,  &GuiManualArmControl::openWidthCallback, this);
    gripperPressureSubscriber_   = n.subscribe("/gui/gripper_pressure",  1,  &GuiManualArmControl::pressureCallback, this);

    ROS_INFO("GuiManualArmControl::GuiManualArmControl()::end");
}

GuiManualArmControl::~GuiManualArmControl()
{

}

void GuiManualArmControl::addClients()
{
    ROS_INFO("GuiManualArmControl::addClients()::begin");

    std::string client;
    BOOST_FOREACH(client, clients_)
    {
        smc_->addClient<arm_controller::manipulateAction>(client, 
            boost::bind(&GuiManualArmControl::CB_action_success, this, _1, _2),
            boost::bind(&GuiManualArmControl::CB_action_fail, this, _1, _2),
            NULL, NULL);
    }

    ROS_INFO("GuiManualArmControl::addClients()::end");
}

// Client callbacks
void GuiManualArmControl::CB_action_success( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
    ROS_INFO("GuiManualArmControl::CB_action_success");
    smc_->cancelAllClients();
}

void GuiManualArmControl::CB_action_fail( const actionlib::SimpleClientGoalState& state, const arm_controller::manipulateResultConstPtr& result )
{
    ROS_INFO("GuiManualArmControl::CB_action_fail");
    ROS_INFO("number of busy clients: %d", smc_->getNumberOfBusyClients());
    // Last client has sent cancel
}

void GuiManualArmControl::CB_serverWork( const arm_controller::manipulateGoalConstPtr& goal, SMC* smc )
{
    ROS_INFO("GuiManualArmControl::CB_serverWork");
}

void GuiManualArmControl::CB_serverCancel( SMC* smc )
{
    ROS_INFO("GuiManualArmControl::CB_serverCancel");
}

void GuiManualArmControl::selectLeftCallback(const std_msgs::String::ConstPtr& msg)
{
    if(strcmp(msg->data.c_str(), "enable") == 0)
    {
        ROS_INFO("Selected left arm");
        selectedArm_ = ArmController::Arms::LEFT;
    }
}

//void GuiManualArmControl::selectRightCallback(const std_msgs::String::ConstPtr& msg)
//{
//    if(strcmp(msg->data.c_str(), "enable") == 0)
//    {
//        ROS_INFO("Selected right arm");
//        selectedArm_ = 0;
//    }
//}

void GuiManualArmControl::openWidthCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int width = msg->data;
    ROS_INFO("Width for arm %i: %i", selectedArm_, width);

    arm_controller::manipulateGoal goal_message;
    goal_message.arm                        = selectedArm_;
    goal_message.required_action            = ArmController::Manipulation::MOVE_GRIPPER;
    goal_message.required_gripper_width     = width;

    smc_->sendGoal<arm_controller::manipulateAction>(goal_message);
}

void GuiManualArmControl::pressureCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_ERROR("Pressure for arm %i: %i. Not implemented", selectedArm_, msg->data);
}
