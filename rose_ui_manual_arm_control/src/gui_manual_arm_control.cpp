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
#include "rose_ui_manual_arm_control/gui_manual_arm_control.hpp"

GuiManualArmControl::GuiManualArmControl( std::string name, ros::NodeHandle n )
    : name_ ( name )
    , n_ ( n )
{
    ROS_INFO("GuiManualArmControl::GuiManualArmControl()::begin");

    smc_ = new SMC(n_, name_, boost::bind(&GuiManualArmControl::CB_serverWork, this, _1, _2),
                       boost::bind(&GuiManualArmControl::CB_serverCancel, this, _1));

    // add all clients
    clients_.push_back("/arm_controller/gripper_width");
    addClients();

    smc_->startServer();

    selectLeftGripperSubscriber_        = n.subscribe("/gui/left_arm_select",   1,  &GuiManualArmControl::selectLeftCallback, this);
//    selectRightGripperSubscriber_= n.subscribe("/gui/right_arm_select",  1,  &GuiManualArmControl::selectRightCallback, this);
    gripperOpenWidthSubscriber_         = n.subscribe("/gui/gripper_width",     1,  &GuiManualArmControl::openWidthCallback, this);
    gripperPressureSubscriber_          = n.subscribe("/gui/gripper_pressure",  1,  &GuiManualArmControl::pressureCallback, this);

    // Get one arm of the arm controller, for now
    //! @todo MdL [IMPR]: Store multiple arms.
    ros::ServiceClient get_arms_client  = n.serviceClient<rose_arm_controller_msgs::get_arms>("/arm_controller/get_arms");

    // Wait for this service to have come up
    ros::service::waitForService("/arm_controller/get_arms");

    rose_arm_controller_msgs::get_arms get_arms_srv;
    while ( not get_arms_client.call(get_arms_srv) )
    {
        selected_arm_ = get_arms_srv.response.arms[0];
        ros::Duration(0.5).sleep();
    }

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
        smc_->addClient<rose_arm_controller_msgs::set_gripper_widthAction>(client, 
            boost::bind(&GuiManualArmControl::CB_action_success, this, _1, _2),
            boost::bind(&GuiManualArmControl::CB_action_fail, this, _1, _2),
            NULL, NULL);
    }

    ROS_INFO("GuiManualArmControl::addClients()::end");
}

// Client callbacks
void GuiManualArmControl::CB_action_success( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_gripper_widthResultConstPtr& result )
{
    ROS_INFO("GuiManualArmControl::CB_action_success");
    smc_->cancelAllClients();
}

void GuiManualArmControl::CB_action_fail( const actionlib::SimpleClientGoalState& state, const rose_arm_controller_msgs::set_gripper_widthResultConstPtr& result )
{
    ROS_INFO("GuiManualArmControl::CB_action_fail");
    ROS_INFO("number of busy clients: %d", smc_->getNumberOfBusyClients());
    // Last client has sent cancel
}

void GuiManualArmControl::CB_serverWork( const rose_arm_controller_msgs::set_gripper_widthGoalConstPtr& goal, SMC* smc )
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
        ROS_WARN("Callback not implemented");
}

//void GuiManualArmControl::selectRightCallback(const std_msgs::String::ConstPtr& msg)
//{
//    if(strcmp(msg->data.c_str(), "enable") == 0)
//    {
//        ROS_INFO("Selected right arm");
//        selected_arm_ = 0;
//    }
//}

void GuiManualArmControl::openWidthCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int width = msg->data;
    ROS_INFO("Width for arm %s: %i", selected_arm_.c_str(), width);

    rose_arm_controller_msgs::set_gripper_widthGoal goal_message;
    goal_message.arm                        = selected_arm_;
    goal_message.required_width             = 0.15*width; //! @todo MdL [IMPR]: Get max gripper width from the arm controller.

    smc_->sendGoal<rose_arm_controller_msgs::set_gripper_widthAction>(goal_message);
}

void GuiManualArmControl::pressureCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_ERROR("Pressure for arm %s: %i. Not implemented", selected_arm_.c_str(), msg->data);
}
