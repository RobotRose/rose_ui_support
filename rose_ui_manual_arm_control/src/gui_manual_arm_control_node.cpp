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
#include "rose_ui_manual_arm_control/gui_manual_arm_control_node.hpp"

int main(int argc, char **argv)
{
    /*Initiate ROS*/
    ros::init(argc, argv, "gui_manual_arm_control");

    ros::NodeHandle n;

    GuiManualArmControl* arm_control = new GuiManualArmControl("gui_manual_arm_control", n);

    ros::spin();

    return 0;
}
