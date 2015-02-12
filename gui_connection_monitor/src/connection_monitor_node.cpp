/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2014/01/29
* 		- File created.
*
* Description:
*	description
* 
***********************************************************************************/
#include "connection_monitor_node.hpp"

int main( int argc, char **argv )
{
    ros::init(argc, argv, "connection_monitor");
    ros::NodeHandle n;

    int rate;
    std::string topic;

    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("rate", rate, int(1));
    private_node_handle_.param("topic", topic, std::string("connection_monitor"));

    ros::Rate r(rate);

    std_msgs::Bool onoff;
    onoff.data = false;

    ros::Publisher connection_publisher = n.advertise<std_msgs::Bool>("/HeartBeat", 1);
    while (n.ok() )
    {
        onoff.data = !onoff.data;
        connection_publisher.publish(onoff);
        ros::spinOnce();

        r.sleep();
    }

    return 0;
}