/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*    Author: Mathijs de Langen
*    Date  : 2013/12/11
*         - File created.
*
* Description:
*    Node that publishes audio to a certain ros topic.
* 
***********************************************************************************/

//#define ALSA_PCM_NEW_HW_PARAMS_API

#include "ros/ros.h"

#include "VoiceCapturer.h"

#include <iostream>
#include <sstream>
#include <alsa/asoundlib.h>

int main(int argc, char **argv)
{
    //Get parameters
    if(argc < 5)
    {
        std::cerr << "Error: Wrong usage. Format is: "
                << argv[0] << "<samplerate> <channels (1,2)> <service_topic> <public_topic>" << std::endl;
        exit(1);
    }

    unsigned int sample_rate      = (unsigned int)atoi(argv[1]);
    unsigned int channels         = (unsigned int)atoi(argv[2]);
    std::string service_topic     = argv[3];
    std::string publisher_topic   = argv[4];

    ros::init(argc, argv, publisher_topic + "Capturer");

    VoiceCapturer(sample_rate, channels, service_topic, publisher_topic);

    return 0;
}
