/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/11
* 		- File created.
*
* Description:
*	Captures audio from microphone and publishes it to topic.
* 
***********************************************************************************/
#include "VoiceCapturer.h"

#include "ros/ros.h"
#include "roscomm/toggle_operator_voice_capturer.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>
#include <alsa/asoundlib.h>

VoiceCapturer::VoiceCapturer( unsigned int sample_rate, unsigned int channels, std::string service_topic, std::string publisher_topic )
	:channels_( channels )
	,sample_rate_( sample_rate )
    ,service_topic_ ( service_topic )
    ,publisher_topic_ ( publisher_topic )
{
	setEnabled(true);
    initAudio();
    
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService(service_topic+"_"+publisher_topic, &VoiceCapturer::toggleVoiceCapturer, this);
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>(publisher_topic, 1000);

    while (ros::ok())
    {
        if(voice_capturer_on_)
        {
            readBuffer();

            std_msgs::String msg;
            std::stringstream ss;

            std::string bufferstr;
            for(int i = 0; i < buffer_size_; i++)
                bufferstr+=buffer_[i];

            ss << bufferstr;
            msg.data = ss.str();

            chatter_pub.publish(msg);
        }
        ros::spinOnce();
    }
};
//---------------------------------------------------------------------------------
//
//
VoiceCapturer::~VoiceCapturer()
{
	close();
};
//---------------------------------------------------------------------------------
//
//
void VoiceCapturer::setEnabled(bool enable)
{ 
    voice_capturer_on_ = enable; 
};
//---------------------------------------------------------------------------------
//
//
bool VoiceCapturer::isEnabled() 
{ 
    return voice_capturer_on_; 
};
//---------------------------------------------------------------------------------
//
//
bool VoiceCapturer::initAudio()
{
    //ROS_INFO("Audio bit rate is ~%i kbit/s", (sample_rate_ * channels_ * 16) / 1000);

    // Open PCM device for recording (capture).
    rc_ = snd_pcm_open(&handle_, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (rc_ < 0) {
        std::cerr << "Init: cannot open audio device (" << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }
    else
    {
        std::cout << "Audio device opened succesfully." << std::endl;
    }

    // Allocate a hardware parameters object.
    snd_pcm_hw_params_alloca(&params_);

    // Fill it in with default values.
    rc_ = snd_pcm_hw_params_any(handle_, params_);
    if(rc_ < 0) {
        std::cerr << "Init: cannot initialize hardware parameter structure ("
                << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }

    // Set the desired hardware parameters.

    // Interleaved mode
    rc_ = snd_pcm_hw_params_set_access(handle_, params_,
                SND_PCM_ACCESS_RW_INTERLEAVED);
    if(rc_ < 0) {
        std::cerr << "Init: cannot set access type (" << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }

    // Signed 16-bit little-endian format
    rc_ = snd_pcm_hw_params_set_format(handle_, params_,
                                 SND_PCM_FORMAT_S16_LE);
    if(rc_ < 0) {
        std::cerr << "Init: cannot set sample format (" << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }

    // Set channel amount
    rc_ = snd_pcm_hw_params_set_channels(handle_, params_, channels_);
    if(rc_ < 0){
        std::cerr << "Init: cannot set channel count to " << channels_ << " ("
                << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }

    // Set sample rate, e.g. 44100 bits/second sampling rate (CD quality)
    unsigned int actual_rate = sample_rate_;
    rc_ = snd_pcm_hw_params_set_rate_near(handle_, params_, &actual_rate, &dir_);
    if(rc_ < 0){
        std::cerr << "Init: cannot set sample rate to " << sample_rate_ << " ("
                << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }
    if(actual_rate != sample_rate_)
    {
        std::cout << "Init: sample rate does not match requested rate ("
                << sample_rate_ << " requested, " << actual_rate << " acquired)" << std::endl;
    }

    //Set period size
    // frames_ = 32;
    frames_ = 32;
    snd_pcm_hw_params_set_period_size_near(handle_, params_, &frames_, &dir_);

    //Apply the hardware parameters
    rc_ = snd_pcm_hw_params(handle_, params_);
    if(rc_ < 0) {
        std::cerr << "Init: cannot set parameters (" << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }
    else {
        std::cout << "Audio device parameters have been set succesfully" << std::endl;
    }

    //Get the period size
    snd_pcm_hw_params_get_period_size(params_, &frames_, &dir_);
    std::cout << "Init: period size = " << frames_ << " frames_." << std::endl;

    //Display bit size of samples
    int sbits = snd_pcm_hw_params_get_sbits(params_);
    std::cout << "Init: significant bits for linear samples = " << sbits << std::endl;

    buffer_size_ = frames_ * (sbits / 8) * channels_;
    buffer_ = (char*)malloc(buffer_size_);

    //Prepare interface for use
    rc_ = snd_pcm_prepare(handle_);
    if(rc_ < 0)
    {
        std::cerr << "Init: cannot prepare audio interface for use ("
                << snd_strerror(rc_) << ")" << std::endl;
        exit(1);
    }
    else
    {
        std::cout << "Init: Audio device has been prepared for use." << std::endl;
    }

    return true;
};
//---------------------------------------------------------------------------------
//
//
char* VoiceCapturer::readBuffer()
{
	rc_ = snd_pcm_readi(handle_, buffer_, frames_);
    if (rc_ == -EPIPE) {
        // EPIPE means overrun
        std::cerr << "overrun occurred" << std::endl;
        snd_pcm_prepare(handle_);
    } else if (rc_ < 0) {
        std::cerr << "error from read: " << snd_strerror(rc_) << std::endl;
    } else if (rc_ != (int)frames_) {
        std::cerr << "short read, read " << rc_ << "frames" << std::endl;
    }

    return buffer_;
}
//---------------------------------------------------------------------------------
//
//
bool VoiceCapturer::close()
{
	snd_pcm_drain(handle_);
    snd_pcm_close(handle_);
    free(buffer_);

    std::cout << "Audio device has been closed" << std::endl;
    return true;
};
//---------------------------------------------------------------------------------
//
//
bool VoiceCapturer::toggleVoiceCapturer(roscomm::toggle_operator_voice_capturer::Request  &req,
                                 roscomm::toggle_operator_voice_capturer::Response &res )
{
    setEnabled(req.toggle_operator_voice_capturer_request);
    res.toggle_operator_voice_capturer_response = isEnabled();
    ROS_INFO("request: toggle_operator_voice_capturer_request=%d", req.toggle_operator_voice_capturer_request);
    ROS_INFO("sending back response: [%d]", res.toggle_operator_voice_capturer_response);
    return true;
};
