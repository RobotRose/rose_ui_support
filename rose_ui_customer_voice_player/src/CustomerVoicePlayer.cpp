//===============================================================================
// $Copyright: Copyright © 1996-2011 Sioux Embedded Systems B.V. $
// All rights reserved.
//===============================================================================

#define ALSA_PCM_NEW_HW_PARAMS_API

#include <iostream>
#include <alsa/asoundlib.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <roscomm/toggle_customer_voice_capturer.h>

int dir;
int rc;
snd_pcm_t *handle;
snd_pcm_hw_params_t *params;
snd_pcm_uframes_t frames;
unsigned int sample_rate;
unsigned int channels;
std::string input_topic;

ros::Subscriber sub;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    rc = snd_pcm_writei(handle, msg->data.c_str(), frames);
    if (rc == -EPIPE) {
      /* EPIPE means underrun */
//      fprintf(stderr, "underrun occurred\n");
      snd_pcm_prepare(handle);
    } else if (rc < 0) {
      fprintf(stderr,
              "error from writei: %s\n",
              snd_strerror(rc));
    }  else if (rc != (int)frames) {
      fprintf(stderr,
              "short write, write %d frames\n", rc);
    }
}

bool initAudioDevice(unsigned int sample_rate, unsigned int channels)
{
    ROS_INFO("Audio bit rate is ~%i kbit/s", (sample_rate * channels * 16) / 1000);

    // Open PCM device for playback.
    rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
    if (rc < 0) {
        std::cerr << "Init: cannot open audio device (" << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }
    else
    {
        std::cout << "Audio device opened succesfully." << std::endl;
    }

    // Allocate a hardware parameters object.
    snd_pcm_hw_params_alloca(&params);

    // Fill it in with default values.
    rc = snd_pcm_hw_params_any(handle, params);
    if(rc < 0) {
        std::cerr << "Init: cannot initialize hardware parameter structure ("
                << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }

    // Set the desired hardware parameters.

    // Interleaved mode
    rc = snd_pcm_hw_params_set_access(handle, params,
                SND_PCM_ACCESS_RW_INTERLEAVED);
    if(rc < 0) {
        std::cerr << "Init: cannot set access type (" << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }

    // Signed 16-bit little-endian format
    rc = snd_pcm_hw_params_set_format(handle, params,
                                 SND_PCM_FORMAT_S16_LE);
    if(rc < 0) {
        std::cerr << "Init: cannot set sample format (" << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }

    // Set channel amount
    rc = snd_pcm_hw_params_set_channels(handle, params, channels);
    if(rc < 0){
        std::cerr << "Init: cannot set channel count to " << channels << " ("
                << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }

    // Set sample rate, e.g. 44100 bits/second sampling rate (CD quality)
    unsigned int actual_rate = sample_rate;
    rc = snd_pcm_hw_params_set_rate_near(handle, params, &actual_rate, &dir);
    if(rc < 0){
        std::cerr << "Init: cannot set sample rate to " << sample_rate << " ("
                << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }
    if(actual_rate != sample_rate)
    {
        std::cout << "Init: sample rate does not match requested rate ("
                << sample_rate << " requested, " << actual_rate << " acquired)" << std::endl;
    }

    //Set period size
    frames = 32;
    snd_pcm_hw_params_set_period_size_near(handle, params, &frames, &dir);

    //Apply the hardware parameters
    rc = snd_pcm_hw_params(handle, params);
    if(rc < 0) {
        std::cerr << "Init: cannot set parameters (" << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }
    else {
        std::cout << "Audio device parameters have been set succesfully" << std::endl;
    }

    //Get the period size
    snd_pcm_hw_params_get_period_size(params, &frames, &dir);
    std::cout << "Init: period size = " << frames << " frames." << std::endl;

    //Display bit size of samples
    int sbits = snd_pcm_hw_params_get_sbits(params);
    std::cout << "Init: significant bits for linear samples = " << sbits << std::endl;

    //Prepare interface for use
    rc = snd_pcm_prepare(handle);
    if(rc < 0){
        std::cerr << "Init: cannot prepare audio interface for use ("
                << snd_strerror(rc) << ")" << std::endl;
        exit(1);
    }
    else{
        std::cout << "Init: Audio device has been prepared for use." << std::endl;
    }

    return true;
}

bool closeAudioDevice()
{
    snd_pcm_drain(handle);
    snd_pcm_close(handle);

    std::cout << "Audio device has been closed" << std::endl;
    return true;
}

bool start_listening()
{
    ROS_INFO("Start listening");
    ros::NodeHandle n;
    sub = n.subscribe(input_topic, 10, chatterCallback);
//    sub = n.subscribe("OperatorVoice", 1000, chatterCallback); // for testing the operator audio
    return true;
}

bool stop_listening()
{
    ROS_INFO("Stop listening");
    sub.shutdown();
    return true;
}

bool toggle_stream( roscomm::toggle_customer_voice_capturer::Request  &req,
                    roscomm::toggle_customer_voice_capturer::Response &res)
{
    if(req.toggle_customer_voice_capturer_request == true) // == true: then start listening
    {
        res.toggle_customer_voice_capturer_response = true;
        return start_listening();
    }
    else
    {
        res.toggle_customer_voice_capturer_response = false;
        return stop_listening();
    }
}

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        fprintf(stderr,
            "Error: Wrong usage. Format is: %s <samplerate> <channels (1,2)> <input_topic>\n", argv[0]);
        exit(1);
    }
    sample_rate = (unsigned int)atoi(argv[1]);
    channels = (unsigned int)atoi(argv[2]);
    input_topic = argv[3];

    initAudioDevice(sample_rate, channels);

    /*Initiate ROS*/
    ros::init(argc, argv, input_topic + "Player", ros::init_options::AnonymousName);

    ros::NodeHandle n;
    start_listening();
    ros::ServiceServer toggleService = n.advertiseService("toggle_"+input_topic, toggle_stream);

    ros::spin();

    return 0;
}
