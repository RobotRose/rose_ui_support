/***********************************************************************************
* Copyright: Rose B.V. (2013)
*
* Revision History:
*	Author: Mathijs de Langen
*	Date  : 2013/12/11
* 		- File created.
*
* Description:
*	Captures audio from microphone.
* 
***********************************************************************************/
#ifndef VOICECAPTURER_H
#define VOICECAPTURER_H

#include <iostream>
#include <alsa/asoundlib.h>

#include "roscomm/toggle_operator_voice_capturer.h"

class VoiceCapturer
{
public:
	VoiceCapturer(unsigned int sample_rate, unsigned int channels, std::string service_topic, std::string publisher_topic);
	~VoiceCapturer();

	void setEnabled(bool enable);
	bool isEnabled();

private:
	bool initAudio();
	bool close();

	char* readBuffer();
	bool toggleVoiceCapturer(roscomm::toggle_operator_voice_capturer::Request  &req,
							 roscomm::toggle_operator_voice_capturer::Response &res );

	unsigned int channels_;
	unsigned int sample_rate_;

	std::string service_topic_; 
	std::string publisher_topic_;

	bool voice_capturer_on_;

	int dir_;
	int rc_;
	int buffer_size_;
	snd_pcm_t *handle_;
	snd_pcm_hw_params_t *params_;
	snd_pcm_uframes_t frames_;
	char *buffer_;
};


#endif // VOICECAPTURER_H