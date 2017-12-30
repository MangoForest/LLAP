//
// Created by sensj on 2017/12/18.
//

#ifndef MYAPPLICATION_AUDIOCONTROLLER_H
#define MYAPPLICATION_AUDIOCONTROLLER_H

#include <AndroidIO/SuperpoweredAndroidAudioIO.h>
#include <SuperpoweredSimple.h>
#include <SuperpoweredCPU.h>
#include <SLES/OpenSLES.h>
#include <SLES/OpenSLES_AndroidConfiguration.h>
#include <android/log.h>
#include "RangeFinder.h"
#include <android/log.h>
#include <jni.h>
//Record sample rate
#define AUDIO_SAMPLE_RATE   48000
//Start audio frequency
#define START_FREQ          17500.0
//Frequency interval
#define FREQ_INTERVAL       350.0
//Number of frequency
#define NUM_FREQ            10

//Number of frame size
#define MAX_FRAME_SIZE      1920

//Speed adjust
#define SPEED_ADJ           1.1
#define LOG_TAG    "LLAP"
#define DebugLog(...) __android_log_print(ANDROID_LOG_INFO,LOG_TAG,__VA_ARGS__)


class AudioController {
public:
    RangeFinder*_myRangeFinder;
    void init();
    void setUpAudio();
    static bool performRender(void * __unused clientdata, short int *audioInputOutput, int numberOfSamples, int __unused samplerate);
};


#endif //MYAPPLICATION_AUDIOCONTROLLER_H
