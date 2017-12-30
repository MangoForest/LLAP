//
// Created by sensj on 2017/12/18.
//

#include "AudioController.h"
#include <sys/time.h>
#include <cstring>
#include <cmath>
#include <time.h>
#include <cstdlib>
#include <unistd.h>
#include <thread>

jmethodID gOnNativeID;
JNIEnv* genv;
jobject mObject;
jclass mClass;
JavaVM* gs_jvm;

struct CallbackData {
    RangeFinder *rangeFinder;
    uint64_t mtime;
    uint64_t mUIUpdateTime;
    float distance;
    float distancechange;

    CallbackData() : rangeFinder(NULL), mtime(0), mUIUpdateTime(0), distance(0), distancechange(0){}
} cd;

int64_t getTimeNsec() {
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t) now.tv_sec * 1000000000LL + now.tv_nsec;
}


bool AudioController::performRender(void *__unused clientdata, short int *audioInputOutput,
                                    int inNumberFrames, int __unused samplerate) {
    CallbackData *cd = (CallbackData *) clientdata;

//    float distancechange;
    uint64_t startTime = (uint64_t) getTimeNsec();
//    DebugLog("start   %d",int(startTime/1e6));
//    DebugLog("%d %d %d", int(audioInputOutput[0]),int(audioInputOutput[1]),int(audioInputOutput[2]));




    memcpy((void *) cd->rangeFinder->GetRecDataBuffer(inNumberFrames), (void *) audioInputOutput,
           sizeof(int16_t) * inNumberFrames);

    cd->distancechange = cd->rangeFinder->GetDistanceChange();

    cd->distance = (float) (cd->distance + cd->distancechange * SPEED_ADJ);

    if (cd->distance < 0) {
        cd->distance = 0;
    }
    if (cd->distance > 500) {
        cd->distance = 500;
    }



    memcpy((void *) audioInputOutput, (void *) cd->rangeFinder->GetPlayBuffer(inNumberFrames),
           sizeof(int16_t) * inNumberFrames);

    cd->mtime = startTime;

    if (fabs(cd->distancechange) > 0.06 && (startTime - cd->mUIUpdateTime) / 1.0e6 > 10) {
//        DebugLog("distance: %f", cd->distance* SPEED_ADJ);
//        env->CallVoidMethod(instance,method,distancechange);
//        cd->audioController->env->CallVoidMethod(cd->audioController->instance,cd->audioController->method,int(cd->distance));
//            [[NSNotificationCenter defaultCenter] postNotificationName:@"AudioDisUpdate" object:nil];
//        cd->audioController->audiodistance=cd->distance;
//        DebugLog("Distance: %f", cd->distance);
//        genv->CallVoidMethod(mObject,gOnNativeID,400);
        JNIEnv *env;
        if((*gs_jvm).AttachCurrentThread(&env, NULL)<0){
        } else{
            (*env).CallVoidMethod(mObject,gOnNativeID,int(cd->distance));
        }
        cd->mUIUpdateTime = startTime;
    }

    return true;
}


void AudioController::init() {
    DebugLog("init()");
    setUpAudio();
}

void AudioController::setUpAudio() {
    DebugLog("setUpAudio");
    _myRangeFinder = new RangeFinder(MAX_FRAME_SIZE, NUM_FREQ, START_FREQ, FREQ_INTERVAL);
    cd.rangeFinder = _myRangeFinder;
    SuperpoweredCPU::setSustainedPerformanceMode(true);
    new SuperpoweredAndroidAudioIO(AUDIO_SAMPLE_RATE, MAX_FRAME_SIZE , true, true, performRender,
                                   &cd, -1, SL_ANDROID_STREAM_MEDIA, MAX_FRAME_SIZE *2);

}

extern "C"
JNIEXPORT void JNICALL
Java_cn_sencs_llap_MainActivity_Begin(JNIEnv *env, jobject instance) {
    jclass clazz = (*env).FindClass("cn/sencs/llap/MainActivity");
    if(clazz==NULL)
    {
        DebugLog("clazz IS NULL................");
        return;
    }

    mObject = (jobject)(*env).NewGlobalRef(instance);
    gOnNativeID = (*env).GetMethodID(clazz, "refresh", "(I)V");
    if(gOnNativeID==NULL)
    {
        DebugLog("gOnNativeID IS NULL................");
        return;
    }

    (*env).GetJavaVM(&gs_jvm);
    genv = env;
    AudioController audioController;

    audioController.init();

}



