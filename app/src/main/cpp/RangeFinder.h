//
// Created by sensj on 2017/12/18.
//

#ifndef LLAP_RANGEFINDER_H
#define LLAP_RANGEFINDER_H

//max number of frequency
#include <cstdint>

#define MAX_NUM_FREQS           16
//pi
#define PI                      3.1415926535
//audio sample rate
#define AUDIO_SAMPLE_RATE       48000  //should be the same as in controller, will add later
//default temperature
#define TEMPERATURE             20
//volume
#define VOLUME                  0.20
//cic filter stages
#define CIC_SEC                 4
//cic filter decimation
#define CIC_DEC                 16
//cic filter delay
#define CIC_DELAY               17
//socket buffer length
#define SOCKETBUFLEN            40960
//power threshold
#define POWER_THR               15000
//peak threshold
#define PEAK_THR                220
//dc_trend threshold
#define DC_TREND                0.25

class RangeFinder
{
public:
    RangeFinder( uint32_t inMaxFramesPerSlice, uint32_t inNumFreqs, float inStartFreq, float inFreqInterv );
    ~RangeFinder();


    int16_t*        GetPlayBuffer(uint32_t inSamples);
    int16_t*        GetRecDataBuffer(uint32_t inSamples);
    float         GetDistanceChange(void);


private:
    void            InitBuffer();
    void            GetBaseBand();
    void            RemoveDC();
    float         CalculateDistance();


    uint32_t          mNumFreqs;//number of frequency
    uint32_t          mCurPlayPos;//current play position
    uint32_t          mCurProcPos;//current process position
    uint32_t          mCurRecPos;//current receive position
    uint32_t          mLastCICPos;//last cic filter position
    uint32_t          mBufferSize;//buffer size
    uint32_t          mRecDataSize;//receive data size
    uint32_t          mDecsize;//buffer size after decimation
    float         mFreqInterv;//frequency interval
    float         mSoundSpeed;//sound speed
    float         mFreqs[MAX_NUM_FREQS];//frequency of the ultsound signal
    float         mWaveLength[MAX_NUM_FREQS];//wave length of the ultsound signal

    int16_t*        mPlayBuffer;
    int16_t*        mRecDataBuffer;
    float*        mFRecDataBuffer;
    float*        mSinBuffer[MAX_NUM_FREQS];
    float*        mCosBuffer[MAX_NUM_FREQS];
    float*        mBaseBandReal[MAX_NUM_FREQS];
    float*        mBaseBandImage[MAX_NUM_FREQS];
    float*        mTempBuffer;
    float*        mCICBuffer[MAX_NUM_FREQS][CIC_SEC][2];
    float         mDCValue[2][MAX_NUM_FREQS];
    float         mMaxValue[2][MAX_NUM_FREQS];
    float         mMinValue[2][MAX_NUM_FREQS];
    float         mFreqPower[MAX_NUM_FREQS];
};



#endif //LLAP_RANGEFINDER_H
