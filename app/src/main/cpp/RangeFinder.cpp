//
// Created by sensj on 2017/12/18.
//

#include <cstdlib>
#include <cmath>
#include <cstring>
#include "DSPSplitComplex.h"
#include "RangeFinder.h"
#include "AudioController.h"
#include <complex>


RangeFinder::RangeFinder(uint32_t inMaxFramesPerSlice, uint32_t inNumFreq, float inStartFreq,
                         float inFreqInterv) {
    //Number of frequency
    mNumFreqs = inNumFreq;
    //Buffer size
    mBufferSize = inMaxFramesPerSlice;
    //Frequency interval
    mFreqInterv = inFreqInterv;
    //Receive data size
    mRecDataSize = 4*inMaxFramesPerSlice;
    //Sound speed
    mSoundSpeed = 331.3 + 0.606 * TEMPERATURE;
    //Init buffer
    for(uint32_t i=0; i<MAX_NUM_FREQS; i++){
        mSinBuffer[i]=(float*) calloc(2*inMaxFramesPerSlice, sizeof(float));
        mCosBuffer[i]=(float*) calloc(2*inMaxFramesPerSlice, sizeof(float));

        mFreqs[i]=inStartFreq+i*inFreqInterv;

        mWaveLength[i]=mSoundSpeed/mFreqs[i]*1000; //all distance is in mm

        mBaseBandReal[i]=(float*) calloc(mRecDataSize/CIC_DEC, sizeof(float));
        mBaseBandImage[i]=(float*) calloc(mRecDataSize/CIC_DEC, sizeof(float));
        for(uint32_t k=0;k<CIC_SEC;k++)
        {
            mCICBuffer[i][k][0]=(float*) calloc(mRecDataSize/CIC_DEC+CIC_DELAY, sizeof(float));
            mCICBuffer[i][k][1]=(float*) calloc(mRecDataSize/CIC_DEC+CIC_DELAY, sizeof(float));
        }
    }

    mPlayBuffer = (int16_t*) calloc(2*inMaxFramesPerSlice, sizeof(int16_t));

    mRecDataBuffer = (int16_t*) calloc(mRecDataSize, sizeof(int16_t));
    mFRecDataBuffer = (float*) calloc(mRecDataSize, sizeof(float));
    mTempBuffer = (float*) calloc(mRecDataSize, sizeof(float));
    mCurPlayPos = 0;
    mCurRecPos = 0;
    mCurProcPos= 0;
    mLastCICPos =0;
    mDecsize=0;


    InitBuffer();
}

RangeFinder::~RangeFinder() {
    for (uint32_t i=0;i<mNumFreqs; i++)
    {
        if(mSinBuffer[i]!=NULL)
        {
            free(mSinBuffer[i]);
            mSinBuffer[i]=NULL;
        }
        if(mCosBuffer[i]!=NULL)
        {
            free(mCosBuffer[i]);
            mCosBuffer[i]=NULL;
        }
        if(mBaseBandReal[i]!=NULL)
        {
            free(mBaseBandReal[i]);
            mBaseBandReal[i]=NULL;
        }
        if(mBaseBandImage[i]!=NULL)
        {
            free(mBaseBandImage[i]);
            mBaseBandImage[i]=NULL;
        }
        for(uint32_t k=0;k<CIC_SEC;k++)
        {
            if(mCICBuffer[i][k][0]!=NULL)
            {
                free(mCICBuffer[i][k][0]);
                mCICBuffer[i][k][0]=NULL;
            }
            if(mCICBuffer[i][k][1]!=NULL)
            {
                free(mCICBuffer[i][k][1]);
                mCICBuffer[i][k][1]=NULL;
            }
        }
    }
    if(mPlayBuffer!=NULL)
    {
        free(mPlayBuffer);
        mPlayBuffer= NULL;
    }
    if(mTempBuffer!=NULL)
    {
        free(mTempBuffer);
        mTempBuffer= NULL;
    }

    if(mRecDataBuffer!=NULL)
    {
        free(mRecDataBuffer);
        mRecDataBuffer= NULL;
    }
    if(mFRecDataBuffer!=NULL)
    {
        free(mFRecDataBuffer);
        mFRecDataBuffer= NULL;
    }
}

int16_t *RangeFinder::GetPlayBuffer(uint32_t inSamples) {
    int16_t* playDataPointer = mPlayBuffer + mCurPlayPos;

    mCurPlayPos += inSamples;

    if(mCurPlayPos >=mBufferSize)
        mCurPlayPos =mCurPlayPos -mBufferSize;
    return playDataPointer;
}

int16_t *RangeFinder::GetRecDataBuffer(uint32_t inSamples) {
    int16_t* RecDataPointer = mRecDataBuffer + mCurRecPos;

    mCurRecPos += inSamples;

    if(mCurRecPos >= mRecDataSize) //over flowed RecBuffer
    {
        mCurRecPos=0;
        RecDataPointer = mRecDataBuffer;
    }

    return RecDataPointer;
}


float RangeFinder::GetDistanceChange(void) {
    float distancechange=0;

    //each time we process the data in the RecDataBuffer and clear the mCurRecPos

    //Get base band signal
    GetBaseBand();

    //Remove dcvalue from the baseband signal
    RemoveDC();

    //Calculate distance from the phase change
    distancechange=CalculateDistance();

    return distancechange;
}


void RangeFinder::InitBuffer() {
    for(uint32_t i=0; i<mNumFreqs; i++){
        for(uint32_t n=0;n<mBufferSize*2;n++){
            mCosBuffer[i][n]= (float) cos(2 * PI * n / AUDIO_SAMPLE_RATE * mFreqs[i]);
            mSinBuffer[i][n]= (float) -sin(2 * PI * n / AUDIO_SAMPLE_RATE * mFreqs[i]);
        }
        mDCValue[0][i]=0;
        mMaxValue[0][i]=0;
        mMinValue[0][i]=0;
        mDCValue[1][i]=0;
        mMaxValue[1][i]=0;
        mMinValue[1][i]=0;
    }

    float mTempSample;
    for(uint32_t n=0;n<mBufferSize*2;n++){
        mTempSample=0;
        for(uint32_t i=0; i<mNumFreqs; i++){
            mTempSample+=mCosBuffer[i][n]*VOLUME;
        }
        mPlayBuffer[n]=(int16_t) (mTempSample/mNumFreqs*32767);
    }
}

void RangeFinder::GetBaseBand() {
    uint32_t i,index,decsize,cid;
    decsize=mCurRecPos/CIC_DEC;
    mDecsize=decsize;

    //change data from int to float32

    for(i=0;i<mCurRecPos; i++)
    {
        mFRecDataBuffer[i]= (float) (mRecDataBuffer[i]/32767.0);
    }
//    DebugLog("%f %f %f",mFRecDataBuffer[0],mFRecDataBuffer[1],mFRecDataBuffer[2]);

    for(i=0;i<mNumFreqs; i++)//mNumFreqs
    {
        DSP_vmul(mFRecDataBuffer,mCosBuffer[i]+mCurProcPos,mTempBuffer,mCurRecPos); //multiply the cos
        cid=0;
        //sum CIC_DEC points of data, put into CICbuffer
        memmove(mCICBuffer[i][0][cid],mCICBuffer[i][0][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        index=CIC_DELAY;
        for(uint32_t k=0;k<mCurRecPos;k+=CIC_DEC)
        {
            DSP_sve(mTempBuffer+k,mCICBuffer[i][0][cid]+index,CIC_DEC);
            index++;
        }

        //prepare CIC first level
        memmove(mCICBuffer[i][1][cid],mCICBuffer[i][1][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        //Sliding window sum
        DSP_vswsum(mCICBuffer[i][0][cid],mCICBuffer[i][1][cid]+CIC_DELAY,decsize,CIC_DELAY);
        //prepare CIC second level
        memmove(mCICBuffer[i][2][cid],mCICBuffer[i][2][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        //Sliding window sum
        DSP_vswsum(mCICBuffer[i][1][cid],mCICBuffer[i][2][cid]+CIC_DELAY,decsize,CIC_DELAY);
        //prepare CIC third level
        memmove(mCICBuffer[i][3][cid],mCICBuffer[i][3][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        //Sliding window sum
        DSP_vswsum(mCICBuffer[i][2][cid],mCICBuffer[i][3][cid]+CIC_DELAY,decsize,CIC_DELAY);
        //CIC last level to Baseband
        DSP_vswsum(mCICBuffer[i][3][cid],mBaseBandReal[i],decsize,CIC_DELAY);


        DSP_vmul(mFRecDataBuffer,mSinBuffer[i]+mCurProcPos,mTempBuffer,mCurRecPos); //multiply the sin
        cid=1;
        //sum CIC_DEC points of data, put into CICbuffer
        memmove(mCICBuffer[i][0][cid],mCICBuffer[i][0][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        index=CIC_DELAY;
        for(uint32_t k=0;k<mCurRecPos;k+=CIC_DEC)
        {
            DSP_sve(mTempBuffer+k,mCICBuffer[i][0][cid]+index,CIC_DEC);
            index++;
        }

        //prepare CIC first level
        memmove(mCICBuffer[i][1][cid],mCICBuffer[i][1][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        //Sliding window sum
        DSP_vswsum(mCICBuffer[i][0][cid],mCICBuffer[i][1][cid]+CIC_DELAY,decsize,CIC_DELAY);
        //prepare CIC second level
        memmove(mCICBuffer[i][2][cid],mCICBuffer[i][2][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        //Sliding window sum
        DSP_vswsum(mCICBuffer[i][1][cid],mCICBuffer[i][2][cid]+CIC_DELAY,decsize,CIC_DELAY);
        //prepare CIC third level
        memmove(mCICBuffer[i][3][cid],mCICBuffer[i][3][cid]+mLastCICPos,CIC_DELAY*sizeof(float));
        //Sliding window sum
        DSP_vswsum(mCICBuffer[i][2][cid],mCICBuffer[i][3][cid]+CIC_DELAY,decsize,CIC_DELAY);
        //CIC last level to Baseband
        DSP_vswsum(mCICBuffer[i][3][cid],mBaseBandImage[i],decsize,CIC_DELAY);

    }

    mCurProcPos=mCurProcPos+mCurRecPos;
    if(mCurProcPos >= mBufferSize)
        mCurProcPos= mCurProcPos - mBufferSize;
    mLastCICPos=decsize;
    mCurRecPos=0;
    // For debug --------------------------------------------------------------------------
//    for(int f=0;f<mNumFreqs;f++) {
//
//        float temp_val = 0;
//        float temp_val_ = 0;
//        for (int i = 0; i < mDecsize; ++i) {
//            temp_val += mBaseBandReal[f][i] * mBaseBandReal[f][i];
//            temp_val_ += mBaseBandImage[f][i] * mBaseBandImage[f][i];
//        }
//        DebugLog("Real: %f",temp_val);
//        DebugLog("Image: %f",temp_val_);
//
//    }

//    mCurProcPos = mCurProcPos + mCurRecPos;
//    if (mCurProcPos >= mBufferSize)
//        mCurProcPos = mCurProcPos - mBufferSize;
//    mLastCICPos = decsize;
//    mCurRecPos = 0;
}

void RangeFinder::RemoveDC() {
    int f,i;
    float tempdata[4096],tempdata2[4096],temp_val;
    float vsum,dsum,max_valr,min_valr,max_vali,min_vali;
    if(mDecsize>4096)
        return;

    //'Levd' algorithm to calculate the DC value;
    for(f=0;f<mNumFreqs;f++)
    {
        vsum=0;
        dsum=0;
        //real part
        DSP_maxv(mBaseBandReal[f],&max_valr,mDecsize);
        DSP_minv(mBaseBandReal[f],&min_valr,mDecsize);
        //getvariance,first remove the first value
        temp_val=-mBaseBandReal[f][0];
        DSP_vsadd(mBaseBandReal[f],&temp_val,tempdata,mDecsize);
        DSP_sve(tempdata,&temp_val,mDecsize);
        dsum=dsum+fabs(temp_val)/mDecsize;
        DSP_vsq(tempdata,tempdata2,mDecsize);
        DSP_sve(tempdata2,&temp_val,mDecsize);
        vsum=vsum+fabs(temp_val)/mDecsize;

        //imag part
        DSP_maxv(mBaseBandImage[f],&max_vali,mDecsize);
        DSP_minv(mBaseBandImage[f],&min_vali,mDecsize);
        //getvariance,first remove the first value
        temp_val=-mBaseBandImage[f][0];
        DSP_vsadd(mBaseBandImage[f],&temp_val,tempdata,mDecsize);
        DSP_sve(tempdata,&temp_val,mDecsize);
        dsum=dsum+fabs(temp_val)/mDecsize;
        DSP_vsq(tempdata,tempdata2,mDecsize);
        DSP_sve(tempdata2,&temp_val,mDecsize);
        vsum=vsum+fabs(temp_val)/mDecsize;

        mFreqPower[f]=(vsum+dsum*dsum);///fabs(vsum-dsum*dsum)*vsum;

        //Get DC estimation
        if(mFreqPower[f]>POWER_THR)
        {
            if ( max_valr > mMaxValue[0][f] ||
                 (max_valr > mMinValue[0][f]+PEAK_THR &&
                  (mMaxValue[0][f]-mMinValue[0][f]) > PEAK_THR*4) )
            {
                mMaxValue[0][f]=max_valr;
            }

            if ( min_valr < mMinValue[0][f] ||
                 (min_valr < mMaxValue[0][f]-PEAK_THR &&
                  (mMaxValue[0][f]-mMinValue[0][f]) > PEAK_THR*4) )
            {
                mMinValue[0][f]=min_valr;
            }

            if ( max_vali > mMaxValue[1][f] ||
                 (max_vali > mMinValue[1][f]+PEAK_THR &&
                  (mMaxValue[1][f]-mMinValue[1][f]) > PEAK_THR*4) )
            {
                mMaxValue[1][f]=max_vali;
            }

            if ( min_vali < mMinValue[1][f] ||
                 (min_vali < mMaxValue[1][f]-PEAK_THR &&
                  (mMaxValue[1][f]-mMinValue[1][f]) > PEAK_THR*4) )
            {
                mMinValue[1][f]=min_vali;
            }


            if ( (mMaxValue[0][f]-mMinValue[0][f]) > PEAK_THR &&
                 (mMaxValue[1][f]-mMinValue[1][f]) > PEAK_THR )
            {
                for(i=0;i<=1;i++)
                    mDCValue[i][f]=(1-DC_TREND)*mDCValue[i][f]+
                                   (mMinValue[i][f]+mMaxValue[i][f])/2*DC_TREND;
            }

        }

        //remove DC
        for(i=0;i<mDecsize;i++)
        {
            mBaseBandReal[f][i]=mBaseBandReal[f][i]-mDCValue[0][f];
            mBaseBandImage[f][i]=mBaseBandImage[f][i]-mDCValue[1][f];
        }

    }
}


float RangeFinder::CalculateDistance() {
    float distance=0;
    DSPSplitComplex tempcomplex;
    float tempdata[4096],tempdata2[4096],tempdata3[4096],temp_val;
    float phasedata[MAX_NUM_FREQS][4096];
    int     ignorefreq[MAX_NUM_FREQS];


    if(mDecsize>4096)
        return 0;

    for(int f=0;f<mNumFreqs;f++)
    {
        ignorefreq[f]=0;
        //get complex number
        tempcomplex.realp=mBaseBandReal[f];
        tempcomplex.imagp=mBaseBandImage[f];

        //get magnitude

        temp_val = 0;
        for (int i = 0; i < mDecsize; ++i) {
            tempdata[i]=tempcomplex.realp[i] * tempcomplex.realp[i] + tempcomplex.imagp[i] * tempcomplex.imagp[i];
            temp_val += tempdata[i];
        }
//        DebugLog("mDecsize: %d",int(mDecsize));
//        DebugLog("POWER: %f",temp_val/mDecsize);
        if(temp_val/mDecsize>POWER_THR) //only calculate the high power vectors
        {
            for (int i = 0; i < mDecsize; i++) {
                phasedata[f][i]= (float) atan2(tempcomplex.imagp[i], tempcomplex.realp[i]);
            }
            //phase unwarp
            for(int i=1;i<mDecsize;i++)
            {
                while(phasedata[f][i]-phasedata[f][i-1]>PI)
                    phasedata[f][i]= (float) (phasedata[f][i] - 2 * PI);
                while(phasedata[f][i]-phasedata[f][i-1]<-PI)
                    phasedata[f][i]= (float) (phasedata[f][i] + 2 * PI);
            }
            if(fabs(phasedata[f][mDecsize-1]-phasedata[f][0])>PI/4)
            {
                for(int i=0;i<=1;i++)
                    mDCValue[i][f]= (float) ((1 - DC_TREND * 2) * mDCValue[i][f] +
                                                       (mMinValue[i][f]+mMaxValue[i][f])/2*DC_TREND*2);
            }

            //prepare linear regression
            //remove start phase
            temp_val=-phasedata[f][0];
            for (int i = 0; i < mDecsize; i++) {
                tempdata[i] = phasedata[f][i] + temp_val;
            }
            //divide the constants
            temp_val=2*PI/mWaveLength[f];
            for (int i = 0; i < mDecsize; i++) {
                phasedata[f][i] = tempdata[i] / temp_val;
            }
        }
        else //ignore the low power vectors
        {
            ignorefreq[f]=1;
        }

    }

    //linear regression
    for(int i=0;i<mDecsize;i++)
        tempdata2[i]=i;
    float sumxy=0;
    float sumy=0;
    int     numfreqused=0;
    for(int f=0;f<mNumFreqs;f++)
    {
        if(ignorefreq[f])
        {
            continue;
        }

        numfreqused++;

        temp_val = 0;
        for (int i = 0; i < mDecsize; i++) {
            tempdata[i] = phasedata[f][i] * tempdata2[i];
            temp_val += tempdata[i];
        }


        sumxy+=temp_val;
        temp_val = 0;
        for (int i = 0; i < mDecsize; i++) {
            temp_val += phasedata[f][i];
        }

        sumy+=temp_val;

    }
    if(numfreqused==0)
    {
        distance=0;
//        DebugLog("numfrequesd = 0");
        return distance;
    }

    float deltax=deltax=mNumFreqs*((mDecsize-1)*mDecsize*(2*mDecsize-1)/6-(mDecsize-1)*mDecsize*(mDecsize-1)/4);
    float delta= (float) ((sumxy - sumy * (mDecsize - 1) / 2.0) / deltax * mNumFreqs / numfreqused);

    float varsum=0;
    float var_val[MAX_NUM_FREQS];
    for(int i=0;i<mDecsize;i++)
        tempdata2[i]=i*delta;

    //get variance of each freq;
    for(int f=0;f<mNumFreqs;f++)
    {   var_val[f]=0;
        if(ignorefreq[f])
        {
            continue;
        }
        for (int i = 0; i < mDecsize; i++) {
            tempdata[i] = phasedata[f][i] - tempdata2[i];
            tempdata3[i] = tempdata[i] * tempdata[i];
            var_val[f] += tempdata3[i];
        }
        varsum+=var_val[f];
    }
    varsum=varsum/numfreqused;
    for(int f=0;f<mNumFreqs;f++)
    {
        if(ignorefreq[f])
        {
            continue;
        }
        if(var_val[f]>varsum)
            ignorefreq[f]=1;
    }

    //linear regression
    for(int i=0;i<mDecsize;i++)
        tempdata2[i]=i;

    sumxy=0;
    sumy=0;
    numfreqused=0;
    for(int f=0;f<mNumFreqs;f++)
    {
        if(ignorefreq[f])
        {
            continue;
        }

        numfreqused++;

        temp_val = 0;
        for (int i = 0; i < mDecsize; i++) {
            tempdata[i] = phasedata[f][i] * tempdata2[i];
            temp_val += tempdata[i];
        }

        sumxy+=temp_val;

        temp_val = 0;

        for (int i = 0; i < mDecsize; i++) {
            temp_val += phasedata[f][i];
        }

        sumy+=temp_val;

    }
    if(numfreqused==0)
    {
        distance=0;
        return distance;
    }

    delta= (float) ((sumxy - sumy * (mDecsize - 1) / 2.0) / deltax * mNumFreqs / numfreqused);

    distance=-delta*mDecsize/2;
    return distance;
}
