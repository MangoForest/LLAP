//
// Created by sensj on 2017/12/18.
//

#ifndef MYAPPLICATION_DSPSPLITCOMPLEX_H
#define MYAPPLICATION_DSPSPLITCOMPLEX_H


struct DSPSplitComplex {
    float*realp;
    float*imagp;
};

void DSP_maxv(float*A, float*C, int N)
{
    *C=-INFINITY;
    for (int i = 0; i < N; ++i) {
        if(*C<A[i])
            *C=A[i];
    }
}

void DSP_minv(float*A,float*C,int N)
{
    *C=INFINITY;
    for (int i = 0; i < N; ++i) {
        if(*C>A[i])
            *C=A[i];
    }
}


void DSP_vsadd(float*A,float *B,float*C,int N)
{
    for (int i = 0; i < N; ++i) {
        C[i]=A[i]+*B;
    }
}

void DSP_sve(float*A,float*C,int N)
{
    *C=0;
    for (int i = 0; i < N; ++i) {
        *C+=A[i];
    }
}

void DSP_vsq(float*A,float*C,int N)
{
    for (int i = 0; i < N; ++i) {
        C[i]=A[i]*A[i];
    }
}

void DSP_vmul(float*A,float*B,float*C,int N)
{
    for (int i = 0; i < N; ++i) {
        C[i]=A[i]*B[i];
    }
}

void DSP_vswsum(float*A,float*C,int N,int P){
    C[0]=0;
    for (int i = 0; i < P; ++i) {
        C[0]+=A[i];
    }
    for (int i = 1; i < N; ++i) {
        C[i] = C[(i-1)] - A[(i-1)]+A[(i+P-1)];
    }
}

#endif //MYAPPLICATION_DSPSPLITCOMPLEX_H
