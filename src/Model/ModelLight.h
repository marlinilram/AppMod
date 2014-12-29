#ifndef ModelLight_H
#define ModelLight_H

#include "SAMPLE.h"

class ModelLight
{
public:
    ModelLight(int numSamples, int sqrtNumSamples, int numChannels);
    ~ModelLight();

    int getNumSamples();
    int getSqrtNumSamples();
    int getNumChannels();
    SAMPLE *getSamples();
    inline Eigen::MatrixX3f &getSampleMatrix(){ return sample_matrix; };

private:
    int num_samples;
    int sqrt_num_samples;
    int num_channels;
    int num_bands;
    SAMPLE *samples;
    Eigen::MatrixX3f sample_matrix;
};

#endif