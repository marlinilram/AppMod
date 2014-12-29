#include "ModelLight.h"
#include "LOG.h"
#include "GenerateSamples.h"

ModelLight::ModelLight(int numSamples, int sqrtNumSamples, int numChannels)
    :num_samples(numSamples), sqrt_num_samples(sqrtNumSamples), num_channels(numChannels), num_bands(4)
{
    samples = new SAMPLE[num_samples];

    if (!samples)
    {
        LOG::Instance()->OutputError("Unable to allocate space for samples");
    }

    if (!GenerateSamples(sqrtNumSamples, num_bands, samples))
    {
        return;
    }

    sample_matrix.resize(num_samples, 3);
    for (int i = 0; i < num_samples; ++i)
    {
        sample_matrix.row(i) = samples[i].direction;
    }
}

ModelLight::~ModelLight()
{
    delete[] samples;
}

int ModelLight::getNumSamples()
{
    return num_samples;
}

int ModelLight::getSqrtNumSamples()
{
    return sqrt_num_samples;
}

int ModelLight::getNumChannels()
{
    return num_channels;
}

SAMPLE *ModelLight::getSamples()
{
    return samples;
}