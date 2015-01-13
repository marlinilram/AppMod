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

void ModelLight::loadOutsideLight(std::string file)
{
	std::ifstream f_light(file);
	std::vector<Eigen::Vector3f> light_vec;

	if (f_light)
	{
		while (!f_light.eof())
		{
			std::string line;
			getline(f_light, line);

			std::stringstream parser(line);
			Eigen::Vector3f cur_row;
			parser >> cur_row(0) >> cur_row(1) >> cur_row(2);
			light_vec.push_back(cur_row);
		}

		f_light.close();
	}

	// pass light vec to light matrix
	light_matrix.resize(light_vec.size(), 3);
	for (size_t i = 0; i < light_vec.size(); ++i)
	{
		light_matrix.row(i) = light_vec[i];
	}

}

void ModelLight::loadOutsideSample(std::string file)
{
    std::ifstream f_light(file);
    std::vector<Eigen::Vector3f> sample_vec;

    if (f_light)
    {
        while (!f_light.eof())
        {
            std::string line;
            getline(f_light, line);

            std::stringstream parser(line);
            Eigen::Vector3f cur_row;
            parser >> cur_row(0) >> cur_row(1) >> cur_row(2);
            sample_vec.push_back(cur_row);
        }

        f_light.close();
    }

    // pass light vec to light matrix
    sample_matrix_outside.resize(sample_vec.size(), 3);
    for (size_t i = 0; i < sample_vec.size(); ++i)
    {
        sample_matrix_outside.row(i) = sample_vec[i];
    }
}