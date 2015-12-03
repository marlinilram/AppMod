//////////////////////////////////////////////////////////////////////////////////////////
//	SAMPLE.h
//	Class for a sample on the surface of the unit sphere
//	Downloaded from: www.paulsprojects.net
//	Created:	21st September 2003
//  Modified:   11/28/2014 Lin Ma majcjc@gmail.com
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	

#ifndef SAMPLE_H
#define SAMPLE_H

#include "Eigen\Eigen"
#include <vector>

class SAMPLE
{
public:
    //Spherical polar coords
    double theta;
    double phi;

    //Cartesian direction
    Eigen::Vector3f direction;

    //Values of each SH function at this point
    std::vector<double> shValues;

    SAMPLE():shValues(NULL)
    {}
    ~SAMPLE()
    {}
};

#endif


