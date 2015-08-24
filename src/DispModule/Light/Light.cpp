//////////////////////////////////////////////////////////////////////////////////////////
//	Light.cpp
//	Return the brightness of the light source, given spherical polar direction
//	Downloaded from: www.paulsprojects.net
//	Created:	21st September 2003
//
//	Copyright (c) 2006, Paul Baker
//	Distributed under the New BSD Licence. (See accompanying file License.txt or copy at
//	http://www.paulsprojects.net/NewBSDLicense.txt)
//////////////////////////////////////////////////////////////////////////////////////////	
#include <math.h>
#include "Light.h"

/*double Light(double theta, double phi)
{
    double temp1=sin(2*theta);

    if(temp1<0.0)
        temp1=0.0;

    double temp2=sin(phi/2-M_PI/8);

    if(temp2<0.0)
        temp2=0.0;

    double temp3=8.0*temp1*temp2-6.0;

    if(temp3<0.0)
        temp3=0.0;

    return temp3;
}*/


double Light(double theta, double phi, int chn)
{
    //if (chn == 0) return (theta < 1 * M_PI / 6 && theta > 0 * M_PI / 4 && phi < 8 * M_PI / 4 && phi > 0 * M_PI / 4) ? 1.0 : 0;
    //else if (chn == 1) return (theta < 1 * M_PI / 6 && theta > 0 * M_PI / 4 && phi < 8 * M_PI / 4 && phi > 0 * M_PI / 4) ? 1.0 : 0;
    //else return (theta < 1 * M_PI / 6 && theta > 0 * M_PI / 4 && phi < 8 * M_PI / 4 && phi > 0 * M_PI / 4) ? 1.0 : 0;

    //return 0.1;
    return (theta < 4 * M_PI / 6 && phi < 4 * M_PI / 3 && phi > 2 * M_PI / 3) ? 0.7 : 0;
    //return (theta < M_PI / 6) ? 0.8 : 0;
}