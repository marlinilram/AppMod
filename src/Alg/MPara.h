#ifndef MPara_H
#define MPara_H

#include <iostream>

class MPara
{
public:

    MPara();
    ~MPara(){};

    void setInitParameter();

    void setOptParameter(const int numIntParas, int *intParas, const int numDoubleParas, double *other_paras);

    // opt parameters
    int num_iter;

    // image part parameters
    double BRDF_Light_sfs;
    double Light_Reg;
    double cluster_smooth;
    double Norm_sfs;
    double Norm_smooth;
    double Norm_normalized;
    int num_cluster;

    // geometry part parameters
    double lambd_k_strech;
    double lambd_k_bend;
    double lambd_deform_normal;
    double lambd_vertical_move;
    int deform_max_iter;
};

#endif