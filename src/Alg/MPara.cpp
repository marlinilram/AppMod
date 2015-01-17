#include "MPara.h"

MPara::MPara()
{
    // opt parameters
    num_iter = 1;

    // image part parameters
    BRDF_Light_sfs = 1;
    Light_Reg = 0.1;
    rho_d_smooth = 0.01;
    rho_s_r = 10;
    cluster_smooth = 0.3;
    Norm_sfs = 1;
    Norm_smooth = 0;
    Norm_normalized = 0.5;
    Norm_prior = 0.3;
    num_cluster = 1;

    // geometry part parameters
    lambd_k_strech = 10;
    lambd_k_bend = 15;
    lambd_deform_normal = 25;
    lambd_vertical_move = 1;
    deform_max_iter = 20;
}

void MPara::setInitParameter()
{

}

void MPara::setOptParameter(const int numIntParas, int *intParas, const int numDoubleParas, double *doubleParas)
{
    num_iter = intParas[0];
    deform_max_iter = intParas[1];
    num_cluster = intParas[2];

    BRDF_Light_sfs = doubleParas[0];
    std::cout<<"BRDF_Light_sfs:\t"<<BRDF_Light_sfs<<"\n";
    Light_Reg = doubleParas[1];
    std::cout<<"Light_Reg:\t"<<Light_Reg<<"\n";

    rho_d_smooth = doubleParas[10];
    std::cout<<"rho_d_smooth:\t"<<rho_d_smooth<<"\n";
    rho_s_r = doubleParas[11];
    std::cout<<"rho_s_r:\t"<<rho_s_r<<"\n";

    cluster_smooth = doubleParas[2];
    std::cout<<"cluster_smooth:\t"<<cluster_smooth<<"\n";
    Norm_sfs = doubleParas[3];
    std::cout<<"Norm_sfs:\t"<<Norm_sfs<<"\n";
    Norm_smooth = doubleParas[4];
    std::cout<<"Norm_smooth:\t"<<Norm_smooth<<"\n";
    Norm_normalized = doubleParas[5];
    std::cout<<"Norm_normalized:\t"<<Norm_normalized<<"\n";

    Norm_prior = doubleParas[12];
    std::cout<<"Norm_prior:\t"<<Norm_prior<<"\n";

    lambd_k_strech = doubleParas[6];
    std::cout<<"K Strech:\t"<<lambd_k_strech<<"\n";
    lambd_k_bend = doubleParas[7];
    std::cout<<"K Bend:\t"<<lambd_k_bend<<"\n";
    lambd_deform_normal = doubleParas[8];
    std::cout<<"Deform Normal:\t"<<lambd_deform_normal<<"\n";
    lambd_vertical_move = doubleParas[9];
    std::cout<<"Vertical Move:\t"<<lambd_vertical_move<<"\n";
    
}