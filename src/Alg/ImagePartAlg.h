#ifndef ImagePartAlg_H
#define ImagePartAlg_H

#include <cv.h>
#include <QObject>

#include <math.h>
#include <iostream>

#include "nlopt.hpp"
#include "Coarse.h"
#include "Viewer.h"
#include "GeometryPartAlg.h"

class ImagePartAlg : public QObject
{
    Q_OBJECT

public:
    ImagePartAlg();
    //ImagePartAlg(Coarse *model, Viewer *model_viewer);
    ~ImagePartAlg();


public:
	Eigen::VectorXf brightness_sig_chan;
	Eigen::MatrixXf T_coef;

	Eigen::MatrixX3f brightness_mat;
	Eigen::MatrixX3f A_mat;
	Eigen::MatrixX3f B_mat;
	Eigen::MatrixX3f C_mat;
	std::vector<std::vector<int>> F_smooth_adj;

	Eigen::MatrixX3f I_mat;
	Eigen::VectorXf intensity_sig_chan;
	Eigen::VectorXf Light_rec_sig_chan;
	Eigen::MatrixX3f S_mat; // samples
	Eigen::VectorXf rho_d_sig_chan;
	Eigen::MatrixX3f rho_d_mat;
	Eigen::Vector3f view; 

    int num_pixels_init;

    std::vector<double> last_iter_output;

    std::vector<int> pixel_cluster_label;

    std::vector<std::vector<int>> I_smooth_adj;

	float lambd_sfs;
	float lambd_smooth;
	float lambd_norm;

    double lambd_BRDF_Light_sfs;
    double lambd_Light_Reg;
    double lambd_cluster_smooth;
    double lambd_norm_sfs;
    double lambd_norm_smooth;
    double lambd_norm_normalized;

    int cur_iter;

    //void setRunFunctionType(int functionType);
private slots:
    void computeInitLight(Coarse *model, Viewer *viewer);
    void updateLight(Coarse *model, Viewer *viewer);
	void updateRho(Coarse *model, Viewer *viewer);
    void updateLightSingleChannel(cv::Mat &photo, cv::Mat &mask, cv::Mat &r_img, Coarse *model, Viewer *viewer, cv::Mat &rho_img, Eigen::VectorXf &Light_rec);
    void computeNormal(Coarse *model, Viewer *viewer);
    float sigmoid(float coef, float t);
	void testNLopt();
    void findISmoothAdj(std::vector<std::vector<int>> &I_smooth_adj, std::vector<Eigen::Vector2i> &I_xy_vec);

    //void runWholeIter()

signals:
    void refreshScreen();

//protected:
//    void run();
//
//private:
//    int function_type;
//
//    Coarse *coarse_model;
//    Viewer *viewer;
};

#endif