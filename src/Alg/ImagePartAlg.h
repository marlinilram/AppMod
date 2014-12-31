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
	float lambd_sfs;
	float lambd_smooth;
	float lambd_norm;

    //void setRunFunctionType(int functionType);
private slots:
    void computeInitLight(Coarse *model, Viewer *viewer);
    void updateLight(Coarse *model, Viewer *viewer);
    void updateLightSingleChannel(cv::Mat &photo, cv::Mat &mask, cv::Mat &r_img, Coarse *model, Viewer *viewer, cv::Mat &rho_img, Eigen::VectorXf &Light_rec);
    void computeNormal(Coarse *model, Viewer *viewer);
    float sigmoid(float coef, float t);
	void testNLopt();

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