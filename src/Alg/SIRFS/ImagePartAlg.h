#ifndef ImagePartAlg_H
#define ImagePartAlg_H

#include <QObject>

#include <math.h>
#include <iostream>

#include <Eigen\Eigen>
#include <Eigen\Sparse>
#include <cv.h>
#include <highgui.h>

class Coarse;
class MPara;
class Viewer;

class ImagePartAlg : public QObject
{
    Q_OBJECT

public:
    ImagePartAlg();
    //ImagePartAlg(Coarse *model, Viewer *model_viewer);
    ~ImagePartAlg();

    double get_time();

public:
	Eigen::VectorXf brightness_sig_chan;
	Eigen::MatrixXf T_coef;
	Eigen::MatrixXf V_coef;

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

    //Eigen::MatrixX3f reweighted_mat;
    Eigen::MatrixX3f light_reweighted_mat;
    Eigen::MatrixX3f rho_d_reweighted_mat;

    Eigen::MatrixX3f normal_debug;
    std::vector<Eigen::Vector3f> pixel_init_normal;

    double cur_min_funval;

    int num_pixels_init;

    std::vector<double> last_iter_output;

    std::vector<int> pixel_cluster_label;

    std::vector<std::vector<int>> I_smooth_adj;

    MPara *m_para;

    //void setRunFunctionType(int functionType);
private slots:
    void computeInitLight(Coarse *model, Viewer *viewer);
    void updateLight(Coarse *model, Viewer *viewer);
	void updateRho(Coarse *model, Viewer *viewer);
    void updateLightSingleChannel(cv::Mat &photo, cv::Mat &mask, cv::Mat &r_img, Coarse *model, Viewer *viewer, cv::Mat &rho_img, Eigen::VectorXf &Light_rec);
    void computeNormal(Coarse *model, Viewer *viewer);
    float sigmoid(float coef, float t);

    void findISmoothAdj(std::vector<std::vector<int>> &I_smooth_adj, std::vector<Eigen::Vector2i> &I_xy_vec);

    void runWholeIter(Coarse *model, Viewer *viewer);

	void solveRenderEqAll(Coarse *model, Viewer *viewer);

    void geodesticNormalMean(std::vector<Eigen::Vector3f> &normal_list, Eigen::Vector3f &new_normal);

    void getNewNormal(Eigen::VectorXf &new_normal, std::vector<std::vector<Eigen::Vector3f>> &face_crsp_normal_lsit);

    void setRhodInitFromKMeans(cv::Mat &rho_img, cv::Mat &photo, std::vector<Eigen::Vector2i> &I_xy_vec);
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