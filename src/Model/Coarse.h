#ifndef Coarse_H
#define Coarse_H

#include "Model.h"
#include "GroundTruth.h"
#include "Viewer.h"

class Groundtruth;

class Coarse : public Model
{
public:
    typedef Eigen::Matrix3f Mat3F;

public:
    Coarse(){};
    ~Coarse(){};

    Coarse(const int id, const std::string path, const std::string name);

    void loadS2ITransform();

    void updateVertexRho();
    void updateVertexBrightnessAndColor();
    bool getPixelLightCoeffs(int x, int y, Eigen::VectorXf &light_coeffs, Viewer *viewer, float &winx, float &winy);
    inline Eigen::MatrixX3f getRecoveredLight(){ return light_rec; };
	bool getPixelVisbCoeffs(int x, int y, Eigen::VectorXf &visb_coeffs, Viewer *viewer, float &winx, float &winy, Eigen::Vector3f &cur_normal, Eigen::Vector3f &cur_pos);

    void getCrspFromModelToPhoto(int v_id, float xy_photo[2]);
    void getCrspFromPhotoToRImg(int x, int y, float xy_rimg[2]);
    bool getCrspFaceIdFromPhotoToRImg(int x, int y, int &face_id);
    void findFacesInPhoto(std::vector<int> &faces_in_photo);
    void setModelNewNormal(Eigen::VectorXf &new_face_in_photo_normal, std::vector<int> &faces_in_photo);
	void rhoFromKMeans(int nCluster, Eigen::MatrixX3f &rhos_temp, std::vector<int> &cluster_label);

    inline std::vector<float> *getModelNewNormal(){ return &model_new_normals; };
    inline cv::Mat &getPhoto(){ return photo; };
    inline cv::Mat &getMask(){ return mask; };
    inline cv::Mat &getRhoImg(){ return rho_img; };
    inline Eigen::MatrixX3f &getLightRec(){ return light_rec; };
    inline std::vector<Eigen::Vector2i> &getXYInMask(){ return xy_in_mask; };
    inline cv::Mat &getPhotoPS(){ return photo_ps; };


    void drawNormal();

    void setOptParameter(const int &numIter, const int &numParas, double *other_paras);
    inline int getParaNumIter() { return num_iter; };
    inline double getParaBRDFLightSfS() { return BRDF_Light_sfs; };
    inline double getParaLightReg() { return Light_Reg; };
    inline double getParaClutserSmooth() { return cluster_smooth; };
    inline double getParaNormSfS() { return Norm_sfs; };
    inline double getParaNormSmooth() { return Norm_smooth; };
    inline double getParaNormNormalized() { return Norm_normalized; };
    inline int &getCurIter() { return cur_iter; };
    inline Eigen::Matrix3f &getModelToImgTrans() { return model_to_img_trans; };
    inline int getParaNumCluster() { return num_cluster; };
	inline double getParaKStrech() { return lambd_k_strech; };
	inline double getParaKBend() { return lambd_k_bend; };
	inline double getParaDeformNormal() { return lambd_deform_normal; };
	inline double getParaVerticalMove() { return lambd_vertical_move; };
	inline int getParaMaxDeformIter() { return deform_max_iter; };
    
    inline void setGtModelPtr(Groundtruth *gtModel){gt_model = gtModel;};
    inline Groundtruth *getGtModelPtr(){return gt_model;};

protected:
    Mat3F model_to_img_trans;
    Normallist model_new_normals;
    Eigen::MatrixX3f light_rec;

    cv::Mat photo;
    cv::Mat photo_ps;
    cv::Mat mask;
    cv::Mat rho_img;
    std::vector<Eigen::Vector2i> xy_in_mask;

    // opt parameter
    int num_iter;
    int cur_iter;
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


    Groundtruth *gt_model;
};

#endif