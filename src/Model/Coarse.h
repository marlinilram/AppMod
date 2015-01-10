#ifndef Coarse_H
#define Coarse_H

#include "Model.h"
#include "Viewer.h"

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

protected:
    Mat3F model_to_img_trans;
    Normallist model_new_normals;
    Eigen::MatrixX3f light_rec;

    cv::Mat photo;
    cv::Mat mask;
    cv::Mat rho_img;
    std::vector<Eigen::Vector2i> xy_in_mask;

    // opt parameter
    int num_iter;
    int cur_iter;
    double BRDF_Light_sfs;
    double Light_Reg;
    double cluster_smooth;
    double Norm_sfs;
    double Norm_smooth;
    double Norm_normalized;
};

#endif