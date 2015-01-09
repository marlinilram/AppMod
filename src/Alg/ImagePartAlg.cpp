#include "ImagePartAlg.h"

ImagePartAlg::ImagePartAlg()
{
    //function_type = -1;
    //coarse_model = nullptr;
    //viewer = nullptr;
    last_iter_output.clear();
}

//ImagePartAlg::ImagePartAlg(Coarse *model, Viewer *model_viewer)
//    :coarse_model(model), viewer(model_viewer)
//{
//    ImagePartAlg();
//}

ImagePartAlg::~ImagePartAlg()
{

}

//void ImagePartAlg::run()
//{
//    if (function_type == 0)
//        computeInitLight(coarse_model, viewer);
//    else if (function_type == 1)
//        updateLight(coarse_model, viewer);
//    else if (function_type == 2)
//        computeNormal(coarse_model, viewer);
//    else return;
//}

//void ImagePartAlg::setRunFunctionType(int functionType)
//{
//    function_type = functionType;
//}

double funcSFSLight(const std::vector<double> &L, std::vector<double> &grad, void *data_ptr)
{
    // reinterpret data_ptr
    ImagePartAlg *img_alg_data_ptr = reinterpret_cast<ImagePartAlg *>(data_ptr);
    size_t n_dim = L.size();

    // here we compute the value of objective function given the current L
    std::vector<double> L_temp = L;
    Eigen::Map<Eigen::VectorXd>cur_L(&L_temp[0], n_dim);

    Eigen::VectorXf &brightness = img_alg_data_ptr->brightness_sig_chan;
    Eigen::MatrixXf &T_coeff = img_alg_data_ptr->T_coef;

    Eigen::VectorXf T_L = T_coeff*cur_L.cast<float>();
    Eigen::VectorXf bright_minus_TL = brightness - T_L;

    //std::cout<<brightness<<"\n";

    if (grad.size() != 0)
    {
        // here we need to set grad to grad vector
        for (size_t i = 0; i < grad.size(); ++i)
        {
            grad[i] = 2*cur_L(i) - 2*(bright_minus_TL.dot(T_coeff.col(i)));
        }
    }

    //std::cout<<cur_L.squaredNorm()+bright_minus_TL.squaredNorm()<<"\n";

    return cur_L.squaredNorm()+bright_minus_TL.squaredNorm();

}

double funcSFSNormal(const std::vector<double> &N, std::vector<double> &grad, void *data_ptr)
{
    // reinterpret data_ptr
    ImagePartAlg *img_alg_data_ptr = reinterpret_cast<ImagePartAlg *>(data_ptr);
    size_t n_dim = N.size();
    size_t n_tuple = N.size()/3;

    // here we compute the value of objective function given the current L
    std::vector<double> N_temp = N;
    Eigen::Map<Eigen::MatrixXd>cur_N_tmep(&N_temp[0], 3, n_tuple);
    Eigen::MatrixX3f cur_N = (cur_N_tmep.transpose()).cast<float>();// change cur_N to form n*3
    // data stored in under rule
    // each col means a channel
    // each row means a pixel
    Eigen::MatrixX3f &brightness = img_alg_data_ptr->brightness_mat;
    Eigen::MatrixX3f &A = img_alg_data_ptr->A_mat;
    Eigen::MatrixX3f &B = img_alg_data_ptr->B_mat;
    Eigen::MatrixX3f &C = img_alg_data_ptr->C_mat;
    std::vector<std::vector<int>> &F_smooth_adj = img_alg_data_ptr->F_smooth_adj;
    float lambd_sfs = img_alg_data_ptr->lambd_sfs;
    float lambd_smooth = img_alg_data_ptr->lambd_smooth;
    float lambd_norm = img_alg_data_ptr->lambd_norm;

    double func_val = 0;

    // sfs energy
    for (int i = 0; i < 3; ++i)
    {
        //Eigen::VectorXf sfs_ch = brightness.col(i) - (A.col(i).array()*cur_N.col(0).array()).matrix();
        //Eigen::VectorXf sfs_temp = (A.col(i).array()*cur_N.col(0).array() 
        //	+ B.col(i).array()*cur_N.col(1).array() 
        //	+ C.col(i).array()*cur_N.col(2).array()).matrix();

        Eigen::VectorXf sfs_ch = brightness.col(i) - (A.col(i).array()*cur_N.col(0).array() 
            + B.col(i).array()*cur_N.col(1).array() 
            + C.col(i).array()*cur_N.col(2).array()).matrix();

        func_val += lambd_sfs*sfs_ch.squaredNorm();
        //std::cout<<sfs_temp.squaredNorm()<<"\n";
    }

    // smooth energy
    for (size_t i = 0; i < n_tuple; ++i)
    {
        std::vector<int> &cur_f_smooth_adj = F_smooth_adj[i];

        for (size_t j = 0; j < cur_f_smooth_adj.size(); ++j)
        {
            func_val += lambd_smooth*((cur_N.row(i) - cur_N.row(cur_f_smooth_adj[j])).squaredNorm());
        }
    }

    // normalization energy
    for (size_t i = 0; i < n_tuple; ++i)
    {
        float term_val_temp = cur_N.row(i).squaredNorm()-1;
        func_val += lambd_norm*term_val_temp*term_val_temp;
    }

    if (grad.size() != 0)
    {
        for (size_t i = 0; i < grad.size()/3; ++i)
        {

            // sfs grad
            Eigen::Vector3f cur_brightness = brightness.row(i);			
            Eigen::Vector3f cur_n = cur_N.row(i);
            Eigen::Matrix3f cur_ABC;
            cur_ABC.col(0) = A.row(i);
            cur_ABC.col(1) = B.row(i);
            cur_ABC.col(2) = C.row(i);

            grad[3*i+0] = lambd_sfs*(-2*(cur_brightness - cur_ABC*cur_n).dot(cur_ABC.col(0)));
            grad[3*i+1] = lambd_sfs*(-2*(cur_brightness - cur_ABC*cur_n).dot(cur_ABC.col(1)));
            grad[3*i+2] = lambd_sfs*(-2*(cur_brightness - cur_ABC*cur_n).dot(cur_ABC.col(2)));

            // smooth grad
            std::vector<int> &cur_f_smooth_adj = F_smooth_adj[i];
            for (size_t j = 0; j < cur_f_smooth_adj.size(); ++j)
            {
                Eigen::Vector3f cur_n_adj = cur_N.row(cur_f_smooth_adj[j]);
                grad[3*i+0] += lambd_smooth*2*(cur_n(0)-cur_n_adj(0));
                grad[3*i+1] += lambd_smooth*2*(cur_n(1)-cur_n_adj(1));
                grad[3*i+2] += lambd_smooth*2*(cur_n(2)-cur_n_adj(2));
            }

            // normalization grad
            grad[3*i+0] += lambd_norm*2*(cur_n.squaredNorm()-1)*2*cur_n(0);
            grad[3*i+1] += lambd_norm*2*(cur_n.squaredNorm()-1)*2*cur_n(1);
            grad[3*i+2] += lambd_norm*2*(cur_n.squaredNorm()-1)*2*cur_n(2);
        }
    }


    return func_val;
}

double funcSFSRho(const std::vector<double> &Rho, std::vector<double> &grad, void *data_ptr)
{
    ImagePartAlg *img_alg_data_ptr = reinterpret_cast<ImagePartAlg *>(data_ptr);

    // we set Cx, Cy and Cz and n in the first 4 variables

    // compute value of objective function given current rhos

    size_t n_dim = Rho.size();
    Eigen::VectorXf &intensities = img_alg_data_ptr->intensity_sig_chan;
    Eigen::VectorXf &Light_rec = img_alg_data_ptr->Light_rec_sig_chan;
    Eigen::MatrixXf &T_coeff = img_alg_data_ptr->T_coef;
    Eigen::MatrixX3f &S = img_alg_data_ptr->S_mat;
    Eigen::Vector3f &view = img_alg_data_ptr->view;
    Eigen::VectorXf &rho_d_last_kmeans = img_alg_data_ptr->rho_d_sig_chan;

    std::vector<double> rho_temp = Rho;
    //Eigen::Vector4f rho_s(Rho[0], Rho[1], Rho[2], Rho[3]);
    Eigen::Map<Eigen::VectorXd>rho_d_temp(&rho_temp[4], n_dim-4);
    Eigen::VectorXf rho_d = rho_d_temp.cast<float>();

    Eigen::VectorXf rho_d_T_L = ((T_coeff*Light_rec).array()*rho_d.array()).matrix();

    // {Cx*viewx, Cy*viewy, Cz*viewz}
    Eigen::Vector3f C_view(rho_temp[0]*view(0), rho_temp[1]*view(1), rho_temp[2]*view(2));
    Eigen::VectorXf S_C_view = ((S*C_view).array()<0).select(0, S*C_view); // we shouldn't consider those v*s < 0

    Eigen::VectorXf rho_s = (S_C_view.array().pow(rho_temp[3])).matrix();
    Eigen::VectorXf rho_s_T_L = T_coeff*((rho_s.array()*Light_rec.array()).matrix());

    Eigen::VectorXf rho_s_log = (((S_C_view.array()<=0).select(1, S_C_view)).array().log()).matrix();
    Eigen::VectorXf rho_s_log_T_L = T_coeff*(rho_s.array()*rho_s_log.array()*Light_rec.array()).matrix();

    Eigen::VectorXf rho_s_exp_n_1 = float(rho_temp[3])*(S_C_view.array().pow(rho_temp[3]-1)).matrix();
    Eigen::VectorXf rho_s_exp_n_1_T_L_x = T_coeff*(rho_s_exp_n_1.array()*Light_rec.array()*(view(0)*S.col(0)).array()).matrix();
    Eigen::VectorXf rho_s_exp_n_1_T_L_y = T_coeff*(rho_s_exp_n_1.array()*Light_rec.array()*(view(1)*S.col(1)).array()).matrix();
    Eigen::VectorXf rho_s_exp_n_1_T_L_z = T_coeff*(rho_s_exp_n_1.array()*Light_rec.array()*(view(2)*S.col(2)).array()).matrix();

    // rho_s parsimony
    double rho_s_pars = 0;
    double lambd_rho_s_pars = 10.0;
    rho_s_pars = (rho_d-rho_d_last_kmeans).squaredNorm();
    //double pars_band = 0.1;
    //size_t n_rho_d = Rho.size() - 4;
    //for (size_t i = 0; i < n_rho_d; ++i)
    //{
    //	//for (size_t j = i+1; j < n_rho_d; ++j)
    //	{
    //		rho_s_pars += exp(-(Rho[i+4]-rho_d_last_kmeans(i))*(Rho[i+4]-rho_d_last_kmeans(i))/(4*pars_band*pars_band));
    //	}
    //}
    //rho_s_pars += n_rho_d;
    //rho_s_pars = -log(rho_s_pars/(n_rho_d*2*pars_band*1.7725)); // sqrt(PI) = 1.7725

    if (grad.size() != 0)
    {
        // for every rho_d 
        for (size_t i = 4; i < grad.size(); ++i)
        {
            grad[i] = -2*(intensities(i-4)-rho_d_T_L(i-4)-rho_s_T_L(i-4))*(T_coeff.row(i-4).dot(Light_rec));

            // gradient of rho_s_pars
            double rho_d_pars_grad = 0;
            rho_d_pars_grad = 2*(Rho[i] - rho_d_last_kmeans(i-4));
            //for (size_t k = 0; k < n_rho_d; ++k)
            //{
            //	rho_d_pars_grad += exp(-(Rho[i]-rho_d_last_kmeans(i-4))*(Rho[i]-rho_d_last_kmeans(i-4))/(4*pars_band*pars_band))*(Rho[i]-rho_d_last_kmeans(i-4));
            //}
            //rho_d_pars_grad = (-2*rho_d_pars_grad/(n_rho_d*2*pars_band*1.7725)/(4*pars_band*pars_band))/rho_s_pars;

            grad[i] += lambd_rho_s_pars*rho_d_pars_grad;
        }

        // for rho_s, 4 parameters in total
        grad[3] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_log_T_L.array()).sum();

        grad[0] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_exp_n_1_T_L_x.array()).sum();
        grad[1] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_exp_n_1_T_L_y.array()).sum();
        grad[2] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_exp_n_1_T_L_z.array()).sum();
    }

    std::cout<<(intensities-rho_d_T_L-rho_s_T_L).squaredNorm()+lambd_rho_s_pars*rho_s_pars<<"\n";

    return (intensities-rho_d_T_L-rho_s_T_L).squaredNorm()+lambd_rho_s_pars*rho_s_pars;
}

double funcSFSLightBRDF(const std::vector<double> &para, std::vector<double> &grad, void *data)
{
    ImagePartAlg *img_alg_data_ptr = reinterpret_cast<ImagePartAlg *>(data);

    size_t n_dim = para.size(); 
    Eigen::VectorXf &intensities = img_alg_data_ptr->intensity_sig_chan;
    Eigen::MatrixXf &T_coeff = img_alg_data_ptr->T_coef;
    Eigen::MatrixX3f &S = img_alg_data_ptr->S_mat;
    Eigen::Vector3f &view = img_alg_data_ptr->view;
    Eigen::VectorXf &rho_d_last_kmeans = img_alg_data_ptr->rho_d_sig_chan;
    std::vector<int> &cluster_label = img_alg_data_ptr->pixel_cluster_label;
    std::vector<std::vector<int>> &I_smooth_adj = img_alg_data_ptr->I_smooth_adj;

    std::vector<double> para_temp = para;
    Eigen::VectorXf Light_rec = Eigen::Map<Eigen::VectorXd>(&para_temp[4+T_coeff.rows()], S.rows(), 1).cast<float>();
    Eigen::VectorXf rho_d = Eigen::Map<Eigen::VectorXd>(&para_temp[4], T_coeff.rows(), 1).cast<float>();

    Eigen::VectorXf rho_d_T_L = ((T_coeff*Light_rec).array()*rho_d.array()).matrix();

    // {Cx*viewx, Cy*viewy, Cz*viewz}
    Eigen::Vector3f C_view(para_temp[0]*view(0), para_temp[1]*view(1), para_temp[2]*view(2));
    Eigen::VectorXf S_C_view = ((S*C_view).array()<0).select(0, S*C_view); // we shouldn't consider those v*s < 0

    Eigen::VectorXf rho_s = (S_C_view.array().pow(para_temp[3])).matrix();
    Eigen::VectorXf rho_s_T_L = T_coeff*((rho_s.array()*Light_rec.array()).matrix());

    Eigen::VectorXf rho_s_log = (((S_C_view.array()<=0).select(1, S_C_view)).array().log()).matrix();
    Eigen::VectorXf rho_s_log_T_L = T_coeff*(rho_s.array()*rho_s_log.array()*Light_rec.array()).matrix();

    Eigen::VectorXf rho_s_exp_n_1 = float(para_temp[3])*(S_C_view.array().pow(para_temp[3]-1)).matrix();
    Eigen::VectorXf rho_s_exp_n_1_T_L_x = T_coeff*(rho_s_exp_n_1.array()*Light_rec.array()*(view(0)*S.col(0)).array()).matrix();
    Eigen::VectorXf rho_s_exp_n_1_T_L_y = T_coeff*(rho_s_exp_n_1.array()*Light_rec.array()*(view(1)*S.col(1)).array()).matrix();
    Eigen::VectorXf rho_s_exp_n_1_T_L_z = T_coeff*(rho_s_exp_n_1.array()*Light_rec.array()*(view(2)*S.col(2)).array()).matrix();

    // update cluster
    Eigen::Matrix<double, 5, 1> cnt_cluster= Eigen::Matrix<double, 5, 1>::Zero();
    Eigen::Matrix<double, 5, 1> center_cluster = Eigen::Matrix<double , 5, 1>::Zero();
    for (size_t i = 0; i < n_dim - 4 - Light_rec.rows(); ++i)
    {
        switch(cluster_label[i])
        {
        case 0:
            ++cnt_cluster(0);
            center_cluster(0) += rho_d(i);
            break;
        case 1:
            ++cnt_cluster(1);
            center_cluster(1) += rho_d(i);
            break;
        case 2:
            ++cnt_cluster(2);
            center_cluster(2) += rho_d(i);
            break;
        case 3:
            ++cnt_cluster(3);
            center_cluster(3) += rho_d(i);
            break;
        case 4:
            ++cnt_cluster(4);
            center_cluster(4) += rho_d(i);
            break;
        default:
            break;
        }
    }
    center_cluster = (center_cluster.array()/cnt_cluster.array()).matrix();
    // set new label
    double term_cluster = 0;
    Eigen::VectorXf rho_d_cluster(cluster_label.size());
    for (size_t i = 0; i < n_dim - 4 - Light_rec.rows(); ++i)
    {
        double min_dist = std::numeric_limits<double>::max();
        for (int j = 0; j < 5; ++j)
        {
            double dist = (rho_d(i)-center_cluster(j))*(rho_d(i)-center_cluster(j));
            if (dist < min_dist)
            {
                cluster_label[i] = j;
                rho_d_cluster(i) = center_cluster(j);
                min_dist = dist;
            }
        }
    }

    // rho d smooth term
    double rho_d_smooth = 0;
    for (size_t i = 0; i < I_smooth_adj.size(); ++i)
    {
        for (size_t j = 0; j < I_smooth_adj[i].size(); ++j)
        {
            rho_d_smooth += (rho_d(i) - rho_d(I_smooth_adj[i][j]))*(rho_d(i) - rho_d(I_smooth_adj[i][j]));
        }
    }

    if (grad.size() != 0)
    {
        // for rho_s, 4 parameters in total
        grad[3] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_log_T_L.array()).sum() + 1e-4;

        grad[0] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_exp_n_1_T_L_x.array()).sum() + 1e-4;
        grad[1] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_exp_n_1_T_L_y.array()).sum() + 1e-4;
        grad[2] = -2*((intensities-rho_d_T_L-rho_s_T_L).array()*rho_s_exp_n_1_T_L_z.array()).sum() + 1e-4;

        size_t i = 4;
        for (; i < grad.size() - Light_rec.rows(); ++i)
        {
            grad[i] = -2*(intensities(i-4)-rho_d_T_L(i-4)-rho_s_T_L(i-4))*(T_coeff.row(i-4).dot(Light_rec)) + 0.8*2*(rho_d(i-4)-rho_d_cluster(i-4)) + 1e-4;

            // gradient of rho_s_pars
            //double rho_d_pars_grad = 0;
            //rho_d_pars_grad = 2*(Rho[i] - rho_d_last_kmeans(i-4));
            ////for (size_t k = 0; k < n_rho_d; ++k)
            ////{
            ////	rho_d_pars_grad += exp(-(Rho[i]-rho_d_last_kmeans(i-4))*(Rho[i]-rho_d_last_kmeans(i-4))/(4*pars_band*pars_band))*(Rho[i]-rho_d_last_kmeans(i-4));
            ////}
            ////rho_d_pars_grad = (-2*rho_d_pars_grad/(n_rho_d*2*pars_band*1.7725)/(4*pars_band*pars_band))/rho_s_pars;
            // grad of smooth term
            for (size_t j = 0; j < I_smooth_adj[i-4].size(); ++j)
            {
                grad[i] += 4*(rho_d(i-4) - rho_d(I_smooth_adj[i-4][j]));//4 comes from Ii - Ij and Ij - Ii
            }

            //grad[i] += lambd_rho_s_pars*rho_d_pars_grad;
        }


        for (; i < grad.size(); ++i)
        {
            Eigen::VectorXf temp = (T_coeff.col(i-4-T_coeff.rows()).array()*rho_d.array()).matrix()
                + rho_s(i-4-T_coeff.rows())*T_coeff.col(i-4-T_coeff.rows());

            grad[i] = -2*(intensities-rho_d_T_L-rho_s_T_L).dot(temp) + 2*Light_rec(i-(4+T_coeff.rows())) + 1e-4;
        }

    }

    std::cout << rho_d_smooth + (intensities-rho_d_T_L-rho_s_T_L).squaredNorm() + Light_rec.squaredNorm() + 0.8*(rho_d-rho_d_cluster).squaredNorm()<<"\n";
        //<<"\t"<<(intensities-rho_d_T_L-rho_s_T_L).squaredNorm()
        //<<"\t"<<Light_rec.squaredNorm()
        //<<"\t"<<0.8*(rho_d-rho_d_cluster).squaredNorm()

    return rho_d_smooth + (intensities-rho_d_T_L-rho_s_T_L).squaredNorm() + Light_rec.squaredNorm() + 0.8*(rho_d-rho_d_cluster).squaredNorm();
}

double constraintsRhoC(const std::vector<double> &C, std::vector<double> &grad, void *data)
{
    float *C_coef = reinterpret_cast<float *>(data);

    if (grad.size() != 0)
    {
        grad = std::vector<double>(grad.size(), 0);

        grad[0] = -C_coef[0];
        grad[1] = -C_coef[1];
        grad[2] = -C_coef[2];
    }

    return ( - C[0]*C_coef[0] - C[1]*C_coef[1] - C[2]*C_coef[2]);
}

void ImagePartAlg::computeInitLight(Coarse *model, Viewer *viewer)
{
    std::cout << "\nBegin compute init light...\n";

    cv::Mat &photo = model->getPhoto();
    cv::Mat &mask = model->getMask();
    cv::Mat &rho_img = model->getRhoImg();
    Eigen::MatrixX3f &Light_rec = model->getLightRec();
    Light_rec.resize(model->getModelLightObj()->getNumSamples(), 3);
    std::vector<Eigen::Vector2i> &I_xy_vec = model->getXYInMask();


    // set lambd



    // photo should be float range [0, 1]

    // prepare Light coefficients matrix namely T
    uchar *mask_ptr = (uchar *)mask.data;
    Eigen::VectorXf light_coeffs;
    std::vector<Eigen::VectorXf> light_coeffs_vec;
    std::vector<Eigen::Vector3f> I_vec;
    I_xy_vec.clear();

    // for each pixel, Ii = (4*pi/num_samples)*Ti'*L. here we build T and I
    for (int i = 0; i < mask.rows; ++i)
    { // OpenCV stores mat in row major from left top while MATLAB is col major from left top
        for (int j = 0; j < mask.cols; ++j)
        {
            if (mask_ptr[i*mask.cols + j] > 0)
            { // TODO: need to refine, pixel in photo may not have correspondences in RImg, vice versa
                float winx, winy;
                if (!model->getPixelLightCoeffs(j, i, light_coeffs, viewer, winx, winy)) continue;
                light_coeffs_vec.push_back(light_coeffs);
                cv::Vec3f color = photo.at<cv::Vec3f>(i, j);
                I_vec.push_back(Eigen::Vector3f(color[0], color[1], color[2]));
                I_xy_vec.push_back(Eigen::Vector2i(j, i));
            }
        }
        //emit(refreshScreen());
    }

    I_smooth_adj.clear();
    findISmoothAdj(I_smooth_adj, I_xy_vec);


    T_coef.resize(light_coeffs_vec.size(), light_coeffs.size());
    Eigen::MatrixX3f I(I_vec.size(), 3);

    for (decltype(light_coeffs_vec.size()) i = 0; i < light_coeffs_vec.size(); ++i)
    {
        T_coef.row(i) = light_coeffs_vec[i];
        I.row(i) = I_vec[i];
    }

    // TODO: solve rho*((4*pi/num_samples).*T)*L = I if over determined. (1) least square (2) ransac

    // to make the decomposition more robust, we put the 4*pi/num_smaples to right hand of the eqation.
    // also, we consider initial BRDF rho as average of total pixel intensities
    Eigen::MatrixX3f I_temp = I;
    I_mat = I;
    float rhos[3] = { I_temp.col(0).mean(), I_temp.col(1).mean(), I_temp.col(2).mean() };
    //I_temp = (I_temp.array() / rhos_temp.array()).matrix();
    //I_temp.col(0) = I_temp.col(0) / rhos[0];
    //I_temp.col(1) = I_temp.col(1) / rhos[1];
    //I_temp.col(2) = I_temp.col(2) / rhos[2];
    //I_temp = (model->getModelLightObj()->getNumSamples() / 4 / M_PI)*I_temp;


    T_coef = (4 * M_PI / model->getModelLightObj()->getNumSamples())*T_coef;


    // use nlopt to solve bounded non-linear least squares

    //int n_dim = model->getModelLightObj()->getNumSamples();
    //nlopt::opt opt(nlopt::LD_MMA, n_dim);

    //std::vector<double> lb(n_dim, 0);
    //opt.set_lower_bounds(lb);

    //opt.set_min_objective(funcSFSLight, this);

    //opt.set_stopval(1e-4);
    //opt.set_ftol_rel(1e-4);
    //opt.set_ftol_abs(1e-4);
    //opt.set_xtol_rel(1e-4);
    //opt.set_xtol_abs(1e-4);


    //for (int k_chan = 0; k_chan < 3; ++k_chan)
    //{
    //	//std::cout<<I_temp.col(k_chan)<<"\n";
    //	brightness_sig_chan = I_temp.col(k_chan);
    //	std::vector<double> x(n_dim, 0.5);
    //	double minf;
    //	std::cout<<"Min value initialized: "<<minf<<"\n";
    //	nlopt::result result = opt.optimize(x, minf);
    //	std::cout<<"Min value optimized: "<<minf<<"\n";

    //	Eigen::Map<Eigen::VectorXd>cur_l_rec(&x[0], n_dim);
    //	Light_rec.col(k_chan) = cur_l_rec.cast<float>();
    //	std::cout<<"Return values of NLopt: "<<result<<"\n";
    //}

    //std::cout << "Compute init light finished\n";

    // standard non-linear least squares
    //Eigen::MatrixXf L_temp = Light_coeffs.transpose().eval()*Light_coeffs;
    //L_temp += Eigen::MatrixXf::Identity(light_coeffs_vec.size(), light_coeffs_vec.size());

    //Light_rec = L_temp.ldlt().solve(Light_coeffs.transpose().eval()*I_temp);

    // store new rho image computed by rho = I/(((4*pi/num_samples).*T)*L)
    // notice here we may not have per pixel rho, also need to store I

    // init rho image
    //std::vector<cv::Mat> rho_img_split;
    //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(rhos[0])));
    //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(rhos[1])));
    //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(rhos[2])));
    //cv::merge(rho_img_split, rho_img);

    //cv::imshow("temp_rho", rho_img);

    // put rho computed from new light back to rho image
    //Eigen::MatrixX3f rho_new;
    //rho_new = (I.array() / ((T_coef*Light_rec)).array()).matrix();


    //for (decltype(I_xy_vec.size()) i = 0; i < I_xy_vec.size(); ++i)
    //{
    //    if (rhos_temp(i, 0) <= 1.0f)
    //        rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0] = rhos_temp(i, 0);
    //    else
    //        rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0] = 1.0f;
    //    if (rhos_temp(i, 1) <= 1.0f)
    //        rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[1] = rhos_temp(i, 1);
    //    else
    //        rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[1] = 1.0f;
    //    if (rhos_temp(i, 2) <= 1.0f)
    //        rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[2] = rhos_temp(i, 2);
    //    else
    //        rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[2] = 1.0f;
    //}

    // output some variables for debug here
    //std::ofstream f_light_rec(model->getDataPath() + "/T_coef.mat");
    //if (f_light_rec)
    //{
    //	f_light_rec << T_coef;
    //	f_light_rec.close();
    //}

    //std::ofstream f_L_temp(model->getDataPath() + "/L_temp.mat");
    //if (f_L_temp)
    //{
    //    std::cout << light_coeffs_vec.size();
    //    //f_L_temp << Light_coeffs;
    //    f_L_temp.close();
    //}

    //std::ofstream f_rho_new_init(model->getDataPath() + "/rho_new.mat");
    //if (f_rho_new_init)
    //{
    //    f_rho_new_init << rho_new;
    //    f_rho_new_init.close();
    //}

    updateRho(model, viewer);

    T_coef.resize(0,0);
    brightness_sig_chan.resize(0);
}

void ImagePartAlg::updateLight(Coarse *model, Viewer *viewer)
{
    std::cout << "\nBegin compute updated light...\n";

    cv::Mat &photo = model->getPhoto();
    cv::Mat &mask = model->getMask();
    cv::Mat &rho_img = model->getRhoImg();
    cv::Mat &r_img = model->getRImg();
    Eigen::MatrixX3f &Light_rec = model->getLightRec();

    cv::Mat photo_chan[3];
    cv::Mat rho_img_chan[3];
    cv::Mat r_img_chan[3];
    Eigen::VectorXf Light_rec_chan[3];

    cv::split(photo, photo_chan);
    cv::split(rho_img, rho_img_chan);
    cv::split(r_img, r_img_chan);
    Light_rec_chan[0] = Light_rec.col(0);
    Light_rec_chan[1] = Light_rec.col(1);
    Light_rec_chan[2] = Light_rec.col(2);

    for (int i = 0; i < 3; ++i)
    {
        updateLightSingleChannel(photo_chan[i], mask, r_img_chan[i], model, viewer, rho_img_chan[i], Light_rec_chan[i]);
    }

    cv::merge(photo_chan, 3, photo);
    cv::merge(rho_img_chan, 3, rho_img);
    cv::merge(r_img_chan, 3, r_img);
    Light_rec.col(0) = Light_rec_chan[0];
    Light_rec.col(1) = Light_rec_chan[1];
    Light_rec.col(2) = Light_rec_chan[2];

    // output some variables for debug here
    //std::ofstream f_light_rec(model->getDataPath() + "/L_rec_update.mat");
    //if (f_light_rec)
    //{
    //    f_light_rec << Light_rec;
    //    f_light_rec.close();
    //}

    cv::imshow("rho_img", rho_img);

    cv::Mat brightness;
    cv::divide(photo, rho_img, brightness);
    cv::imshow("brightness", brightness);

    // give vertex on model its rho
    model->updateVertexRho();
    model->updateVertexBrightnessAndColor();

    std::cout << "Update light finished\n";
}

void ImagePartAlg::updateRho(Coarse *model, Viewer *viewer)
{
    std::cout << "\nBegin compute BRDF parameters...\n";

    // set view
    float view_temp[3];
    viewer->getViewDirection(view_temp);
    view = Eigen::Vector3f(view_temp[0], view_temp[1], view_temp[2]);

    // set S matrix
    SAMPLE *model_samples = model->getModelLightObj()->getSamples();
    int num_samples = model->getModelLightObj()->getNumSamples();
    S_mat.resize(num_samples, 3);
    for (int i = 0; i < num_samples; ++i)
    {
        S_mat.row(i) = model_samples[i].direction;
    }


    // set S*view matrix for inequality constraints
    std::vector<float> S_view(S_mat.rows()*S_mat.cols());
    for (size_t i = 0; i < S_view.size()/3; ++i)
    {
        S_view[3*i+0] = S_mat(i, 0)*view(0);
        S_view[3*i+1] = S_mat(i, 1)*view(1);
        S_view[3*i+2] = S_mat(i, 2)*view(2);
    }


    // some data
    std::cout<<T_coef.rows()<<"\n";
    Eigen::MatrixX3f Rho_rec(T_coef.rows() + 4, 3);
    Eigen::MatrixX3f &Light_rec = model->getLightRec();
    cv::Mat &rho_img = model->getRhoImg();
    std::vector<Eigen::Vector2i> &I_xy_vec = model->getXYInMask();
    Eigen::Matrix<float, 4, 3> &rho_specular = model->getRhoSpclr();
    //Eigen::MatrixX3f rho_s_mat(I_xy_vec.size(), 3);
    //for (size_t i = 0; i < I_xy_vec.size(); ++i)
    //{
    //	rho_s_mat.row(i) = Eigen::Map<Eigen::Vector3f>(&rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0], 3, 1);
    //}

    // use the I_xy_vec to compute kmeans
    Eigen::MatrixX3f rhos_temp;
    std::vector<int> cluster_label;
    model->rhoFromKMeans(5, rhos_temp, cluster_label);
    rho_d_mat = rhos_temp;

    // set optimization
    int n_dim = T_coef.rows() + 4 + T_coef.cols();
    nlopt::opt opt(nlopt::LD_MMA, n_dim);


    // set bounds
    std::vector<double> lb(n_dim, 0);
    lb[0] = -HUGE_VAL;
    lb[1] = -HUGE_VAL;
    lb[2] = -HUGE_VAL;
    lb[3] = 1; // rho_s_n should be larger than 1

    std::vector<double> ub(n_dim, 1);
    ub[0] = HUGE_VAL;
    ub[1] = HUGE_VAL;
    ub[2] = HUGE_VAL;
    ub[3] = HUGE_VAL;

    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    // objective function
    opt.set_min_objective(funcSFSLightBRDF, this);

    // set inequality constraints to ensure Cx*ux*vx+Cy*uy*vy+Cz*uz*vz > 0
    //std::vector<int> Rho_C_constraint_index;
    //for (int i = 0; i < S_mat.rows(); ++i)
    //{
    //	if (S_mat.row(i).dot(view) > 0) 
    //		Rho_C_constraint_index.push_back(i);
    //}

    //std::ofstream f_S_view(model->getDataPath() + "/T_coef.mat");
    //if (f_S_view)
    //{
    //	for (size_t i = 0; i < Rho_C_constraint_index.size(); ++i)
    //	{
    //		f_S_view <<Rho_C_constraint_index[i]<<"\n";
    //	}
    //	
    //	f_S_view.close();
    //}

    //for (size_t i = 0; i < Rho_C_constraint_index.size(); ++i)
    //{
    //	//opt.add_inequality_constraint(constraintsRhoC, &S_view[3*Rho_C_constraint_index[i]], 1e-8);
    //}

    // stop criteria
    //opt.set_stopval(1e-4);
    //opt.set_ftol_rel(1e-4);
    opt.set_ftol_abs(1e-3);
    opt.set_maxtime(60);
    //opt.set_xtol_rel(1e-4);
    //opt.set_xtol_abs(1e-3);


    for (int k_chan = 0; k_chan < 3; ++k_chan)
    {
        //std::cout<<I_temp.col(k_chan)<<"\n";
        intensity_sig_chan = I_mat.col(k_chan);
        Light_rec_sig_chan = Light_rec.col(k_chan);
        rho_d_sig_chan = rho_d_mat.col(k_chan);
        pixel_cluster_label = cluster_label;

        //std::vector<double> x(n_dim, 1);
        // prepare start status of x
        std::vector<double> x(n_dim);
        for (int i = 0; i < 4; ++i)
        {
            x[i] = rho_specular(i, k_chan);
        }
        for (int i = 4; i < n_dim - Light_rec.rows(); ++i)
        {
            x[i] = rho_img.at<cv::Vec3f>(I_xy_vec[i-4](1), I_xy_vec[i-4](0))[k_chan];
        }
        for (int i = n_dim - Light_rec.rows(); i < n_dim; ++i )
        {
            x[i] = Light_rec(i-(n_dim-Light_rec.rows()), k_chan);
        }


        //for (size_t i = 0; i < I_xy_vec.size(); ++i)
        //{
        //	x[i+4] = rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[k_chan];
        //}
        double minf = 0;
        std::cout<<"Min value initialized: "<<minf<<"\n";
        nlopt::result result = opt.optimize(x, minf);
        std::cout<<"Min value optimized: "<<minf<<"\n";

        Eigen::Map<Eigen::VectorXd>cur_rho_rec(&x[0], n_dim-T_coef.cols());
        Rho_rec.col(k_chan) = cur_rho_rec.cast<float>();
        Eigen::Map<Eigen::VectorXd>cur_l_rec(&x[n_dim-T_coef.cols()], T_coef.cols());
        Light_rec.col(k_chan) = cur_l_rec.cast<float>();
        std::cout<<"Return values of NLopt: "<<result<<"\n";
    }

    // update rho
    cv::Mat &photo = model->getPhoto();
    cv::Mat &mask = model->getMask();
    


    rho_specular.row(0) = Rho_rec.row(0);
    rho_specular.row(1) = Rho_rec.row(1);
    rho_specular.row(2) = Rho_rec.row(2);
    rho_specular.row(3) = Rho_rec.row(3);

    std::vector<cv::Mat> rho_img_split;
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(Rho_rec.col(0).tail(Rho_rec.rows()-4).mean())));
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(Rho_rec.col(1).tail(Rho_rec.rows()-4).mean())));
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(Rho_rec.col(2).tail(Rho_rec.rows()-4).mean())));
    cv::merge(rho_img_split, rho_img);

    for (decltype(I_xy_vec.size()) i = 0; i < I_xy_vec.size(); ++i)
    {
        if (Rho_rec(i+4, 0) <= 1.0f)
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0] = Rho_rec(i+4, 0);
        else
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0] = 1.0f;

        if (Rho_rec(i+4, 1) <= 1.0f)
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[1] = Rho_rec(i+4, 1);
        else
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[1] = 1.0f;

        if (Rho_rec(i+4, 2) <= 1.0f)
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[2] = Rho_rec(i+4, 2);
        else
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[2] = 1.0f;
    }

    char time_postfix[50];
    time_t current_time = time(NULL);
    strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
    std::string file_time_postfix = time_postfix;

    cv::imshow("rho_img", rho_img);
    cv::imwrite(model->getOutputPath() + "/rho_img" + file_time_postfix + ".png", rho_img*255);

    //cv::Mat brightness;
    //cv::divide(photo, rho_img, brightness);
    //double b_min, b_max;
    //cv::minMaxLoc(brightness, &b_min, &b_max);
    //cv::threshold(brightness, brightness, 5, 1, cv::THRESH_TRUNC);
    //brightness = brightness / b_max;
    //cv::imshow("brightness", brightness);

    std::ofstream f_output(model->getOutputPath() + "/Rho_rec" + file_time_postfix + ".mat");
    if (f_output)
    {
        f_output << Rho_rec << "\n";
        f_output.close();
    }

    f_output.open(model->getOutputPath() + "/I_mat" + file_time_postfix + ".mat");
    if (f_output)
    {
        f_output << I_mat << "\n";
        f_output.close();
    }

    f_output.open(model->getOutputPath() + "/Light_rec" + file_time_postfix + ".mat");
    if (f_output)
    {
        f_output << Light_rec << "\n";
        f_output.close();
    }

    // give vertex on model its rho
    model->updateVertexRho();
    model->updateVertexBrightnessAndColor();


    std::cout<<"Compute BRDF finished...\n";

}

void ImagePartAlg::updateLightSingleChannel(cv::Mat &photo, cv::Mat &mask, cv::Mat &r_img, Coarse *model, Viewer *viewer, cv::Mat &rho_img, Eigen::VectorXf &Light_rec)
{
    // single channel here! caller function should split the photo into channel image
    // because weight of pixels different in different channel makes to T different
    // in each channel (I=rho*T*L)
    // photo and r_img should be float range [0, 1]
    // mask should be uchar or bool?

    // TODO: use sift flow correspondences, not pixel correspondence

    // prepare Light coefficients namely T, here we assign a weight to each pixel
    // according to the absolute difference between photo and render image
    //cv::Mat weight = cv::abs(photo - r_img);
    uchar *mask_ptr = (uchar *)mask.data;
    Eigen::VectorXf light_coeffs;
    std::vector<Eigen::VectorXf> light_coeffs_vec;
    std::vector<float> I_vec;
    std::vector<float> w_vec;
    // we have the x, y in mask computed in last step when initializing light,
    // just get it from model
    std::vector<Eigen::Vector2i> &I_xy_vec = model->getXYInMask();
    std::vector<Eigen::Vector2i> I_cur_xy_vec;
    std::vector<float> rho_vec;

    // we may not need mask here if we have dense correspondences rimg(x,y) and photo(x,y)
    // for each pixel, Ii = (4*pi/num_samples)*Ti'*L. here we build T and I

    // here we have the x, y in mask, so do not traverse all the image
    // just traverse the I_xy_vec
    int num_rho_zero = 0;
    for (decltype(I_xy_vec.size()) i = 0; i < I_xy_vec.size(); ++i)
    {
        if (abs(rho_img.at<float>(I_xy_vec[i](1), I_xy_vec[i](0))) < 0.001)
        {
            //std::cout << rho_img.at<float>(I_xy_vec[i](1), I_xy_vec[i](0)) << "\n";
            ++num_rho_zero;
            continue;
        }
        I_cur_xy_vec.push_back(I_xy_vec[i]);
        float winx, winy;
        model->getPixelLightCoeffs(I_xy_vec[i](0), I_xy_vec[i](1), light_coeffs, viewer, winx, winy);
        light_coeffs_vec.push_back(light_coeffs);
        I_vec.push_back(photo.at<float>(I_xy_vec[i](1), I_xy_vec[i](0)));
        rho_vec.push_back(rho_img.at<float>(I_xy_vec[i](1), I_xy_vec[i](0)));
        w_vec.push_back(sigmoid(3.0f, abs(photo.at<float>(I_xy_vec[i](1), I_xy_vec[i](0)) - r_img.at<float>((int)(winy+0.5f), (int)(winx+0.5f)))));
        //w_vec.push_back(1.0f);
    }

    std::cout << "Number of rho values zero: " << num_rho_zero << "\n";

    //num_rho_zero = 0;
    //for (decltype(I_xy_vec.size()) i = 0; i < I_xy_vec.size(); ++i)
    //{
    //    if (photo.at<float>(I_xy_vec[i](1), I_xy_vec[i](0)) < 0.001f)
    //    {
    //        ++num_rho_zero;
    //    }
    //}
    //std::cout << "Number of photo values zero: " << num_rho_zero << "\n";

    //for (int i = 0; i < mask.rows; ++i)
    //{ // OpenCV stores mat in row major from left top while MATLAB is col major from left top
    //    for (int j = 0; j < mask.cols; ++j)
    //    {
    //        if (mask_ptr[i*mask.cols + j] > 0)
    //        {
    //            float winx, winy;
    //            if(!model->getPixelLightCoeffs(j, i, light_coeffs, viewer, winx, winy)) continue;
    //            light_coeffs_vec.push_back(light_coeffs);
    //            I_vec.push_back(photo.at<float>(i, j));
    //            w_vec.push_back(photo.at<float>(i, j)-weight.at<float>(i, j));
    //            I_xy_vec.push_back(Eigen::Vector2i(j, i));
    //        }
    //    }
    //}


    Eigen::MatrixXf Light_coeffs(light_coeffs_vec.size(), light_coeffs.size());
    Eigen::VectorXf I = Eigen::Map<Eigen::VectorXf>(&I_vec[0], I_vec.size());
    Eigen::VectorXf rho = Eigen::Map<Eigen::VectorXf>(&rho_vec[0], rho_vec.size());
    Eigen::VectorXf Weight = Eigen::Map<Eigen::VectorXf>(&w_vec[0], w_vec.size());

    for (decltype(light_coeffs_vec.size()) i = 0; i < light_coeffs_vec.size(); ++i)
    {
        Light_coeffs.row(i) = sqrt(w_vec[i])*light_coeffs_vec[i];
    }

    // TODO: solve rho*((4*pi/num_samples).*T)*L = I if over determined. (1) least square (2) ransac

    // to make the decomposition more robust, we put the 4*pi/num_smaples to right hand of the eqation.
    // also, we need to divide I by corresponding rho
    Eigen::VectorXf I_temp = I;
    I_temp = (Weight.array().sqrt() * I.array() / rho.array()).matrix();
    //I_temp = (model->getModelLightObj()->getNumSamples() / 4 / M_PI)*I_temp;

    Light_coeffs = (4 * M_PI / model->getModelLightObj()->getNumSamples())*Light_coeffs;
    Eigen::MatrixXf L_temp = Light_coeffs.transpose().eval()*Light_coeffs;
    L_temp += Eigen::MatrixXf::Identity(light_coeffs_vec.size(), light_coeffs_vec.size());

    //std::ofstream f_light_rec(model->getDataPath() + "/L_temp_update.mat");
    //if (f_light_rec)
    //{
    //    f_light_rec << L_temp;
    //    f_light_rec.close();
    //}

    Light_rec = L_temp.ldlt().solve(Light_coeffs.transpose().eval()*I_temp);

    // put rho computed from new light back to rho image
    Eigen::VectorXf rho_new;
    rho_new = (I.array() / ((Light_coeffs*Light_rec)).array()).matrix();
    for (decltype(I_cur_xy_vec.size()) i = 0; i < I_cur_xy_vec.size(); ++i)
    {
        // under this framework it happens in some pixel T*L < I 
        // which means it is unreliable to compute rho by I/(T*L)
        // we retain the last step rho here to prevent it excess 1
        if (rho_new(i) <= 1.0f)
            rho_img.at<float>(I_cur_xy_vec[i](1), I_cur_xy_vec[i](0)) = rho_new(i);
        //else
        //rho_img.at<float>(I_cur_xy_vec[i](1), I_cur_xy_vec[i](0)) = 1.0f;
    }

    std::cout << "Update light of single channel finished...\n";
}

void ImagePartAlg::computeNormal(Coarse *model, Viewer *viewer)
{
    std::cout << "\nBegin compute normal...\n";

    cv::Mat &photo = model->getPhoto();
    cv::Mat &mask = model->getMask();
    cv::Mat &rho_img = model->getRhoImg();
    std::vector<float> *vertex_list = model->getVertexList();
    std::vector<unsigned int> *face_list = model->getFaceList();
    std::vector<float> *face_normal_list = model->getFaceNormalList();

    // we only compute guidance normal defined on face

    std::vector<int> faces_in_photo;
    model->findFacesInPhoto(faces_in_photo);

    // test if we get right faces
    std::cout << "Found number of faces in photo: " << faces_in_photo.size() << "\n";
    viewer->setShowModel(false);
    std::cout<<"test";
    for (auto &i : faces_in_photo)
    {
        int v0_id = (*face_list)[3 * i + 0];
        int v1_id = (*face_list)[3 * i + 1];
        int v2_id = (*face_list)[3 * i + 2];
        float v0[3] = { (*vertex_list)[3 * v0_id + 0], (*vertex_list)[3 * v0_id + 1], (*vertex_list)[3 * v0_id + 2] };
        float v1[3] = { (*vertex_list)[3 * v1_id + 0], (*vertex_list)[3 * v1_id + 1], (*vertex_list)[3 * v1_id + 2] };
        float v2[3] = { (*vertex_list)[3 * v2_id + 0], (*vertex_list)[3 * v2_id + 1], (*vertex_list)[3 * v2_id + 2] };
        float c[3] = { 1.0f, 0.0f, 0.0f };
        viewer->addDrawableTri(v0, v1, v2, c, c, c);
        //system("pause");
    }
    emit(refreshScreen());
    //system("pause");


    Eigen::VectorXf pixel_counts = Eigen::VectorXf::Zero(faces_in_photo.size());
    Eigen::MatrixX3f rho_in_photo = Eigen::MatrixX3f::Zero(faces_in_photo.size(), 3);
    Eigen::MatrixX3f I_in_photo = Eigen::MatrixX3f::Zero(faces_in_photo.size(), 3);
    uchar *mask_ptr = (uchar *)mask.data;

    // prepare data

    for (int i = 0; i < mask.rows; ++i)
    { // OpenCV stores mat in row major from left top while MATLAB is col major from left top
        for (int j = 0; j < mask.cols; ++j)
        {
            if (mask_ptr[i*mask.cols + j] > 0)
            {   
                int face_id = -1;
                model->getCrspFaceIdFromPhotoToRImg(j, i, face_id);
                const std::vector<int>::iterator id = find(faces_in_photo.begin(), faces_in_photo.end(), face_id);
                const int idx = static_cast<int> (id - faces_in_photo.begin());
                if (id!=faces_in_photo.end())
                {
                    pixel_counts(idx) += 1.0f;
                    cv::Vec3f color = photo.at<cv::Vec3f>(i, j);
                    cv::Vec3f rho = rho_img.at<cv::Vec3f>(i, j);
                    rho_in_photo.row(idx) += Eigen::RowVector3f(rho[0], rho[1], rho[2]);
                    I_in_photo.row(idx) += Eigen::RowVector3f(color[0], color[1], color[2]);
                }
            }
        }
    }

    rho_in_photo.col(0) = (rho_in_photo.col(0).array() / pixel_counts.array()).matrix();
    rho_in_photo.col(1) = (rho_in_photo.col(1).array() / pixel_counts.array()).matrix();
    rho_in_photo.col(2) = (rho_in_photo.col(2).array() / pixel_counts.array()).matrix();
    I_in_photo.col(0) = (I_in_photo.col(0).array() / pixel_counts.array()).matrix();
    I_in_photo.col(1) = (I_in_photo.col(1).array() / pixel_counts.array()).matrix();
    I_in_photo.col(2) = (I_in_photo.col(2).array() / pixel_counts.array()).matrix();
    Eigen::MatrixX3f brightness_in_photo = (I_in_photo.array() / rho_in_photo.array()).matrix();


    // TODO: traverse mask to find faces in photo, record number of pixel of each face and sum of intensity,rho

    // get recovered light
    Eigen::MatrixX3f light_rec = model->getRecoveredLight();

    // get sample direction
    int num_samples = model->getModelLightObj()->getNumSamples();
    Eigen::MatrixX3f &samples = S_mat; 


    // build linear sys
    //std::vector<Eigen::Triplet<float>> normal_coeffs;
    //Eigen::VectorXf right_hand(3 * faces_in_photo.size());
    //Eigen::SparseMatrix<float> Normal_coeffs(3 * faces_in_photo.size(), 3 * faces_in_photo.size());
    brightness_mat = (num_samples/4/M_PI)*I_in_photo;// we now consider full render equation, no need to divide rho_d
    A_mat.resize(faces_in_photo.size(), 3);
    B_mat.resize(faces_in_photo.size(), 3);
    C_mat.resize(faces_in_photo.size(), 3);
    F_smooth_adj.resize(faces_in_photo.size());

    Eigen::Matrix<float, 4, 3> &rho_specular = model->getRhoSpclr();
    Eigen::Matrix3f C_view;
    C_view << rho_specular(0,0)*view(0), rho_specular(0,1)*view(0), rho_specular(0,2)*view(0), 
        rho_specular(1,0)*view(1), rho_specular(1,1)*view(1), rho_specular(1,2)*view(1), 
        rho_specular(2,0)*view(2), rho_specular(2,1)*view(2), rho_specular(2,2)*view(2);
    Eigen::MatrixX3f S_C_view = ((S_mat*C_view).array()<0).select(0, S_mat*C_view);
    Eigen::MatrixX3f rho_s(S_C_view.rows(), 3);
    rho_s.col(0) = (S_C_view.col(0).array().pow(rho_specular(3, 0))).matrix();
    rho_s.col(1) = (S_C_view.col(1).array().pow(rho_specular(3, 1))).matrix();
    rho_s.col(2) = (S_C_view.col(2).array().pow(rho_specular(3, 2))).matrix();

    lambd_sfs = 10.0f;
    lambd_smooth = 0.0f;
    lambd_norm = 5.0f;
    float lambda_sfs = 10.0;
    float lambda_smooth = 0.0;


    for (decltype(faces_in_photo.size()) i = 0; i < faces_in_photo.size(); ++i)
    {
        //for each face we need to build its 3 equations for nx, ny and nz
        Eigen::VectorXf visb;
        model->computeVisbs(faces_in_photo[i], visb);

        // for 3 channels
        Eigen::Vector3f A(0.0, 0.0, 0.0);
        Eigen::Vector3f B(0.0, 0.0, 0.0);
        Eigen::Vector3f C(0.0, 0.0, 0.0);

        for (int j = 0; j < 3; ++j)
        {
            // (I/rho)*(num_samples/4/pi) = A*nx + B*ny + C*nz
            // 2015/1/5 modified: I = A*nx + B*ny + C*nz (A, B and C contains rho_d and rho_s)

            Eigen::VectorXf A_temp_vec = (light_rec.col(j).array()*samples.col(0).array()*visb.array()).matrix();
            A(j) = rho_in_photo(i, j)*A_temp_vec.array().sum();
            A(j) += (rho_s.col(j).array()*A_temp_vec.array()).sum();
            A_mat(i, j) = A(j);
            //if (A(j) < 0) std::cout << "Normal coefficient is less than zero!\t" << "i j: " << i << "\t" << j << "\n";

            Eigen::VectorXf B_temp_vec = (light_rec.col(j).array()*samples.col(1).array()*visb.array()).matrix();
            B(j) = rho_in_photo(i, j)*(light_rec.col(j).array()*samples.col(1).array()*visb.array()).sum();
            B(j) += (rho_s.col(j).array()*B_temp_vec.array()).sum();
            B_mat(i, j) = B(j);
            //if (B(j) < 0) std::cout << "Normal coefficient is less than zero!t" << "i j: " << i << "\t" << j << "\n";

            Eigen::VectorXf C_temp_vec = (light_rec.col(j).array()*samples.col(2).array()*visb.array()).matrix();
            C(j) = rho_in_photo(i, j)*(light_rec.col(j).array()*samples.col(2).array()*visb.array()).sum();
            C(j) += (rho_s.col(j).array()*C_temp_vec.array()).sum();
            C_mat(i,j) = C(j);
            //if (C(j) < 0) std::cout << "Normal coefficient is less than zero!\t" << "i j: " << i << "\t" << j << "\n";
        }

        //      // equation for nx
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 0, lambda_sfs*A.dot(A)));
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 1, lambda_sfs*B.dot(A)));
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 2, lambda_sfs*C.dot(A)));

        //      // equation for ny
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 0, lambda_sfs*A.dot(B)));
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 1, lambda_sfs*B.dot(B)));
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 2, lambda_sfs*C.dot(B)));

        //      // equation for nz
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 0, lambda_sfs*A.dot(C)));
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 1, lambda_sfs*B.dot(C)));
        //      normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 2, lambda_sfs*C.dot(C)));

        //right_hand(3 * i + 0) = (num_samples / 4 / M_PI)*(A.dot(brightness_in_photo.row(i)));
        //right_hand(3 * i + 1) = (num_samples / 4 / M_PI)*(B.dot(brightness_in_photo.row(i)));
        //right_hand(3 * i + 2) = (num_samples / 4 / M_PI)*(C.dot(brightness_in_photo.row(i)));

        // smooth term
        std::vector<int> *cur_face_adj = model->getFaceAdj(faces_in_photo[i]);
        int num_adj_faces_in_photo = 0; // the adjacent list may contains faces that are not visible in the photo
        std::vector<int> cur_f_smooth_adj;


        for (decltype((*cur_face_adj).size()) j = 0; j < (*cur_face_adj).size(); ++j)
        {
            // find id of adjacent face in the faces_in_photo
            const std::vector<int>::iterator pos = std::find(faces_in_photo.begin(), faces_in_photo.end(), (*cur_face_adj)[j]);
            if (pos == faces_in_photo.end()) continue;
            const int idx = static_cast<int> (pos - faces_in_photo.begin());
            ++num_adj_faces_in_photo;

            cur_f_smooth_adj.push_back(idx);

            //// equation for nx
            //normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * idx + 0, -lambda_smooth));

            //// equation for ny
            //normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * idx + 1, -lambda_smooth));

            //// equation for nz
            //normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * idx + 2, -lambda_smooth));
        }
        //normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 0, lambda_smooth*num_adj_faces_in_photo));
        //normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 1, lambda_smooth*num_adj_faces_in_photo));
        //normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 2, lambda_smooth*num_adj_faces_in_photo));

        F_smooth_adj[i] = cur_f_smooth_adj;
    }


    //Normal_coeffs.setFromTriplets(normal_coeffs.begin(), normal_coeffs.end());

    //Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> chol(Normal_coeffs);
    //Eigen::VectorXf new_face_in_photo_normal = chol.solve(right_hand);


    // use nlopt to solve bounded non-linear least squares

    int n_dim = faces_in_photo.size()*3;
    nlopt::opt opt(nlopt::LD_MMA, n_dim);

    opt.set_min_objective(funcSFSNormal, this);

    opt.set_stopval(1e-4);
    opt.set_ftol_rel(1e-4);
    opt.set_ftol_abs(1e-4);
    opt.set_xtol_rel(1e-4);
    opt.set_xtol_abs(1e-4);

    // declare initial x
    std::vector<double> x(n_dim);

    // we set the initial x as its original normal
    for (size_t i = 0; i < faces_in_photo.size(); ++i)
    {
        x[3*i+0] = (*face_normal_list)[3*faces_in_photo[i]+0];
        x[3*i+1] = (*face_normal_list)[3*faces_in_photo[i]+1];
        x[3*i+2] = (*face_normal_list)[3*faces_in_photo[i]+2];
    }

    double minf;
    std::cout<<"Min value initialized: "<<minf<<"\n";
    nlopt::result result = opt.optimize(x, minf);
    std::cout<<"Min value optimized: "<<minf<<"\n";
    std::cout<<"Return values of NLopt: "<<result<<"\n";

    Eigen::Map<Eigen::VectorXd>N_rec(&x[0], n_dim);
    Eigen::VectorXf new_face_in_photo_normal = N_rec.cast<float>();


    // output some variables for debug here

    //std::ofstream f_brightness_in_photo(model->getDataPath() + "/brightness_in_photo.mat");
    //if (f_brightness_in_photo)
    //{
    //    f_brightness_in_photo << brightness_in_photo;
    //    f_brightness_in_photo.close();
    //}

    //std::ofstream f_Normal_coeffs(model->getDataPath() + "/Normal_coeffs.mat");
    //if (f_Normal_coeffs)
    //{
    //    f_Normal_coeffs << Normal_coeffs;
    //    f_Normal_coeffs.close();
    //}

    std::ofstream f_new_normal(model->getOutputPath() + "/new_normal.mat");
    if (f_new_normal)
    {
        f_new_normal << new_face_in_photo_normal;
        f_new_normal.close();
    }

    f_new_normal.open(model->getOutputPath() + "/faces_in_photo.mat");
    if (f_new_normal)
    {
        f_new_normal << Eigen::Map<Eigen::VectorXi>(&faces_in_photo[0], faces_in_photo.size(), 1);
        f_new_normal.clear();
    }

    //std::ofstream f_right_hand(model->getDataPath() + "/right_hand.mat");
    //if (f_right_hand)
    //{
    //    f_right_hand << right_hand;
    //    f_right_hand.close();
    //}

    //std::ofstream f_pixel_counts(model->getDataPath() + "/pixel_counts.mat");
    //if (f_pixel_counts)
    //{
    //    f_pixel_counts << pixel_counts;
    //    f_pixel_counts.close();
    //}

    // set new normal to model
    model->setModelNewNormal(new_face_in_photo_normal, faces_in_photo);
    model->updateVertexBrightnessAndColor();
    model->drawNormal();
    emit(refreshScreen());

    brightness_mat.resize(0, 3);
    A_mat.resize(0, 3);
    B_mat.resize(0, 3);
    C_mat.resize(0, 3);
    F_smooth_adj.clear();

    std::cout << "Compute new normal finished...\n";
}

float ImagePartAlg::sigmoid(float coef, float t)
{
    return 2.0f / (1.0f + exp(coef*t));
}

void ImagePartAlg::findISmoothAdj(std::vector<std::vector<int>> &I_smooth_adj, std::vector<Eigen::Vector2i> &I_xy_vec)
{
    I_smooth_adj.clear();


    for (size_t i = 0; i < I_xy_vec.size(); ++i)
    {
        Eigen::Vector2i cur_I_xy = I_xy_vec[i];
        std::vector<int> cur_I_smooth_adj;

        // find its four neighbour
        std::vector<Eigen::Vector2i>::iterator id;
        
        --cur_I_xy(0);
        id = find(I_xy_vec.begin(), I_xy_vec.end(), cur_I_xy);
        if (id != I_xy_vec.end())
            cur_I_smooth_adj.push_back(static_cast<int> (id - I_xy_vec.begin()));

        ++cur_I_xy(0);
        ++cur_I_xy(0);
        id = find(I_xy_vec.begin(), I_xy_vec.end(), cur_I_xy);
        if (id != I_xy_vec.end())
            cur_I_smooth_adj.push_back(static_cast<int> (id - I_xy_vec.begin()));

        --cur_I_xy(0);
        --cur_I_xy(1);
        id = find(I_xy_vec.begin(), I_xy_vec.end(), cur_I_xy);
        if (id != I_xy_vec.end())
            cur_I_smooth_adj.push_back(static_cast<int> (id - I_xy_vec.begin()));

        ++cur_I_xy(1);
        ++cur_I_xy(1);
        id = find(I_xy_vec.begin(), I_xy_vec.end(), cur_I_xy);
        if (id != I_xy_vec.end())
            cur_I_smooth_adj.push_back(static_cast<int> (id - I_xy_vec.begin()));

        I_smooth_adj.push_back(cur_I_smooth_adj);
    }
}

// test NLopt here
double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    if (!grad.empty()) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct {
    double a, b;
} my_constraint_data;

double myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    my_constraint_data *d = (my_constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
}

void ImagePartAlg::testNLopt()
{

    nlopt::opt opt(nlopt::LD_MMA, 2);

    std::vector<double> lb(2);
    lb[0] = -HUGE_VAL; lb[1] = 0;
    opt.set_lower_bounds(lb);

    opt.set_min_objective(myvfunc, NULL);

    my_constraint_data data[2] = { {2,0}, {-1,1} };
    opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
    opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);

    opt.set_xtol_rel(1e-4);

    std::vector<double> x(2);
    x[0] = 1.234; x[1] = 5.678;
    double minf;
    nlopt::result result = opt.optimize(x, minf);
}