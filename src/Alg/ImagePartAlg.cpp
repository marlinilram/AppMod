#include "ImagePartAlg.h"

ImagePartAlg::ImagePartAlg()
{
    //function_type = -1;
    //coarse_model = nullptr;
    //viewer = nullptr;
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

	// here we compute the value of objective function given the current L
	std::vector<double> N_temp = N;
	Eigen::Map<Eigen::VectorXd>cur_N(&N_temp[0], n_dim);
	Eigen::MatrixX3f &brightness = img_alg_data_ptr->brightness_mat;
	Eigen::MatrixX3f &A = img_alg_data_ptr->A_mat;
	Eigen::MatrixX3f &B = img_alg_data_ptr->B_mat;
	Eigen::MatrixX3f &C = img_alg_data_ptr->C_mat;

	double func_val = 0;

	for (int i = 0; i < 3; ++i)
	{

	}

	if (grad.size() != 0)
	{

	}
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
    float rhos[3] = { I_temp.col(0).mean(), I_temp.col(1).mean(), I_temp.col(2).mean() };
    I_temp.col(0) = I_temp.col(0) / rhos[0];
    I_temp.col(1) = I_temp.col(1) / rhos[1];
    I_temp.col(2) = I_temp.col(2) / rhos[2];
    //I_temp = (model->getModelLightObj()->getNumSamples() / 4 / M_PI)*I_temp;

    T_coef = (4 * M_PI / model->getModelLightObj()->getNumSamples())*T_coef;


	// use nlopt to solve bounded non-linear least squares

	int n_dim = model->getModelLightObj()->getNumSamples();
	nlopt::opt opt(nlopt::LD_MMA, n_dim);

	std::vector<double> lb(n_dim, 0);
	opt.set_lower_bounds(lb);

	opt.set_min_objective(funcSFSLight, this);

	opt.set_stopval(1e-4);
	opt.set_ftol_rel(1e-4);
	opt.set_ftol_abs(1e-4);
	opt.set_xtol_rel(1e-4);
	opt.set_xtol_abs(1e-4);


	for (int k_chan = 0; k_chan < 3; ++k_chan)
	{
		//std::cout<<I_temp.col(k_chan)<<"\n";
		brightness_sig_chan = I_temp.col(k_chan);
		std::vector<double> x(n_dim, 0.5);
		double minf;
		std::cout<<"Min value initialized: "<<minf<<"\n";
		nlopt::result result = opt.optimize(x, minf);
		std::cout<<"Min value optimized: "<<minf<<"\n";

		Eigen::Map<Eigen::VectorXd>cur_l_rec(&x[0], n_dim);
		Light_rec.col(k_chan) = cur_l_rec.cast<float>();
		std::cout<<"Return values of NLopt: "<<result<<"\n";
	}



	// standard non-linear least squares
    //Eigen::MatrixXf L_temp = Light_coeffs.transpose().eval()*Light_coeffs;
    //L_temp += Eigen::MatrixXf::Identity(light_coeffs_vec.size(), light_coeffs_vec.size());

    //Light_rec = L_temp.ldlt().solve(Light_coeffs.transpose().eval()*I_temp);

    // store new rho image computed by rho = I/(((4*pi/num_samples).*T)*L)
    // notice here we may not have per pixel rho, also need to store I

    // init rho image
    std::vector<cv::Mat> rho_img_split;
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(rhos[0])));
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(rhos[1])));
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(rhos[2])));
    cv::merge(rho_img_split, rho_img);

    // put rho computed from new light back to rho image
    Eigen::MatrixX3f rho_new;
    rho_new = (I.array() / ((T_coef*Light_rec)).array()).matrix();


    for (decltype(I_xy_vec.size()) i = 0; i < I_xy_vec.size(); ++i)
    {
        if (rho_new(i, 0) <= 1.0f)
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0] = rho_new(i, 0);
        else
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[0] = 1.0f;
        if (rho_new(i, 1) <= 1.0f)
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[1] = rho_new(i, 1);
        else
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[1] = 1.0f;
        if (rho_new(i, 2) <= 1.0f)
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[2] = rho_new(i, 2);
        else
            rho_img.at<cv::Vec3f>(I_xy_vec[i](1), I_xy_vec[i](0))[2] = 1.0f;
    }

    cv::imshow("rho_img", rho_img);

    cv::Mat brightness;
    cv::divide(photo, rho_img, brightness);
    cv::imshow("brightness", brightness);

    // give vertex on model its rho
    model->updateVertexRho();
    model->updateVertexBrightnessAndColor();

    // output some variables for debug here
	std::ofstream f_light_rec(model->getDataPath() + "/L_rec_init.mat");
	if (f_light_rec)
	{
		f_light_rec << Light_rec;
		f_light_rec.close();
	}

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

	T_coef.resize(0,0);
	brightness_sig_chan.resize(0);
    std::cout << "Compute init light finished\n";
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

    // we only compute guidance normal defined on face

    std::vector<int> faces_in_photo;
    model->findFacesInPhoto(faces_in_photo);

    // test if we get right faces
    std::cout << "Found number of faces in photo: " << faces_in_photo.size() << "\n";
    viewer->setShowModel(false);
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
    SAMPLE *model_samples = model->getModelLightObj()->getSamples();
    int num_samples = model->getModelLightObj()->getNumSamples();
    Eigen::MatrixX3f samples(num_samples, 3);
    for (int i = 0; i < num_samples; ++i)
    {
        samples.row(i) = model_samples[i].direction;
    }

    // build linear sys
    std::vector<Eigen::Triplet<float>> normal_coeffs;
    Eigen::VectorXf right_hand(3 * faces_in_photo.size());
    Eigen::SparseMatrix<float> Normal_coeffs(3 * faces_in_photo.size(), 3 * faces_in_photo.size());
	brightness_mat = brightness_in_photo;
	A_mat.resize(faces_in_photo.size(), 3);
	B_mat.resize(faces_in_photo.size(), 3);
	C_mat.resize(faces_in_photo.size(), 3);
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
            A(j) = (light_rec.col(j).array()*samples.col(0).array()*visb.array()).sum();
			A_mat(i, j) = A(j);
            //if (A(j) < 0) std::cout << "Normal coefficient is less than zero!\t" << "i j: " << i << "\t" << j << "\n";
            B(j) = (light_rec.col(j).array()*samples.col(1).array()*visb.array()).sum();
			B_mat(i, j) = B(j);
            //if (C(j) < 0) std::cout << "Normal coefficient is less than zero!t" << "i j: " << i << "\t" << j << "\n";
            C(j) = (light_rec.col(j).array()*samples.col(2).array()*visb.array()).sum();
			C_mat(i,j) = C(j);
            //if (C(j) < 0) std::cout << "Normal coefficient is less than zero!\t" << "i j: " << i << "\t" << j << "\n";
        }

        // equation for nx
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 0, lambda_sfs*A.dot(A)));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 1, lambda_sfs*B.dot(A)));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 2, lambda_sfs*C.dot(A)));

        // equation for ny
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 0, lambda_sfs*A.dot(B)));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 1, lambda_sfs*B.dot(B)));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 2, lambda_sfs*C.dot(B)));

        // equation for nz
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 0, lambda_sfs*A.dot(C)));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 1, lambda_sfs*B.dot(C)));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 2, lambda_sfs*C.dot(C)));

        right_hand(3 * i + 0) = (num_samples / 4 / M_PI)*(A.dot(brightness_in_photo.row(i)));
        right_hand(3 * i + 1) = (num_samples / 4 / M_PI)*(B.dot(brightness_in_photo.row(i)));
        right_hand(3 * i + 2) = (num_samples / 4 / M_PI)*(C.dot(brightness_in_photo.row(i)));

        // smooth term
        std::vector<int> *cur_face_adj = model->getFaceAdj(faces_in_photo[i]);
        int num_adj_faces_in_photo = 0; // the adjacent list may contains faces that are not visible in the photo


        for (decltype((*cur_face_adj).size()) j = 0; j < (*cur_face_adj).size(); ++j)
        {
            // find id of adjacent face in the faces_in_photo
            const std::vector<int>::iterator pos = std::find(faces_in_photo.begin(), faces_in_photo.end(), (*cur_face_adj)[j]);
            if (pos == faces_in_photo.end()) continue;
            const int idx = static_cast<int> (pos - faces_in_photo.begin());
            ++num_adj_faces_in_photo;

            // equation for nx
            normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * idx + 0, -lambda_smooth));

            // equation for ny
            normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * idx + 1, -lambda_smooth));

            // equation for nz
            normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * idx + 2, -lambda_smooth));
        }
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 0, lambda_smooth*num_adj_faces_in_photo));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 1, lambda_smooth*num_adj_faces_in_photo));
        normal_coeffs.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 2, lambda_smooth*num_adj_faces_in_photo));
    }

    Normal_coeffs.setFromTriplets(normal_coeffs.begin(), normal_coeffs.end());

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> chol(Normal_coeffs);
    Eigen::VectorXf new_face_in_photo_normal = chol.solve(right_hand);

    // output some variables for debug here
    std::ofstream f_rho_in_photo(model->getDataPath() + "/rho_in_photo.mat");
    if (f_rho_in_photo)
    {
        f_rho_in_photo << rho_in_photo;
        f_rho_in_photo.close();
    }

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

    std::ofstream f_new_normal(model->getDataPath() + "/new_normal.mat");
    if (f_new_normal)
    {
        f_new_normal << new_face_in_photo_normal;
        f_new_normal.close();
    }

    std::ofstream f_right_hand(model->getDataPath() + "/right_hand.mat");
    if (f_right_hand)
    {
        f_right_hand << right_hand;
        f_right_hand.close();
    }

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

    std::cout << "Compute new normal finished...\n";
}

float ImagePartAlg::sigmoid(float coef, float t)
{
    return 2.0f / (1.0f + exp(coef*t));
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