#include "Coarse.h"

Coarse::Coarse(const int id, const std::string path, const std::string name)
    :Model(id, path, name)
{

    std::cout << "Load Coarse finished...\n";

    cv::Mat photo_temp = cv::imread((path + std::to_string(id) + "/photo.png").c_str());
    cv::Mat mask_temp = cv::imread(path + std::to_string(id) + "/mask.png");

    //cv::cvtColor(photo_temp, photo_temp, CV_BGR2RGB);
    photo_temp.convertTo(photo, CV_32FC3);
    photo = 1e-4 + (photo / 255); // to ensure that I always larger than zero, important!
    cv::threshold(photo, photo, 1.0f, 1.0f, THRESH_TRUNC);

    cv::Mat photo_ps_temp = cv::imread((path + std::to_string(id) + "/photo_ps.png").c_str());
    photo_ps_temp.convertTo(photo_ps, CV_32FC3);
    photo_ps = 1e-4 + (photo_ps / 255);

    cv::cvtColor(mask_temp, mask, CV_BGR2GRAY);
    cv::threshold(mask, mask, 100, 255, cv::THRESH_BINARY);

    cv::imshow("photo", photo);
    cv::imshow("mask", mask);

    light_rec = Eigen::MatrixX3f::Constant(getModelLightObj()->getNumSamples(), 3, 0.05);
    std::vector<cv::Mat> rho_img_split;
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0.5)));
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0.5)));
    rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0.5)));
    cv::merge(rho_img_split, rho_img);

    //photo_temp.convertTo(rho_img, CV_32FC3);
    //rho_img = rho_img / 255;
    //rho_img = photo.clone(); // we use rho_img to do cluster now, so it can be updated each iteration

    std::vector<cv::Mat> normal_img_split;
    normal_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0.0)));
    normal_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0.0)));
    normal_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(1.0)));
    cv::merge(normal_img_split, normal_img);


    std::cout<<"Info of rho imag: "<< "cols: " << rho_img.cols << "\trows: " << rho_img.rows << "\tnchan: "
        << rho_img.channels() << "\n";

    cur_iter = 0;

    m_para = new MPara;

    use_SIFTFlow = false;

}

Coarse::~Coarse()
{
    delete m_para;
}

bool Coarse::getPixelLightCoeffs(int x, int y, Eigen::VectorXf &light_coeffs, Viewer *viewer, float &winx, float &winy)
{
    // this function computes a corresponding virtual point of an arbitrary pixel, its normal and visibility.

    // 0,0 is the left up corner as in cv::mat, and row major

    // compute xy coord in render image
    Eigen::Vector3f xy_model = model_to_img_trans.inverse()*Eigen::Vector3f((float)x, (float)y, 1.0);

    // get its face id
    winx = xy_model(0) / xy_model(2);
    winy = xy_model(1) / xy_model(2);
    int face_id = primitive_ID.at<int>((int)(winy + 0.5), (int)(winx + 0.5));

    cv::Mat temp = primitive_ID;

    if (face_id < 0) return false;

    // compute its world xyz coord
    Eigen::Vector3f xyz_model;
    if (!getWorldCoord(xy_model, xyz_model)) return false;

    viewer->addDrawablePoint(xyz_model(0), xyz_model(1), xyz_model(2), 1.0f, 0.0f, 0.0f);
    //std::cout << "find one point\n";

    // get its normal
    Eigen::Vector3f xyz_normal;
    getPtNormalInFace(xyz_model, face_id, xyz_normal);

    std::vector<bool> visb;
    computeVisbs(xyz_model, xyz_normal, visb);

    // now we have normal, visibility and all sample direction.
    // light coefficients Tj = (nx*Sjx+ny*Sjy+nz*Sjz)*vj
    // notice it should be a ds multiplier 4*pi/num_samples
    // to decrease precision loss we put it to I when solving lsq
    SAMPLE *light_samples = model_light->getSamples();
    int num_samples = model_light->getNumSamples();
    Eigen::MatrixX3f &s_mat = model_light->getSampleMatrix();

    light_coeffs = Eigen::VectorXf::Zero(num_samples);
    for (int i = 0; i < num_samples; ++i)
    {
        // the max(0, dot) has been considered in visb
        light_coeffs(i) = (xyz_normal.dot(s_mat.row(i)))*visb[i];
    }

    return true;
}

bool Coarse::getPixelVisbCoeffs(int x, int y, int &face_id, Eigen::VectorXf &visb_coeffs, Viewer *viewer, float &winx, float &winy, Eigen::Vector3f &cur_normal, Eigen::Vector3f &cur_pos)
{
    // this function computes a corresponding virtual point of an arbitrary pixel, its normal and visibility.

    // 0,0 is the left up corner as in cv::mat, and row major

    // compute xy coord in render image
    Eigen::Vector3f xy_model = model_to_img_trans.inverse()*Eigen::Vector3f((float)x, (float)y, 1.0);

    // get its face id
    winx = xy_model(0) / xy_model(2);
    winy = xy_model(1) / xy_model(2);


    if (use_SIFTFlow)
    {
        if (cur_iter > 0)
        {
            winx = crsp_Pimg.at<cv::Vec2d>(y, x)[0];
            winy = crsp_Pimg.at<cv::Vec2d>(y, x)[1];
            xy_model << winx , winy, 1.0f;
        }
    }




    winx = (winx < primitive_ID.cols && winx >= 0) ? (winx) : (0);
    winy = (winy < primitive_ID.rows && winy >= 0) ? (winy) : (0);

    face_id = primitive_ID.at<int>((int)(winy + 0.5), (int)(winx + 0.5));



    if (face_id < 0) return false;

    // compute its world xyz coord
    Eigen::Vector3f xyz_model;
    if (!getWorldCoord(xy_model, xyz_model)) return false;
    cur_pos = xyz_model;


    viewer->addDrawablePoint(xyz_model(0), xyz_model(1), xyz_model(2), 1.0f, 0.0f, 0.0f);
    //std::cout << "find one point\n";

    // get its normal
    Eigen::Vector3f xyz_normal;
    getPtNormalInFace(xyz_model, face_id, xyz_normal);
    cur_normal = xyz_normal;

    //visb_coeffs = Eigen::VectorXf::Ones(model_light->getNumSamples(), 1);
    computeVisbs(xyz_model, xyz_normal, visb_coeffs);
}

void Coarse::getCrspFromPhotoToRImg(int x, int y, float xy_rimg[2])
{
    Eigen::Vector3f xy_model = model_to_img_trans.inverse()*Eigen::Vector3f((float)x, (float)y, 1.0);

    // get its face id
    xy_rimg[0] = xy_model(0) / xy_model(2);
    xy_rimg[1] = xy_model(1) / xy_model(2);

    xy_rimg[0] = (xy_rimg[0] < primitive_ID.cols && xy_rimg[0] >= 0)?(xy_rimg[0]):(0);
    xy_rimg[1] = (xy_rimg[1] < primitive_ID.rows && xy_rimg[1] >= 0)?(xy_rimg[1]):(0);
}

bool Coarse::getCrspFaceIdFromPhotoToRImg(int x, int y, int &face_id)
{
    float xy_rimg[2] = { 0.0, 0.0 };
    getCrspFromPhotoToRImg(x, y, xy_rimg);

    face_id = primitive_ID.at<int>((int)(xy_rimg[1] + 0.5), (int)(xy_rimg[0] + 0.5));

    if (face_id < 0) return false;

    return true;
}

void Coarse::findFacesInPhoto(std::vector<int> &faces_in_photo)
{
    //faces_in_photo.clear();
    //uchar *mask_ptr = (uchar *)mask.data;

    //// for each pixel, Ii = (4*pi/num_samples)*Ti'*L. here we build T and I
    //for (int i = 0; i < mask.rows; ++i)
    //{ // OpenCV stores mat in row major from left top while MATLAB is col major from left top
    //    for (int j = 0; j < mask.cols; ++j)
    //    {
    //        if (mask_ptr[i*mask.cols + j] > 0)
    //        {
    //            int face_id = -1;
    //            if (!getCrspFaceIdFromPhotoToRImg(j, i, face_id)) continue;
    //            const std::vector<int>::iterator id = find(faces_in_photo.begin(), faces_in_photo.end(), face_id);
    //            if (id == faces_in_photo.end()) faces_in_photo.push_back(face_id);
    //        }
    //    }
    //}



    // for each pixel in primitiveID image
    faces_in_photo.clear();
    int *primitive_ID_ptr = (int *)primitive_ID.data;

    for (int i = 0; i < primitive_ID.rows; ++i)
    {
        for (int j = 0; j < primitive_ID.cols; ++j)
        {
            if (primitive_ID_ptr[i*primitive_ID.cols + j] >= 0)
            {
                Eigen::Vector3f xy = model_to_img_trans*Eigen::Vector3f((float)j, (float)i, 1.0);

                int xy_photo[2] = {int(xy(0)/xy(2) + 0.5), int(xy(1)/xy(2) + 0.5)};

                if (use_SIFTFlow)
                {
                    if (cur_iter > 0)
                    {
                        xy_photo[0] = int(crsp_rimg.at<cv::Vec2d>(i, j)[0] + 0.5);
                        xy_photo[1] = int(crsp_rimg.at<cv::Vec2d>(i, j)[1] + 0.5);
                    }
                }


                xy_photo[0] = (xy_photo[0] < mask.cols && xy_photo[0] >= 0)?(xy_photo[0]):(0);
                xy_photo[1] = (xy_photo[1] < mask.rows && xy_photo[1] >= 0)?(xy_photo[1]):(0);

                if (mask.at<uchar>(xy_photo[1], xy_photo[0]) > 0)
                {
                    // we must ensure this face can at least find a pixel in the mask
                    int face_id = primitive_ID_ptr[i*primitive_ID.cols + j];
                    const std::vector<int>::iterator id = find(faces_in_photo.begin(), faces_in_photo.end(), face_id);
                    if (id == faces_in_photo.end()) faces_in_photo.push_back(face_id);
                }
            }
        }
    }

    std::ofstream f(getOutputPath()+"/found_face.mat");
    if (f)
    {
        for (size_t i = 0; i < faces_in_photo.size(); ++i)
        {
            f << faces_in_photo[i] << "\n";
        }

        f.close();
    }
}

void Coarse::setModelNewNormal(Eigen::VectorXf &new_face_in_photo_normal, std::vector<int> &faces_in_photo)
{
    model_new_normals.clear();
    model_new_normals = model_face_normals;

    for (decltype(faces_in_photo.size()) i = 0; i < faces_in_photo.size(); ++i)
    {
        Eigen::Vector3f cur_normal;
        cur_normal << new_face_in_photo_normal[3 * i + 0],
            new_face_in_photo_normal[3 * i + 1],
            new_face_in_photo_normal[3 * i + 2];
        cur_normal.normalize();
        model_new_normals[3 * faces_in_photo[i] + 0] = cur_normal(0);
        model_new_normals[3 * faces_in_photo[i] + 1] = cur_normal(1);
        model_new_normals[3 * faces_in_photo[i] + 2] = cur_normal(2);
    }
}

void Coarse::loadS2ITransform()
{
    std::ifstream ifs(data_path + std::to_string(ID) + "/transform.mat");
    if (!ifs)
    {
        std::cout << "file doesn't exist or some other errors occurs.\n";
        return;
    }

    model_to_img_trans.resize(3, 3);
    int row_id = 0;
    std::string buffer;

    while (!ifs.eof())
    {
        getline(ifs, buffer);

        if (buffer.size() == 0)
            continue;

        {
            float m0;
            float m1;
            float m2;

            std::stringstream parser(buffer);

            parser >> m0 >> m1 >> m2;
            if (row_id < 3)
                model_to_img_trans.row(row_id) = Eigen::Vector3f(m0, m1, m2);
            ++row_id;
        }
    }

    std::cout << "Loading S2I Transform finished..\n" << model_to_img_trans << "\n";
}

void Coarse::updateVertexRho()
{
    size_t num_vertices = model_vertices.size() / 3;
    Eigen::MatrixX3f normal = (Eigen::Map<Eigen::Matrix3Xf>(&model_normals[0], 3, num_vertices)).transpose();

    uchar *mask_ptr = (uchar *)mask.data;
    cv::Vec3f mean_rho(0, 0, 0);
    int num_pixel_in_mask = 0;
    for (int i = 0; i < mask.rows; ++i)
    { // OpenCV stores mat in row major from left top while MATLAB is col major from left top
        for (int j = 0; j < mask.cols; ++j)
        {
            if (mask_ptr[i*mask.cols + j] > 0)
            {
                mean_rho += rho_img.at<cv::Vec3f>(i, j);
                num_pixel_in_mask += 1;
                break;
            }
        }
    }
    mean_rho = mean_rho / num_pixel_in_mask;

    model_rhos.clear();
    model_rhos.resize(3 * num_vertices);

    model_text_coord.resize(2 * num_vertices);
    for (size_t i = 0; i < num_vertices; ++i)
    {
        //if (v_vis_stat_in_r_img[i] == false)
        //{
        //    // if the vertex is invisible in the render image
        //    // we put the mean rho in these vertices
        //    model_rhos[3 * i + 2] = mean_rho[0];
        //    model_rhos[3 * i + 1] = mean_rho[1];
        //    model_rhos[3 * i + 0] = mean_rho[2];

        //    // set texture coord in rho image
        //    model_text_coord[2*i + 0] = 0.0f;
        //    model_text_coord[2*i + 1] = 0.0f;
        //}
        //else
        {
            float xy_photo[2];
            getCrspFromModelToPhoto(i, xy_photo);

            cv::Vec3f cur_rho = rho_img.at<cv::Vec3f>(int(xy_photo[1]+0.5), int(xy_photo[0]+0.5));
            model_rhos[3 * i + 2] = cur_rho[0];
            model_rhos[3 * i + 1] = cur_rho[1];
            model_rhos[3 * i + 0] = cur_rho[2];

            model_text_coord[2*i + 0] = xy_photo[0] / (float)mask.cols;
            model_text_coord[2*i + 1] = ((float)mask.rows - xy_photo[1]) / (float)mask.rows;
        }
    }
}

void Coarse::getCrspFromModelToPhoto(int v_id, float xy_photo[2])
{
    float v_pos[3] = { model_vertices[3 * v_id + 0], 
        model_vertices[3 * v_id + 1], 
        model_vertices[3 * v_id + 2] };

    float winx;
    float winy;

    getProjectPt(v_pos, winx, winy);

    Eigen::Vector3f xy = model_to_img_trans*Eigen::Vector3f((float)winx, (float)winy, 1.0);

    xy_photo[0] = xy(0) / xy(2);
    xy_photo[1] = xy(1) / xy(2);

    xy_photo[0] = (xy_photo[0] < mask.cols && xy_photo[0] >= 0)?(xy_photo[0]):(0);
    xy_photo[1] = (xy_photo[1] < mask.rows && xy_photo[1] >= 0)?(xy_photo[1]):(0);
}

void Coarse::updateVertexBrightnessAndColor()
{
    int num_samples = model_light->getNumSamples();
    SAMPLE *samples = model_light->getSamples();
    Eigen::MatrixX3f &S_mat = model_light->getSampleMatrix();
    float view_temp[3];
    renderer->getViewDirection(view_temp);
    Eigen::Vector3f view(view_temp[0], view_temp[1], view_temp[2]);
    Eigen::Matrix<float, 4, 3> &rho_specular = getRhoSpclr();
    Eigen::Matrix3f C_view;
    C_view << rho_specular(0,0)*view(0), rho_specular(0,1)*view(0), rho_specular(0,2)*view(0), 
        rho_specular(1,0)*view(1), rho_specular(1,1)*view(1), rho_specular(1,2)*view(1), 
        rho_specular(2,0)*view(2), rho_specular(2,1)*view(2), rho_specular(2,2)*view(2);
    Eigen::MatrixX3f S_C_view = ((S_mat*C_view).array()<0).select(0, S_mat*C_view);
    Eigen::MatrixX3f rho_s(S_C_view.rows(), 3);
    rho_s.col(0) = (S_C_view.col(0).array().pow(rho_specular(3, 0))).matrix();
    rho_s.col(1) = (S_C_view.col(1).array().pow(rho_specular(3, 1))).matrix();
    rho_s.col(2) = (S_C_view.col(2).array().pow(rho_specular(3, 2))).matrix();

    for (decltype(model_vertices.size()) i = 0; i < model_vertices.size() / 3; ++i)
    {
        Eigen::Vector3f cur_v_normal(model_normals[i * 3 + 0], model_normals[i * 3 + 1], model_normals[i * 3 + 2]);

        model_brightness[3 * i + 0] = 0.0f;
        model_brightness[3 * i + 1] = 0.0f;
        model_brightness[3 * i + 2] = 0.0f;

        float brightness_s_temp[3] = {0.0f, 0.0f, 0.0f};

        // foreach sample direction
        for (int k = 0; k < num_samples; ++k)
        {
            // check if light * normal > 0
            float dot = (float)samples[k].direction.dot(cur_v_normal);

            if (model_visbs[i][k] == true)
            {
                model_brightness[3 * i + 2] += dot*light_rec(k, 0);
                model_brightness[3 * i + 1] += dot*light_rec(k, 1);
                model_brightness[3 * i + 0] += dot*light_rec(k, 2);

                brightness_s_temp[2] += dot*light_rec(k, 0)*rho_s(k, 0);
                brightness_s_temp[1] += dot*light_rec(k, 1)*rho_s(k, 1);
                brightness_s_temp[0] += dot*light_rec(k, 2)*rho_s(k, 2);
            }
        }

        model_brightness[3 * i + 2] *= (float)(4 * M_PI / num_samples);
        model_brightness[3 * i + 1] *= (float)(4 * M_PI / num_samples);
        model_brightness[3 * i + 0] *= (float)(4 * M_PI / num_samples);

        brightness_s_temp[2] *=(float)(4 * M_PI / num_samples);
        brightness_s_temp[1] *=(float)(4 * M_PI / num_samples);
        brightness_s_temp[0] *=(float)(4 * M_PI / num_samples);

        //model_colors[3 * i + 0] = 
        model_rhos[3 * i + 0] = brightness_s_temp[2];
        //model_colors[3 * i + 1] = 
        model_rhos[3 * i + 1] = brightness_s_temp[1];
        //model_colors[3 * i + 2] = 
        model_rhos[3 * i + 2] = brightness_s_temp[0];
    }
}

void Coarse::drawNormal()
{
    if (model_new_normals.size() != model_faces.size())
    {
        std::cout << "Error: number of model new normal is different from that of model faces.\n";
        return;
    }

    double norm_length = renderer->sceneRadius() / 30;

    for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
    {
        int v0_id = model_faces[3 * i + 0];
        int v1_id = model_faces[3 * i + 1];
        int v2_id = model_faces[3 * i + 2];

        Eigen::Vector3f v0;
        v0 << model_vertices[3 * v0_id + 0], 
            model_vertices[3 * v0_id + 1], 
            model_vertices[3 * v0_id + 2];
        Eigen::Vector3f v1;
        v1 << model_vertices[3 * v1_id + 0], 
            model_vertices[3 * v1_id + 1], 
            model_vertices[3 * v1_id + 2];
        Eigen::Vector3f v2;
        v2 << model_vertices[3 * v2_id + 0], 
            model_vertices[3 * v2_id + 1], 
            model_vertices[3 * v2_id + 2];

        Eigen::Vector3f start_pt(v0 / 3 + v1 / 3 + v2 / 3);
        Eigen::Vector3f normal_dir;
        normal_dir << model_new_normals[3 * i + 0],
            model_new_normals[3 * i + 1],
            model_new_normals[3 * i + 2];
        Eigen::Vector3f end_pt(start_pt + norm_length*normal_dir);
        Eigen::Vector3f blue_color(0.0f, 0.0f, 1.0f);
        renderer->addDrawableLine(start_pt.data(), end_pt.data(), blue_color.data(), blue_color.data());
    }
}

void Coarse::rhoFromKMeans(int nCluster, Eigen::MatrixX3f &rhos_temp, std::vector<int> &cluster_label)
{
    // use x,y image coordinates and r,g,b color values as feature
    // assume we hav I_xy_vec
    if (cur_iter >= 0)
    {
        cv::Mat pixels(xy_in_mask.size(), 2, CV_32F);
        cv::Mat labels, centers;
        rhos_temp.resize(xy_in_mask.size(), 3);
        cv::Mat photo_xyz = photo.clone();
        cv::cvtColor(photo, photo_xyz, CV_BGR2XYZ);
        cluster_label.resize(xy_in_mask.size());

        for (size_t i = 0; i < xy_in_mask.size(); ++i)
        {
            Eigen::Vector2i cur_xy = xy_in_mask[i];
            cv::Vec3f cur_color = photo_xyz.at<cv::Vec3f>(cur_xy(1), cur_xy(0));
            //pixels.at<float>(i, 0) = cur_xy(0);
            //pixels.at<float>(i, 1) = cur_xy(1);
            //pixels.at<float>(i, 0) = cur_color[0];
            //pixels.at<float>(i, 1) = cur_color[1];
            //pixels.at<float>(i, 2) = cur_color[2];
            pixels.at<float>(i, 0) = cur_color[0] / (cur_color[0] + cur_color[1] + cur_color[2]);
            pixels.at<float>(i, 1) = cur_color[1] / (cur_color[0] + cur_color[1] + cur_color[2]);
        }
        
        cv::kmeans(pixels, nCluster, labels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
            3, cv::KMEANS_PP_CENTERS, centers);


        //std::cout<<"type of labels: "<<labels.type()<<"\n";
        std::cout<<"type of centers: "<<centers.type()<<"\n";
        std::cout<<"size of centers: "<<centers.size()<<"\n";
        std::cout<<centers<<"\n";

        //std::vector<cv::Mat> rho_img_split;
        //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0)));
        //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0)));
        //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0)));
        //cv::merge(rho_img_split, rho_img);

        for (size_t i = 0; i < xy_in_mask.size(); ++i)
        {
            //Eigen::Vector2i cur_xy = xy_in_mask[i];
            //cv::Vec<float, 3> cur_center = centers.row(labels.at<int>(i));
            //rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0)) = cur_center;
            //rhos_temp(i, 0) = cur_center[0];
            //rhos_temp(i, 1) = cur_center[1];
            //rhos_temp(i, 2) = cur_center[2];
            cluster_label[i] = labels.at<int>(i);
        }
    }

    if (cur_iter >= 10)
    {
        cv::Mat pixels(xy_in_mask.size(), 3, CV_32F);
        cv::Mat labels, centers;
        rhos_temp.resize(xy_in_mask.size(), 3);
        cv::Mat photo_xyz = rho_img.clone();
        //cv::cvtColor(photo, photo_xyz, CV_BGR2XYZ);
        cluster_label.resize(xy_in_mask.size());

        for (size_t i = 0; i < xy_in_mask.size(); ++i)
        {
            Eigen::Vector2i cur_xy = xy_in_mask[i];
            cv::Vec3f cur_color = photo_xyz.at<cv::Vec3f>(cur_xy(1), cur_xy(0));
            //pixels.at<float>(i, 0) = cur_xy(0);
            //pixels.at<float>(i, 1) = cur_xy(1);
            //pixels.at<float>(i, 0) = cur_color[0];
            //pixels.at<float>(i, 1) = cur_color[1];
            //pixels.at<float>(i, 2) = cur_color[2];
            pixels.at<float>(i, 0) = cur_color[0] ;
            pixels.at<float>(i, 1) = cur_color[1] ;
            pixels.at<float>(i, 2) = cur_color[2] ;
        }

        cv::kmeans(pixels, nCluster, labels, cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
            3, cv::KMEANS_PP_CENTERS, centers);


        //std::cout<<"type of labels: "<<labels.type()<<"\n";
        std::cout<<"type of centers: "<<centers.type()<<"\n";
        std::cout<<"size of centers: "<<centers.size()<<"\n";
        std::cout<<centers<<"\n";

        //std::vector<cv::Mat> rho_img_split;
        //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0)));
        //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0)));
        //rho_img_split.push_back(cv::Mat(mask.rows, mask.cols, CV_32F, cv::Scalar(0)));
        //cv::merge(rho_img_split, rho_img);

        for (size_t i = 0; i < xy_in_mask.size(); ++i)
        {
            //Eigen::Vector2i cur_xy = xy_in_mask[i];
            //cv::Vec<float, 3> cur_center = centers.row(labels.at<int>(i));
            //rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0)) = cur_center;
            //rhos_temp(i, 0) = cur_center[0];
            //rhos_temp(i, 1) = cur_center[1];
            //rhos_temp(i, 2) = cur_center[2];
            cluster_label[i] = labels.at<int>(i);
        }
    }
    //cv::imshow("rho image from kmeans", rho_img);

    //Eigen::Matrix<double, 5, 1> cnt_cluster= Eigen::Matrix<double, 5, 1>::Zero();
    //Eigen::Matrix<double, 5, 3> center_cluster = Eigen::Matrix<double , 5, 3>::Zero();
    //for (size_t i = 0; i < xy_in_mask.size(); ++i)
    //{
    //    Eigen::Vector2i cur_xy = xy_in_mask[i];

    //    switch(cluster_label[i])
    //    {
    //    case 0:
    //        ++cnt_cluster(0);
    //        center_cluster(0,0) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[0];
    //        center_cluster(0,1) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[1];
    //        center_cluster(0,2) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[2];
    //        break;
    //    case 1:
    //        ++cnt_cluster(1);
    //        center_cluster(1,0) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[0];
    //        center_cluster(1,1) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[1];
    //        center_cluster(1,2) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[2];
    //        break;
    //    case 2:
    //        ++cnt_cluster(2);
    //        center_cluster(2,0) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[0];
    //        center_cluster(2,1) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[1];
    //        center_cluster(2,2) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[2];
    //        break;
    //    case 3:
    //        ++cnt_cluster(3);
    //        center_cluster(3,0) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[0];
    //        center_cluster(3,1) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[1];
    //        center_cluster(3,2) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[2];
    //        break;
    //    case 4:
    //        ++cnt_cluster(4);
    //        center_cluster(4,0) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[0];
    //        center_cluster(4,1) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[1];
    //        center_cluster(4,2) += rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[2];
    //        break;
    //    default:
    //        break;
    //    }
    //}
    //center_cluster.col(0) = (center_cluster.col(0).array()/cnt_cluster.array()).matrix();
    //center_cluster.col(1) = (center_cluster.col(1).array()/cnt_cluster.array()).matrix();
    //center_cluster.col(2) = (center_cluster.col(2).array()/cnt_cluster.array()).matrix();

    //for (size_t i = 0; i < xy_in_mask.size(); ++i)
    //{
    //    //Eigen::Vector2i cur_xy = xy_in_mask[i];
    //    //cv::Vec<float, 3> cur_center = centers.row(labels.at<int>(i));
    //    //rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0)) = cur_center;
    //    //rhos_temp(i, 0) = cur_center[0];
    //    //rhos_temp(i, 1) = cur_center[1];
    //    //rhos_temp(i, 2) = cur_center[2];
    //    Eigen::Vector2i cur_xy = xy_in_mask[i];

    //    rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[0] = center_cluster(cluster_label[i], 0);
    //    rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[1] = center_cluster(cluster_label[i], 1);
    //    rho_img.at<cv::Vec3f>(cur_xy(1), cur_xy(0))[2] = center_cluster(cluster_label[i], 2);
    //}

    //cv::imshow("rho_img_after_cluster", rho_img);

    //std::ofstream f_rhos_tmep(getDataPath() + "/rhos_temp.mat");
    //if (f_rhos_tmep)
    //{
    //	f_rhos_tmep << rhos_temp;
    //	f_rhos_tmep.close();
    //}
}

void Coarse::computeSIFTFlow()
{
    std::cout<<"Compute SIFTFlow...\n";

    SIFTFlowWrapper siftflow;

    siftflow.doSIFTFlow(crsp_rimg, r_img, mask_rimg, photo, mask);
    siftflow.doSIFTFlow(crsp_Pimg, photo, mask, r_img, mask_rimg);



    std::ofstream f_debug(getOutputPath()+"/crsp_rimg.mat");
    if (f_debug)
    {
        f_debug << crsp_rimg;
        f_debug.close();
    }
    f_debug.open(getOutputPath()+"/crsp_Pimg.mat");
    if (f_debug)
    {
        f_debug << crsp_Pimg;
        f_debug.close();
    }

    std::cout<<"finished...\n";
}