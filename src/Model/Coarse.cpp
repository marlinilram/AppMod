#include "Coarse.h"

Coarse::Coarse(const int id, const std::string path, const std::string name)
	:Model(id, path, name)
{

	std::cout << "Load Coarse finished...\n";

    cv::Mat photo_temp = cv::imread((path + std::to_string(id) + "/photo.png").c_str());
    cv::Mat mask_temp = cv::imread(path + std::to_string(id) + "/mask.png");

    //cv::cvtColor(photo_temp, photo_temp, CV_BGR2RGB);
    photo_temp.convertTo(photo, CV_32FC3);
    photo = photo / 255;

    cv::cvtColor(mask_temp, mask, CV_BGR2GRAY);
    cv::threshold(mask, mask, 100, 255, cv::THRESH_BINARY);

    cv::imshow("photo", photo);
    cv::imshow("mask", mask);
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

    light_coeffs = Eigen::VectorXf::Zero(num_samples);
    for (int i = 0; i < num_samples; ++i)
    {
        // the max(0, dot) has been considered in visb
        light_coeffs(i) = (xyz_normal.dot(light_samples[i].direction))*visb[i];
    }

    return true;
}

void Coarse::getCrspFromPhotoToRImg(int x, int y, float xy_rimg[2])
{
    Eigen::Vector3f xy_model = model_to_img_trans.inverse()*Eigen::Vector3f((float)x, (float)y, 1.0);

    // get its face id
    xy_rimg[0] = xy_model(0) / xy_model(2);
    xy_rimg[1] = xy_model(1) / xy_model(2);
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
    faces_in_photo.clear();
    uchar *mask_ptr = (uchar *)mask.data;

    // for each pixel, Ii = (4*pi/num_samples)*Ti'*L. here we build T and I
    for (int i = 0; i < mask.rows; ++i)
    { // OpenCV stores mat in row major from left top while MATLAB is col major from left top
        for (int j = 0; j < mask.cols; ++j)
        {
            if (mask_ptr[i*mask.cols + j] > 0)
            {
                int face_id = -1;
                if (!getCrspFaceIdFromPhotoToRImg(j, i, face_id)) continue;
                const std::vector<int>::iterator id = find(faces_in_photo.begin(), faces_in_photo.end(), face_id);
                if (id == faces_in_photo.end()) faces_in_photo.push_back(face_id);
            }
        }
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
    int num_vertices = model_vertices.size() / 3;
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
    for (int i = 0; i < num_vertices; ++i)
    {
        if (v_vis_stat_in_r_img[i] == false)
        {
            // if the vertex is invisible in the render image
            // we put the mean rho in these vertices
            model_rhos[3 * i + 2] = mean_rho[0];
            model_rhos[3 * i + 1] = mean_rho[1];
            model_rhos[3 * i + 0] = mean_rho[2];
        }
        else
        {
            float xy_photo[2];
            getCrspFromModelToPhoto(i, xy_photo);
            cv::Vec3f cur_rho = rho_img.at<cv::Vec3f>(int(xy_photo[1] + 0.5), int(xy_photo[0] + 0.5));
            model_rhos[3 * i + 2] = cur_rho[0];
            model_rhos[3 * i + 1] = cur_rho[1];
            model_rhos[3 * i + 0] = cur_rho[2];
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
}

void Coarse::updateVertexBrightnessAndColor()
{
    int num_samples = model_light->getNumSamples();
    SAMPLE *samples = model_light->getSamples();

    for (decltype(model_vertices.size()) i = 0; i < model_vertices.size() / 3; ++i)
    {
        Eigen::Vector3f cur_v_normal(model_normals[i * 3 + 0], model_normals[i * 3 + 1], model_normals[i * 3 + 2]);

        model_brightness[3 * i + 0] = 0.0f;
        model_brightness[3 * i + 1] = 0.0f;
        model_brightness[3 * i + 2] = 0.0f;

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
            }
        }

        model_brightness[3 * i + 2] *= (float)(4 * M_PI / num_samples);
        model_brightness[3 * i + 1] *= (float)(4 * M_PI / num_samples);
        model_brightness[3 * i + 0] *= (float)(4 * M_PI / num_samples);



        model_colors[3 * i + 0] = model_rhos[3 * i + 0] * model_brightness[3 * i + 0];
        model_colors[3 * i + 1] = model_rhos[3 * i + 1] * model_brightness[3 * i + 1];
        model_colors[3 * i + 2] = model_rhos[3 * i + 2] * model_brightness[3 * i + 2];
    }
}

void Coarse::drawNormal()
{
    if (model_new_normals.size() != model_faces.size())
    {
        std::cout << "Error: number of model new normal is different from that of model faces.\n";
        return;
    }

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
        Eigen::Vector3f end_pt(start_pt + 10*normal_dir);
        Eigen::Vector3f blue_color(0.0f, 0.0f, 1.0f);
        renderer->addDrawableLine(start_pt.data(), end_pt.data(), blue_color.data(), blue_color.data());
    }
}