#include "Model.h"


Model::Model(const int id, const std::string path, const std::string name) 
    : ID(id), data_path(path), output_path(path), file_name(name)
{
    if (!loadOBJ(file_name, path + std::to_string(id)))
    {
        std::cerr << "Init model failed...\n";
    }

    shadow_on = true;

    model_light = new ModelLight(1600, 40, 3);

    std::ofstream f_sample(getOutputPath() + "/sample.mat");
    if (f_sample)
    {
        f_sample << model_light->getSampleMatrix();
        f_sample.close();
    }

	//ray_cast = new Ray;
    //ray_cast->passModel(model_vertices, model_faces);
    ray_cast.passModel(model_vertices, model_faces);

    computeLight();

    rho_specular = Eigen::MatrixXf::Ones(4, 3);
}

Model::~Model()
{
    delete model_light;
    //delete ray_cast;
}

bool Model::loadOBJ(const std::string name, const std::string base_path)
{
    std::cout << "Loading OBJ file..." << name << std::endl;

    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err = tinyobj::LoadObj(shapes, materials, (base_path + "/" + name).c_str(), nullptr);

    if (!err.empty())
    {
        std::cerr << err << std::endl;
        return false;
    }

    model_vertices = shapes[0].mesh.positions;
    model_vertices_init = model_vertices;
    model_normals = shapes[0].mesh.normals;
    model_normals_init = model_normals;
    model_faces = shapes[0].mesh.indices;
    model_colors = Colorlist(model_vertices.size(), 0.5);
    model_rhos = model_colors;
    model_brightness = model_colors;

	std::cout<<"Building face adjacent list...\n";
    buildFaceAdj();

	std::cout<<"Building 1-ring neighbors list...\n";
    buildVertexShareFaces();

	std::cout<<"Building vertex adjacent list...\n";
    buildVertexAdj();

	std::cout<<"Computing bounding box...\n";
    computeBounds();

	std::cout<<"Computing face normals...\n";
    computeFaceNormal();

    //model_colors[model_faces[169 * 3 + 0] * 3 + 0] = 1.0;
    //model_colors[model_faces[169 * 3 + 1] * 3 + 0] = 1.0;
    //model_colors[model_faces[169 * 3 + 2] * 3 + 0] = 1.0;

    return true;
}

void Model::setInit()
{
    model_vertices = model_vertices_init;

    model_normals = model_normals_init;

    computeFaceNormal();


    //computeVertexNormal();

    updateBSPtree();

    computeLight();

}

void Model::exportOBJ(int cur_iter)
{
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	tinyobj::shape_t shape;

	shape.mesh.positions = model_vertices;
	shape.mesh.indices = model_faces;

	shapes.push_back(shape);

    char time_postfix[50];
    time_t current_time = time(NULL);
    strftime(time_postfix, sizeof(time_postfix), "_%Y%m%d-%H%M%S", localtime(&current_time));
    std::string file_time_postfix = time_postfix;

    std::string output_name = getOutputPath() + "/coarse_output" + file_time_postfix + ".obj";
	WriteObj(output_name, shapes, materials);

}

// computation part
// including compute light, visibilities towards sample directions

void Model::computeLight()
{
    int num_samples = model_light->getNumSamples();
    int num_sqrt_samples = model_light->getSqrtNumSamples();
    int num_channels = model_light->getNumChannels();
    SAMPLE *samples = model_light->getSamples();
    int perc = 0;
    model_visbs.clear();
	//renderer->setCheckVisbStatus(true);

    // for each model vertex
    for (decltype(model_vertices.size()) i = 0; i < model_vertices.size()/3; ++i)
    {
        Eigen::Vector3f cur_v_normal(model_normals[i * 3 + 0], model_normals[i * 3 + 1], model_normals[i * 3 + 2]);
        Eigen::Vector3f cur_v_pos(model_vertices[i * 3 + 0], model_vertices[i * 3 + 1], model_vertices[i * 3 + 2]);
        Eigen::Vector3d ray_start = cur_v_pos.cast<double>() + 0.02*cur_v_normal.cast<double>();

        float brightness[3] = { 0, 0, 0 };
        std::vector<bool> cur_v_visb;

        // foreach sample direction
        for (int k = 0; k < num_samples; ++k)
        {
            // check if light * normal > 0
            float dot = (float)samples[k].direction.dot(cur_v_normal);
            Eigen::Vector3d ray_end = cur_v_pos.cast<double>() + 10000*samples[k].direction.cast<double>();

            if (dot > 0.0)
            {

                if (shadow_on)
                {
                    //if (!intersectModel(ray_start, samples[k].direction, model_vertices, model_faces))
					//if (renderer->checkVertexVisbs(i, this, samples[k].direction))
                    if (ray_cast.intersectModel(ray_start, ray_end))
                    {
                        brightness[0] += dot*(float)Light(samples[k].theta, samples[k].phi, 0);
                        brightness[1] += dot*(float)Light(samples[k].theta, samples[k].phi, 1);
                        brightness[2] += dot*(float)Light(samples[k].theta, samples[k].phi, 2);
                        cur_v_visb.push_back(true);
                    }
                    else { cur_v_visb.push_back(false); }
                }
                else
                {
                    brightness[0] += dot*(float)Light(samples[k].theta, samples[k].phi, 0);
                    brightness[1] += dot*(float)Light(samples[k].theta, samples[k].phi, 1);
                    brightness[2] += dot*(float)Light(samples[k].theta, samples[k].phi, 2);
                    cur_v_visb.push_back(true);
                }
            }
            else cur_v_visb.push_back(false);
        }

        model_visbs.push_back(cur_v_visb);

        brightness[0] *= (float)(4 * M_PI / num_samples);
        brightness[1] *= (float)(4 * M_PI / num_samples);
        brightness[2] *= (float)(4 * M_PI / num_samples);

        model_brightness.push_back(brightness[0]);
        model_brightness.push_back(brightness[1]);
        model_brightness.push_back(brightness[2]);

        model_colors[3 * i + 0] *= (float)brightness[0];
        model_colors[3 * i + 1] *= (float)brightness[1];
        model_colors[3 * i + 2] *= (float)brightness[2];

        if (int(i*100.0f / (model_vertices.size() / 3)) > perc)
        {
            perc = int(i*100.0f / (model_vertices.size() / 3));
            std::cout << perc << "...";
        }
        //else std::cout << "...";
    }
	std::cout<<"\n";

	//std::ofstream f_v(getDataPath() + "/visbs.mat");
	//if (f_v)
	//{
 //       for (size_t k = 0; k < model_visbs.size(); ++k)
 //       {
	//	for (size_t i = 0; i < model_visbs[k].size(); ++i)
	//	{
	//		f_v	<< model_visbs[k][i] << " ";
	//	}
 //       f_v << "\n";
 //       }

	//	f_v.close();
	//}
	//renderer->setCheckVisbStatus(false);
}

void Model::computeModelVisbs()
{
    int num_samples = model_light->getNumSamples();
    int num_sqrt_samples = model_light->getSqrtNumSamples();
    int num_channels = model_light->getNumChannels();
    SAMPLE *samples = model_light->getSamples();
    int perc = 0;
    model_visbs.clear();
    //renderer->setCheckVisbStatus(true);

    // for each model vertex
    for (decltype(model_vertices.size()) i = 0; i < model_vertices.size() / 3; ++i)
    {
        Eigen::Vector3f cur_v_normal(model_normals[i * 3 + 0], model_normals[i * 3 + 1], model_normals[i * 3 + 2]);
        Eigen::Vector3f cur_v_pos(model_vertices[i * 3 + 0], model_vertices[i * 3 + 1], model_vertices[i * 3 + 2]);
        Eigen::Vector3d ray_start = cur_v_pos.cast<double>() + 0.02*cur_v_normal.cast<double>();

        float brightness[3] = { 0, 0, 0 };
        std::vector<bool> cur_v_visb;

        // foreach sample direction
        for (int k = 0; k < num_samples; ++k)
        {
            // check if light * normal > 0
            float dot = (float)samples[k].direction.dot(cur_v_normal);
            Eigen::Vector3d ray_end = cur_v_pos.cast<double>() + 10000 * samples[k].direction.cast<double>();

            if (dot > 0.0)
            {

                if (shadow_on)
                {
                    //if (!intersectModel(ray_start, samples[k].direction, model_vertices, model_faces))
                    //if (renderer->checkVertexVisbs(i, this, samples[k].direction))
                    if (ray_cast.intersectModel(ray_start, ray_end))
                    {
                        cur_v_visb.push_back(true);
                    }
                    else { cur_v_visb.push_back(false); }
                }
                else
                {
                    cur_v_visb.push_back(true);
                }
            }
            else cur_v_visb.push_back(false);
        }

        model_visbs.push_back(cur_v_visb);

        if (int(i*100.0f / (model_vertices.size() / 3)) > perc)
        {
            perc = int(i*100.0f / (model_vertices.size() / 3));
            std::cout << perc << "...";
        }
        //else std::cout << "...";
    }
    std::cout << "\n";
}

void Model::computeVisbs(Eigen::Vector3f &point, Eigen::Vector3f &normal, std::vector<bool> &visb)
{
    // point should be (original point + 0.02*normal at this point)
    int num_samples = model_light->getNumSamples();
    SAMPLE *samples = model_light->getSamples();
    Eigen::Vector3d ray_start = point.cast<double>() + 0.05*normal.cast<double>();
    visb.clear();

    for (int k = 0; k < num_samples; ++k)
    {
        float dot = (float)samples[k].direction.dot(normal);
        Eigen::Vector3d ray_end = point.cast<double>() + 10000*samples[k].direction.cast<double>();

        if (dot > 0.0)
        {
            // check if the light is blocked by the model itself
            // foreach model face
            if (shadow_on)
            {
                //if (!intersectModel(ray_start, samples[k].direction, model_vertices, model_faces))
                if (ray_cast.intersectModel(ray_start, ray_end))
                {
                    visb.push_back(true);
                }
                else visb.push_back(false);
            }
            else visb.push_back(true);
        }
        else visb.push_back(false);
    }
}

void Model::computeVisbs(Eigen::Vector3f &point, Eigen::Vector3f &normal, Eigen::VectorXf &visb)
{
    // point should be (original point + 0.02*normal at this point)
    int num_samples = model_light->getNumSamples();
    SAMPLE *samples = model_light->getSamples();
    Eigen::Vector3d ray_start = point.cast<double>() + 0.02*normal.cast<double>();
    visb = Eigen::VectorXf(num_samples);

    for (int k = 0; k < num_samples; ++k)
    {
        float dot = (float)samples[k].direction.dot(normal);
        Eigen::Vector3d ray_end = point.cast<double>() + 10000*samples[k].direction.cast<double>();

        if (dot > 0.0)
        {
            // check if the light is blocked by the model itself
            // foreach model face
            if (shadow_on)
            {
                //if (!intersectModel(ray_start, samples[k].direction, model_vertices, model_faces))
                if (ray_cast.intersectModel(ray_start, ray_end))
                {
                    visb(k) = 1.0;
                }
                else visb(k) = 0.0;
            }
            else visb(k) = 1.0;
        }
        else visb(k) = 0.0;
    }
}

void Model::computeVisbs(int face_id, std::vector<bool> &visb)
{
    // in obj if a face corresponds to v0 v1 v2
    // its normal is (v1-v0) cross (v2-v1)

    int v0_id = model_faces[3 * face_id + 0];
    int v1_id = model_faces[3 * face_id + 1];
    int v2_id = model_faces[3 * face_id + 2];
    Eigen::Vector3f v0(model_vertices[3 * v0_id + 0], model_vertices[3 * v0_id + 1], model_vertices[3 * v0_id + 2]);
    Eigen::Vector3f v1(model_vertices[3 * v1_id + 0], model_vertices[3 * v1_id + 1], model_vertices[3 * v1_id + 2]);
    Eigen::Vector3f v2(model_vertices[3 * v2_id + 0], model_vertices[3 * v2_id + 1], model_vertices[3 * v2_id + 2]);
    Eigen::Vector3f v_center = v0 / 3 + v1 / 3 + v2 / 3;

    Eigen::Vector3f normal = (v1 - v0).cross(v2 - v1);
    normal.normalize();

    computeVisbs(v_center, normal, visb);
}

void Model::computeVisbs(int face_id, Eigen::VectorXf &visb)
{
    // in obj if a face corresponds to v0 v1 v2
    // its normal is (v1-v0) cross (v2-v1)

    int v0_id = model_faces[3 * face_id + 0];
    int v1_id = model_faces[3 * face_id + 1];
    int v2_id = model_faces[3 * face_id + 2];
    Eigen::Vector3f v0(model_vertices[3 * v0_id + 0], model_vertices[3 * v0_id + 1], model_vertices[3 * v0_id + 2]);
    Eigen::Vector3f v1(model_vertices[3 * v1_id + 0], model_vertices[3 * v1_id + 1], model_vertices[3 * v1_id + 2]);
    Eigen::Vector3f v2(model_vertices[3 * v2_id + 0], model_vertices[3 * v2_id + 1], model_vertices[3 * v2_id + 2]);
    Eigen::Vector3f v_center = v0 / 3 + v1 / 3 + v2 / 3;

    Eigen::Vector3f normal = (v1 - v0).cross(v2 - v1);
    normal.normalize();

    computeVisbs(v_center, normal, visb);
}

// parameter transfer with Viewer
// including send render model to viewer
// get rendered image, depth image, primitive id image from viewer
// get camera parameter from viewer
// get vertices visibility status in rendered image

void Model::passData(VectorF &vertices, Facelist &faces, Colorlist &colors)
{
    vertices = model_vertices;
    faces = model_faces;
    colors = model_colors;
}

void Model::passRenderImgInfo(cv::Mat &zImg, cv::Mat &primitiveID, cv::Mat &rImg)
{
    z_img = zImg.clone();
    primitive_ID = primitiveID.clone();
    r_img = rImg.clone();
}

void Model::passCameraPara(float c_modelview[16], float c_projection[16], int c_viewport[4])
{

    m_modelview = Eigen::Map<Eigen::Matrix4f>(c_modelview, 4, 4);

    m_projection = Eigen::Map<Eigen::Matrix4f>(c_projection, 4, 4);

    m_inv_modelview_projection = (m_projection*m_modelview).inverse();

    m_viewport = Eigen::Map<Eigen::Vector4i>(c_viewport, 4, 1);

}

void Model::passVerticesVisbleStatus(std::vector<bool> &visble)
{
    v_vis_stat_in_r_img.clear();
    v_vis_stat_in_r_img = visble;
}

// sub computation part
// including forward projection from model to rendered image
// backward projection from rendered image to world coordinates
// compute barycenter coordinates in a triangle
// get normal of an arbitrary point in a certain face
// build face adjacent list
// build vertex one-ring face list
// build vertex adjacent list
// compute bounding box of model


bool Model::getProjectPt(float object_coord[3], float &winx, float &winy)
{
    Eigen::Vector4f in(object_coord[0], object_coord[1], object_coord[2], 1.0f);
    Eigen::Vector4f out = m_projection*m_modelview*in;

    if (out(3) == 0.0)
        return false;
    out(0) = out(0) / out(3);
    out(1) = out(1) / out(3);
    winx = m_viewport(0) + m_viewport(2)*(out(0) + 1) / 2;
    winy = m_viewport(1) + m_viewport(3)*(out(1) + 1) / 2;
}

bool Model::getUnprojectPt(float winx, float winy, float winz, float object_coord[3])
{
    // unproject visible point in render image back to world

    //Transformation of normalized coordinates between -1 and 1

    Eigen::Vector4f in((winx - (float)m_viewport(0)) / (float)m_viewport(2) * 2.0 - 1.0,
                       (winy - (float)m_viewport(1)) / (float)m_viewport(3) * 2.0 - 1.0,
                       2.0*winz - 1.0,
                       (float)1.0);

    //Objects coordinates
    Eigen::Vector4f out = m_inv_modelview_projection*in;
    if (out(3) == 0.0)
        return false;
    out(3) = (float)1.0 / out(3);
    object_coord[0] = out(0) * out(3);
    object_coord[1] = out(1) * out(3);
    object_coord[2] = out(2) * out(3);
    return true;
}

bool Model::getWorldCoord(Eigen::Vector3f rimg_coord, Eigen::Vector3f &w_coord)
{
    // passed in screen coordinates is homogeneous
    rimg_coord(2) = (float)1.0 / rimg_coord(2);
    float winx = rimg_coord(0)*rimg_coord(2);
    float winy = rimg_coord(1)*rimg_coord(2);

    float winz = z_img.at<float>((int)(winy + 0.5), (int)(winx + 0.5));

    float obj_coord[3];

    if(!getUnprojectPt(winx, winy, winz, obj_coord))
        return false;

    w_coord = Eigen::Map<Eigen::Vector3f>(obj_coord, 3, 1);

    return true;
}

void Model::computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3])
{
    // v presents triangle vertex, pt locate inside triangle
    // pt[0]->x pt[1]->y pt[2]->z
    // v0[0]->x1 v0[1]->y1 v0[2]->z1
    // v1[0]->x2 v1[1]->y2 v1[2]->z2
    // v2[0]->x3 v2[1]->y3 v2[2]->z3
    Eigen::Vector3f e0(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
    Eigen::Vector3f e1(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);
    Eigen::Vector3f e2(pt[0] - v0[0], pt[1] - v0[1], pt[2] - v0[2]);

    float d00 = e0.dot(e0);
    float d01 = e0.dot(e1);
    float d11 = e1.dot(e1);
    float d20 = e2.dot(e0);
    float d21 = e2.dot(e1);
    float denom = d00*d11 - d01*d01;

    lambd[1] = (d11*d20 - d01*d21) / denom;

    lambd[2] = (d00*d21 - d01*d20) / denom;

    lambd[0] = 1.0f - lambd[1] - lambd[2];

}

void Model::getPtNormalInFace(Eigen::Vector3f &pt, int face_id, Eigen::Vector3f &normal)
{
    int v0_id = model_faces[3 * face_id + 0];
    int v1_id = model_faces[3 * face_id + 1];
    int v2_id = model_faces[3 * face_id + 2];

    Eigen::Vector3f v0( model_vertices[3 * v0_id + 0], 
                    model_vertices[3 * v0_id + 1], 
                    model_vertices[3 * v0_id + 2] );

    Eigen::Vector3f v1( model_vertices[3 * v1_id + 0], 
                    model_vertices[3 * v1_id + 1], 
                    model_vertices[3 * v1_id + 2] );

    Eigen::Vector3f v2( model_vertices[3 * v2_id + 0], 
                    model_vertices[3 * v2_id + 1], 
                    model_vertices[3 * v2_id + 2] );

    float lamd[3];
    computeBaryCentreCoord(pt.data(), v0.data(), v1.data(), v2.data(), lamd);

    Eigen::Vector3f n0;
    n0<< model_normals[3 * v0_id + 0],
         model_normals[3 * v0_id + 1],
         model_normals[3 * v0_id + 2];

    Eigen::Vector3f n1;
    n1 << model_normals[3 * v1_id + 0],
          model_normals[3 * v1_id + 1],
          model_normals[3 * v1_id + 2];

    Eigen::Vector3f n2;
    n2 << model_normals[3 * v2_id + 0],
          model_normals[3 * v2_id + 1],
          model_normals[3 * v2_id + 2];

    normal = lamd[0] * n0 + lamd[1] * n1 + lamd[2] * n2;
    pt = lamd[0] * v0 + lamd[1] * v1 + lamd[2] * v2;
}

void Model::buildFaceAdj()
{
    model_faces_adj.clear();
    for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
    {
        std::vector<int> cur_face_adj;
        // traverse the face list to find adjacent faces
        for (decltype(model_faces.size()) j = 0; j < model_faces.size() / 3; ++j)
        {
            // if i == j its the same face, ignore
            if (i != j)
            {
                if (shareEdge((int)i, (int)j))
                    cur_face_adj.push_back((int)j);
            }
        }
        model_faces_adj.push_back(cur_face_adj);
    }
}

bool Model::shareEdge(int face0, int face1)
{
    unsigned int face0_vertices[3] = { model_faces[3 * face0 + 0], model_faces[3 * face0 + 1], model_faces[3 * face0 + 2] };
    unsigned int face1_vertices[3] = { model_faces[3 * face1 + 0], model_faces[3 * face1 + 1], model_faces[3 * face1 + 2] };

    int tag = 0;
    for (int i = 0; i < 3; ++i)
    {
        
        for (int j = 0; j < 3; ++j)
        {
            if (face0_vertices[i] == face1_vertices[j])
            {
                ++tag;
                break;
            }
        }
        if (tag >= 2)
            return true;
    }
    return false;
}

void Model::buildVertexShareFaces()
{
    model_vertices_share_faces.clear();
    model_vertices_share_faces.resize(model_vertices.size() / 3);
    for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
    {
        model_vertices_share_faces[model_faces[3 * i + 0]].push_back(i);
        model_vertices_share_faces[model_faces[3 * i + 1]].push_back(i);
        model_vertices_share_faces[model_faces[3 * i + 2]].push_back(i);
    }

	//std::ofstream f_vert_share_face(getDataPath() + "/vert_share_face.mat");
	//if (f_vert_share_face)
	//{
	//	for (size_t i = 0; i < model_vertices_share_faces.size(); ++i)
	//	{
	//		for (size_t j = 0; j < model_vertices_share_faces[i].size(); ++j)
	//		{
	//			f_vert_share_face << model_vertices_share_faces[i][j]<<"\t";
	//		}
	//		f_vert_share_face <<"\n";
	//	}
	//	f_vert_share_face.close();
	//}
}

void Model::buildVertexAdj()
{
    model_vertex_adj.clear();
    model_vertex_adj.resize(model_vertices.size() / 3);
    for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
    {
        model_vertex_adj[model_faces[3 * i + 0]].push_back(model_faces[3 * i + 1]);
        model_vertex_adj[model_faces[3 * i + 0]].push_back(model_faces[3 * i + 2]);
        model_vertex_adj[model_faces[3 * i + 1]].push_back(model_faces[3 * i + 2]);
        model_vertex_adj[model_faces[3 * i + 1]].push_back(model_faces[3 * i + 0]);
        model_vertex_adj[model_faces[3 * i + 2]].push_back(model_faces[3 * i + 0]);
        model_vertex_adj[model_faces[3 * i + 2]].push_back(model_faces[3 * i + 1]);
    }

    // adj list is redundant, we need to sort it and delete duplicated element
    for (auto &i : model_vertex_adj)
    {
        std::sort(i.begin(), i.end());
        std::vector<int>::iterator iter = std::unique(i.begin(), i.end());
        i.erase(iter, i.end());
        i.shrink_to_fit();
    }
}

void Model::buildFaceListCompact()
{
    for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
    {
        std::vector<int> f;
        f.push_back(model_faces[3 * i + 0]);
        f.push_back(model_faces[3 * i + 1]);
        f.push_back(model_faces[3 * i + 2]);

        std::sort(f.begin(), f.end());
        model_faces_compact.push_back(Eigen::Map<Eigen::Vector3i>(&f[0]));
    }
}

void Model::computeBounds()
{
    model_bounds.minX = std::numeric_limits<float>::max();
    model_bounds.maxX = std::numeric_limits<float>::min();
    model_bounds.minY = std::numeric_limits<float>::max();
    model_bounds.maxY = std::numeric_limits<float>::min();
    model_bounds.minZ = std::numeric_limits<float>::max();
    model_bounds.maxZ = std::numeric_limits<float>::min();

    for (decltype(model_vertices.size()) i = 0; i < model_vertices.size() / 3; ++i)
    {
        float x = model_vertices[3 * i + 0];
        float y = model_vertices[3 * i + 1];
        float z = model_vertices[3 * i + 2];

        if (x < model_bounds.minX) model_bounds.minX = x;
        if (x > model_bounds.maxX) model_bounds.maxX = x;
        if (y < model_bounds.minY) model_bounds.minY = y;
        if (y > model_bounds.maxY) model_bounds.maxY = y;
        if (z < model_bounds.minZ) model_bounds.minZ = z;
        if (z > model_bounds.maxZ) model_bounds.maxZ = z;
    }
}

void Model::computeFaceNormal()
{
    model_face_normals.clear();

    for (decltype(model_faces.size()) i = 0; i < model_faces.size() / 3; ++i)
    {
        Eigen::Vector3f v0;
        v0 << model_vertices[3 * model_faces[3 * i + 0] + 0],
            model_vertices[3 * model_faces[3 * i + 0] + 1],
            model_vertices[3 * model_faces[3 * i + 0] + 2];

        Eigen::Vector3f v1;
        v1 << model_vertices[3 * model_faces[3 * i + 1] + 0],
            model_vertices[3 * model_faces[3 * i + 1] + 1],
            model_vertices[3 * model_faces[3 * i + 1] + 2];

        Eigen::Vector3f v2;
        v2 << model_vertices[3 * model_faces[3 * i + 2] + 0],
            model_vertices[3 * model_faces[3 * i + 2] + 1],
            model_vertices[3 * model_faces[3 * i + 2] + 2];

        Eigen::Vector3f edge_0 = v1 - v0;
        Eigen::Vector3f edge_1 = v2 - v1;

        Eigen::Vector3f face_normal = edge_0.cross(edge_1);
        face_normal.normalize();
        model_face_normals.push_back(face_normal(0));
        model_face_normals.push_back(face_normal(1));
        model_face_normals.push_back(face_normal(2));
    }
}

void Model::computeVertexNormal()
{
    for (size_t i = 0; i < model_vertices.size()/3; ++i)
    {
        std::vector<int> &cur_1_ring_face = model_vertices_share_faces[i];

        Eigen::Vector3f cur_v_normal(0.0f,0.0f,0.0f);
        for (size_t j = 0; j < cur_1_ring_face.size(); ++j)
        {
            cur_v_normal(0) += model_face_normals[3*cur_1_ring_face[j] + 0];
            cur_v_normal(1) += model_face_normals[3*cur_1_ring_face[j] + 1];
            cur_v_normal(2) += model_face_normals[3*cur_1_ring_face[j] + 2];
        }
        cur_v_normal = cur_v_normal / cur_1_ring_face.size();
        cur_v_normal.normalized();

        model_normals[3*i + 0] = cur_v_normal(0);
        model_normals[3*i + 1] = cur_v_normal(1);
        model_normals[3*i + 2] = cur_v_normal(2);
    }

}