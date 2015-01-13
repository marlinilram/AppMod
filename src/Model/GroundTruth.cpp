#include "GroundTruth.h"

Groundtruth::Groundtruth(Coarse *model)
{

    std::string gt_file_name = "gt.obj";

    if (!loadOBJ(gt_file_name, model->getDataPath()))
    {
        std::cerr << "Init model failed...\n";
    }

    shadow_on = true;


    model_light = model->getModelLightObj();

    std::cout<<"model_vertices size: "<<model_vertices.size()<<"\n";
    std::cout<<"model_faces size: "<<model_faces.size()<<"\n";
    ray_cast.passModel(model_vertices, model_faces);


    rho_specular = Eigen::MatrixXf::Zero(4, 3);
    rho_specular(3,0)=1;
    rho_specular(3,1)=1;
    rho_specular(3,2)=1;
}

void Groundtruth::computeErrorColor(Coarse *model)
{
    std::vector<float> *face_new_normal = model->getModelNewNormal();

    renderer->setShowModel(false);

    std::vector<float> error_list;
    float error_min = std::numeric_limits<float>::max();
    float error_max = 0;

    std::ofstream normal_error(model->getOutputPath()+"/normal_error.mat");
    if (normal_error)
    {
        for (size_t i = 0; i < model_faces.size()/3; ++i)
        {
            float error = 0.0f;
            error += (model_face_normals[3*i+0]-(*face_new_normal)[3*i+0])*(model_face_normals[3*i+0]-(*face_new_normal)[3*i+0]);
            error += (model_face_normals[3*i+1]-(*face_new_normal)[3*i+1])*(model_face_normals[3*i+1]-(*face_new_normal)[3*i+1]);
            error += (model_face_normals[3*i+2]-(*face_new_normal)[3*i+2])*(model_face_normals[3*i+2]-(*face_new_normal)[3*i+2]);
            error = sqrt(error);
            error_list.push_back(error);

            if (error < error_min)
                error_min = error;
            if (error > error_max)
                error_max = error;

            normal_error << model_face_normals[3*i+0]<<" "
            <<model_face_normals[3*i+1]<<" "
            <<model_face_normals[3*i+2]<<"\t"
            <<(*face_new_normal)[3*i+0]<<" "
            <<(*face_new_normal)[3*i+1]<<" "
            <<(*face_new_normal)[3*i+2]<<"\n";
            
           
        }

        normal_error.close();
    }


    for (size_t i = 0; i < model_faces.size()/3; ++i)
    {
        int v0_id = model_faces[3 * i + 0];
        int v1_id = model_faces[3 * i + 1];
        int v2_id = model_faces[3 * i + 2];
        Eigen::Vector3f v0(model_vertices[3 * v0_id + 0], model_vertices[3 * v0_id + 1], model_vertices[3 * v0_id + 2]);
        Eigen::Vector3f v1(model_vertices[3 * v1_id + 0], model_vertices[3 * v1_id + 1], model_vertices[3 * v1_id + 2]);
        Eigen::Vector3f v2(model_vertices[3 * v2_id + 0], model_vertices[3 * v2_id + 1], model_vertices[3 * v2_id + 2]);

        float c[3] = {(error_list[i]-error_min)/(error_max-error_min), 0, 0  };

        renderer->addDrawableTri(v0.data(), v1.data(), v2.data(), c,c,c);
    }



}