#include "NormalTransfer.h"
#include "Model.h"

#include "Solver.h"
#include "ARAP.h"
#include "NormalConstraint.h"

#include <cv.h>

NormalTransfer::NormalTransfer()
{
  actors.push_back(GLActor(ML_POINT, 5.0f));
  actors.push_back(GLActor(ML_MESH, 1.0f));
  actors.push_back(GLActor(ML_LINE, 1.0f));

  solver = nullptr;
  arap = nullptr;
  normal_constraint = nullptr;
}

NormalTransfer::~NormalTransfer()
{

}

void NormalTransfer::prepareNewNormal(std::shared_ptr<Model> model)
{
  // build new normal list for deformation
  // for all visible faces in the image, we set a new normal
  // every pixel has a normal assigned to it, we average all the normals
  // for one face

  actors[0].clearElement();
  actors[1].clearElement();
  actors[2].clearElement();

  cv::Mat photo_normal;

  cv::FileStorage fs(model->getDataPath() + "/normal.xml", cv::FileStorage::READ);
  fs["normal"] >> photo_normal;
  cv::imshow("normal_img", photo_normal);

  // the origin of coordinate system of normal image
  // is top left corner
  // channel 1 -> n_y
  // channel 2 -> n_x
  // channel 3 -> n_z
  // screen system origin bottom left corner

  cv::Mat &primitive_id_img = model->getPrimitiveIDImg();
  int *primitive_id_ptr = (int *)primitive_id_img.data;

  std::map<int, Vector3f> normal_map;
  std::map<int, Vector3f>::iterator iter_map;

  for (int i = 0; i < primitive_id_img.rows; ++i)
  {
    for (int j = 0; j < primitive_id_img.cols; ++j)
    {
      int face_id = primitive_id_ptr[i * primitive_id_img.cols + j];
      if (face_id >= 0)
      {
        int x = j;
        int y = i;
        cv::Vec3f normal_in_photo = photo_normal.at<cv::Vec3f>(y, x);
        Vector3f new_normal;
        new_normal << normal_in_photo[1],
                      -normal_in_photo[0],
                      -normal_in_photo[2];
        model->getUnprojectVec(new_normal);

        iter_map = normal_map.find(face_id);
        if (iter_map != normal_map.end())
        {
          iter_map->second += new_normal;
        }
        else
        {
          normal_map[face_id] = new_normal;
        }
      }
    }
  }

  std::vector<int> faces_in_photo;
  std::vector<Vector3f> faces_new_normal;
  for (iter_map = normal_map.begin(); iter_map != normal_map.end(); ++iter_map)
  {
    faces_in_photo.push_back(iter_map->first);
    faces_new_normal.push_back(iter_map->second.normalized());

    Vector3f start;
    model->getShapeFaceCenter(iter_map->first, start.data());
    Vector3f end = iter_map->second.normalized();
    end = start + 0.1*end;
    actors[2].addElement(start[0], start[1], start[2], 1.0, 0.0, 0.0);
    actors[2].addElement(end[0], end[1], end[2], 1.0, 0.0, 0.0);
  }

  NormalList new_normals;// = model->getShapeFaceNormal();
  for (size_t i = 0; i < faces_in_photo.size(); ++i)
  {
    int f_id = faces_in_photo[i];
    //new_normals[3 * f_id + 0] = faces_new_normal[i](0);
    //new_normals[3 * f_id + 1] = faces_new_normal[i](1);
    //new_normals[3 * f_id + 2] = faces_new_normal[i](2);
    new_normals.push_back(faces_new_normal[i](0));
    new_normals.push_back(faces_new_normal[i](1));
    new_normals.push_back(faces_new_normal[i](2));
  }

  FaceList face_list = model->getShapeFaceList();
  VertexList vertex_list = model->getShapeVertexList();
  NormalList normal_list = model->getShapeNormalList();
  AdjList vertex_shared_faces = model->getShapeVertexShareFaces();
  AdjList adj_list = model->getShapeVertexAdjList();

  if (!solver)
  {
    solver.reset(new Solver);
  }
  if (!normal_constraint)
  {
    normal_constraint.reset(new NormalConstraint);
    // add constraints to solver
    solver->addConstraint(normal_constraint);
  }
  if (!arap)
  {
    arap.reset(new ARAP);
    // add constraints to solver
    solver->addConstraint(arap);
  }

  solver->problem_size = vertex_list.size();
  solver->P_Opt = Eigen::Map<VectorXf>(&(vertex_list)[0], (vertex_list).size());

  arap->setSolver(solver);
  arap->initConstraint(vertex_list, face_list,adj_list);
  arap->setLamdARAP(5.0f);

  normal_constraint->setSolver(solver);
  // use partial normal
  normal_constraint->initMatrix(face_list, vertex_list, vertex_shared_faces, normal_list, new_normals, faces_in_photo);
  //normal_constraint->initMatrix(face_list, vertex_list, vertex_shared_faces, normal_list, new_normals);
  normal_constraint->setLamdNormal(3.0f);
  normal_constraint->setLamdVMove(5.0f);

  solver->initCholesky();
  int max_iter = 20;
  int cur_iter = 0;
  do 
  {
    solver->runOneStep();
    ++cur_iter;

    std::cout << "The " << cur_iter << "th iteration finished" << std::endl;

  } while (cur_iter < max_iter);

  std::vector<float> new_vertex_list(solver->P_Opt.data(), solver->P_Opt.data() + solver->P_Opt.rows() * solver->P_Opt.cols());

  model->updateShape(new_vertex_list);
  // map new texture
  model->updateSHColor();

  std::cout << "Update geometry finished...\n";
}

void NormalTransfer::getDrawableActors(std::vector<GLActor>& actors)
{
   actors = this->actors;
}