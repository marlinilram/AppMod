#include "ARAP.h"
#include "Solver.h"
#include "WunderSVD3x3.h"

ARAP::ARAP()
{
  this->init();
  std::cout << "create an ARAP constraint.\n";
}

ARAP::~ARAP()
{

}

void ARAP::init()
{
  this->lamd_ARAP = 0.0;
  this->P_Num = 0;
  this->solver = nullptr;
}

void ARAP::initConstraint(VertexList& vertex_list, FaceList& face_list, AdjList& adj_list)
{
  this->P_Num = vertex_list.size() / 3;
  P_vec.resize(3 * this->P_Num);
  R = std::vector<Matrix3f>(this->P_Num, Matrix3f::Identity());

  // sort adj_list
  this->adj_list = adj_list;
  for (size_t i = 0; i < this->adj_list.size(); ++i)
  {
    std::sort(this->adj_list[i].begin(), this->adj_list[i].end());
    P_vec[3 * i + 0] = vertex_list[3 * i + 0];
    P_vec[3 * i + 1] = vertex_list[3 * i + 1];
    P_vec[3 * i + 2] = vertex_list[3 * i + 2];
  }

  // sort face_list
  for (size_t i = 0; i < face_list.size() / 3; ++i)
  {
    std::vector<int> face(3, 0);
    face[0] = face_list[3 * i + 0];
    face[1] = face_list[3 * i + 1];
    face[2] = face_list[3 * i + 2];
    std::sort(face.begin(), face.end());
    triangle_list.push_back(Vector3i(face[0], face[1], face[2]));
  }

  buildLaplacianMatrix();
}

void ARAP::buildLaplacianMatrix()
{
  TripletList weight_list;
  TripletList weight_sum_list;
  //weight_list.reserve(3*7*P_Num); // each vertex may have about 7 vertices connected

  for (int i = 0; i < P_Num; ++i) 
  {
    float wi = 0.0f;
    for (size_t j = 0; j < adj_list[i].size(); ++j) 
    {
      int id_j = adj_list[i][j];

      STLVectori share_vertex;
      findShareVertex(i, id_j, share_vertex);

      float wij = 0.0f;
      if (share_vertex.size()==2) 
      {
        wij = computeWij(&P_vec.data()[3 * i], &P_vec.data()[3 * id_j], 
          &P_vec.data()[3 * share_vertex[0]], &P_vec.data()[3 * share_vertex[1]]);
      }
      else wij = computeWij(&P_vec.data()[3*i], &P_vec.data()[3*id_j], &P_vec.data()[3*share_vertex[0]]);

      weight_list.push_back(Triplet(3 * i + 0, 3 * id_j + 0, wij));
      weight_list.push_back(Triplet(3 * i + 1, 3 * id_j + 1, wij));
      weight_list.push_back(Triplet(3 * i + 2, 3 * id_j + 2, wij));

      wi += wij;
    }

    weight_sum_list.push_back(Triplet(3 * i + 0, 3 * i + 0, wi));
    weight_sum_list.push_back(Triplet(3 * i + 1, 3 * i + 1, wi));
    weight_sum_list.push_back(Triplet(3 * i + 2, 3 * i + 2, wi));
  }

  SparseMatrix Weight_sum_matrx;
  Weight_sum_matrx.resize(3*P_Num, 3*P_Num);
  Weight_sum_matrx.setFromTriplets(weight_sum_list.begin(), weight_sum_list.end());

  Weight_matrix.resize(3*P_Num, 3*P_Num);
  Weight_matrix.setFromTriplets(weight_list.begin(), weight_list.end());

  L_matrix = Weight_sum_matrx - Weight_matrix;
}

void ARAP::findShareVertex(int pi, int pj, STLVectori& share_vertex)
{
  STLVectori vertices;
  set_intersection(adj_list[pi].begin(), adj_list[pi].end(), adj_list[pj].begin(), adj_list[pj].end(), back_inserter(vertices));
  for (auto &i : vertices) 
  {
    STLVectori f;
    f.push_back(pi);
    f.push_back(pj);
    f.push_back(i);
    sort(f.begin(), f.end());
    std::vector<Vector3i>::iterator it = std::find(triangle_list.begin(), triangle_list.end(), Eigen::Map<Vector3i>(&f[0]));
    if (it != triangle_list.end())
    {
      if ((*it)(0) != pi && (*it)(0) != pj) share_vertex.push_back((*it)(0));
      else if ((*it)(1) != pi && (*it)(1) != pj) share_vertex.push_back((*it)(1));
      else share_vertex.push_back((*it)(2));
    }
  }
  if (share_vertex.size() > 2) 
  {
    std::cout << "share vertices number warning: " << share_vertex.size() << std::endl;
  }
}

float ARAP::computeWij(const float *p1, const float *p2, const float *p3, const float *p4)
{
  float e1 = sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
  float e2 = sqrt((p1[0]-p3[0])*(p1[0]-p3[0])+(p1[1]-p3[1])*(p1[1]-p3[1])+(p1[2]-p3[2])*(p1[2]-p3[2]));
  float e3 = sqrt((p3[0]-p2[0])*(p3[0]-p2[0])+(p3[1]-p2[1])*(p3[1]-p2[1])+(p3[2]-p2[2])*(p3[2]-p2[2]));
  float alpha_cos = fabs((e3*e3+e2*e2-e1*e1)/(2*e3*e2));
  float beta_cos = 0;
  if (p4 != nullptr) {
    float e4 = sqrt((p1[0]-p4[0])*(p1[0]-p4[0])+(p1[1]-p4[1])*(p1[1]-p4[1])+(p1[2]-p4[2])*(p1[2]-p4[2]));
    float e5 = sqrt((p4[0]-p2[0])*(p4[0]-p2[0])+(p4[1]-p2[1])*(p4[1]-p2[1])+(p4[2]-p2[2])*(p4[2]-p2[2]));
    beta_cos = fabs((e4*e4+e5*e5-e1*e1)/(2*e4*e5));
  }
  return ((alpha_cos/sqrt(1-alpha_cos*alpha_cos))+(beta_cos/sqrt(1-beta_cos*beta_cos)))/2;
}

void ARAP::projection()
{
  //update Ri
  this->updateRi();
}

void ARAP::updateRi()
{
  Matrix3f Si;
  MatrixXf Di;
  Matrix3Xf Pi_Prime;
  Matrix3Xf Pi;
  for (int i = 0; i != P_Num; ++i)
  {
    Di = MatrixXf::Zero(adj_list[i].size(), adj_list[i].size());
    Pi_Prime.resize(3, adj_list[i].size());
    Pi.resize(3, adj_list[i].size());
    // if there is not any single unconnected point this for loop can have a more efficient representation
    for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j)
    {
      Di(j, j) = Weight_matrix.coeffRef(3 * i, 3 * adj_list[i][j]);

      Pi.col(j) = Vector3f(P_vec[3 * i + 0] - P_vec[3 * adj_list[i][j] + 0],
                           P_vec[3 * i + 1] - P_vec[3 * adj_list[i][j] + 1],
                           P_vec[3 * i + 2] - P_vec[3 * adj_list[i][j] + 2]);
      
      Pi_Prime.col(j) = Vector3f(solver->P_Opt[3 * i + 0] - solver->P_Opt[3 * adj_list[i][j] + 0],
                                 solver->P_Opt[3 * i + 1] - solver->P_Opt[3 * adj_list[i][j] + 1],
                                 solver->P_Opt[3 * i + 2] - solver->P_Opt[3 * adj_list[i][j] + 2]);
    }
    Si = Pi * Di * Pi_Prime.transpose();
    Matrix3f Ui;
    Vector3f Wi;
    Matrix3f Vi;
    wunderSVD3x3(Si, Ui, Wi, Vi);
    R[i] = Vi * Ui.transpose();

    if (R[i].determinant() < 0)
    {
      std::cout << "determinant is negative!" << std::endl;
    }
  }
}

void ARAP::update()
{
  // update d vec
  this->updatedvec();
}

void ARAP::updatedvec()
{
  Eigen::Matrix3Xf d = Eigen::Matrix3Xf::Zero(3, P_Num);
  for (int i = 0; i != P_Num; ++i)
  {
    // if there is not any single unconnected point this for loop can have a more efficient representation
    for (decltype(adj_list[i].size()) j = 0; j != adj_list[i].size(); ++j)
    {
      Vector3f P_diff;
      P_diff << P_vec[3 * i + 0] - P_vec[3 * adj_list[i][j] + 0],
                P_vec[3 * i + 1] - P_vec[3 * adj_list[i][j] + 1],
                P_vec[3 * i + 2] - P_vec[3 * adj_list[i][j] + 2];

      // TODO: try to make the scale possible ?
      //Vector3f P_Opt_diff;
      //P_Opt_diff << solver->P_Opt[3 * i + 0] - solver->P_Opt[3 * adj_list[i][j] + 0],
      //              solver->P_Opt[3 * i + 1] - solver->P_Opt[3 * adj_list[i][j] + 1],
      //              solver->P_Opt[3 * i + 2] - solver->P_Opt[3 * adj_list[i][j] + 2];
      //float scale = P_Opt_diff.norm() / P_diff.norm();

      d.col(i) += (Weight_matrix.coeffRef(3 * i, 3 * adj_list[i][j]) / 2) * (R[i] + R[adj_list[i][j]]) * P_diff;
    }
  }

  d_vec = Eigen::Map<VectorXf>(d.data(), 3 * P_Num, 1);
}

void ARAP::getRightHand(VectorXf& right_hand)
{
  right_hand = this->lamd_ARAP * this->d_vec;
}

void ARAP::getLinearSys(SparseMatrix& linear_sys)
{
  linear_sys = this->lamd_ARAP * this->L_matrix;
}

void ARAP::setSolver(std::shared_ptr<Solver> solver)
{
  this->solver = solver;
}