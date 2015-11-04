#include "PlaneConstraint.h"
#include "Solver.h"
#include "WunderSVD3x3.h"

#include <fstream>

PlaneConstraint::PlaneConstraint()
{
  this->init();
  std::cout << "create a plane constraint.\n";
}

PlaneConstraint::~PlaneConstraint()
{

}

void PlaneConstraint::init()
{
  this->lamd_plane = 0.0;
  this->P_Num = 0;
  this->solver = nullptr;
}

void PlaneConstraint::initMatrix(STLVectori& _plane_vertex)
{
  // given a std::vector<int> which stores the vertex on
  // a feature line successively
  this->P_Num = this->solver->problem_size / 3;
  this->plane_vertex = _plane_vertex;

  TripletList line_triplets;
  this->right_hand = VectorXf::Zero(3 * this->P_Num);

  for (size_t i = 0; i < plane_vertex.size(); ++i)
  {
    int v_id = plane_vertex[i];
    line_triplets.push_back(Triplet(3 * v_id + 0, 3 * v_id + 0, 1.0));
    line_triplets.push_back(Triplet(3 * v_id + 1, 3 * v_id + 1, 1.0));
    line_triplets.push_back(Triplet(3 * v_id + 2, 3 * v_id + 2, 1.0));
  }

  this->constraint_matrix.resize(3 * this->P_Num, 3 * this->P_Num);
  this->constraint_matrix.setFromTriplets(line_triplets.begin(), line_triplets.end());
}

void PlaneConstraint::projection()
{

  Eigen::Matrix3Xf C(3, plane_vertex.size());
  for (size_t i = 0; i < plane_vertex.size(); ++i)
  {
    int v_id = plane_vertex[i];
    Vector3f v_pos;
    v_pos << this->solver->P_Opt[3 * v_id + 0], 
             this->solver->P_Opt[3 * v_id + 1],
             this->solver->P_Opt[3 * v_id + 2];
    C.col(i) = v_pos;
  }

  // Unclear: it is C.transpose() * C in original paper
  // C need to be mean centered
  Vector3f center = C * VectorXf::Ones(plane_vertex.size()) / plane_vertex.size();
  C = ((MatrixXf::Identity(plane_vertex.size(), plane_vertex.size()) - MatrixXf::Ones(plane_vertex.size(), plane_vertex.size()) / plane_vertex.size()) * C.transpose()).transpose();
  Matrix3f C_cov = C * C.transpose();
  Matrix3f U;
  Vector3f W;
  Matrix3f V;
  wunderSVD3x3(C_cov, U, W, V);

  Eigen::Matrix3Xf U_part(3, 2);
  U_part.col(0) = U.col(0);
  U_part.col(1) = U.col(1);

  //std::ofstream f_debug("PlaneVertices.mat");
  //if (f_debug)
  //{
  //  f_debug << C;
  //  f_debug.close();
  //}

  std::cout << "U*U':\n" << U_part * U_part.transpose() << "\n";

  Eigen::Matrix3Xf temp = U_part * U_part.transpose() * C;

  for (size_t i = 0; i < plane_vertex.size(); ++i)
  {
    int v_id = plane_vertex[i];
    this->right_hand[3 * v_id + 0] = temp(0, i) + center(0);
    this->right_hand[3 * v_id + 1] = temp(1, i) + center(1);
    this->right_hand[3 * v_id + 2] = temp(2, i) + center(2);
  }
}

void PlaneConstraint::update()
{

}

void PlaneConstraint::getRightHand(VectorXf& right_hand)
{
  right_hand = this->lamd_plane * this->right_hand;
}

void PlaneConstraint::getLinearSys(SparseMatrix& linear_sys)
{
  linear_sys = this->lamd_plane * this->constraint_matrix;
}

void PlaneConstraint::setSolver(std::shared_ptr<Solver> solver)
{
  this->solver = solver;
}