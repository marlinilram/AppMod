#include "LineConstraint.h"
#include "Solver.h"
#include "WunderSVD3x3.h"

LineConstraint::LineConstraint()
{
  this->init();
  std::cout << "create a feature line constraint.\n";
}

LineConstraint::~LineConstraint()
{

}

void LineConstraint::init()
{
  this->lamd_feature_line = 0.0;
  this->P_Num = 0;
  this->solver = nullptr;
}

void LineConstraint::initMatrix(STLVectori& feature_line, VertexList& vertex_list)
{
  // given a std::vector<int> which stores the vertex on
  // a feature line successively
  this->P_Num = this->solver->problem_size / 3;
  this->feature_line = feature_line;

  TripletList line_triplets;
  this->right_hand = VectorXf::Zero(3 * this->P_Num);

  for (size_t i = 0; i < feature_line.size(); ++i)
  {
    int v_id = feature_line[i];
    line_triplets.push_back(Triplet(3 * v_id + 0, 3 * v_id + 0, 1.0));
    line_triplets.push_back(Triplet(3 * v_id + 1, 3 * v_id + 1, 1.0));
    line_triplets.push_back(Triplet(3 * v_id + 2, 3 * v_id + 2, 1.0));
  }

  this->constraint_matrix.resize(3 * this->P_Num, 3 * this->P_Num);
  this->constraint_matrix.setFromTriplets(line_triplets.begin(), line_triplets.end());
}

void LineConstraint::projection()
{

  Eigen::Matrix3Xf C(3, feature_line.size());
  for (size_t i = 0; i < feature_line.size(); ++i)
  {
    int v_id = feature_line[i];
    Vector3f v_pos;
    v_pos << this->solver->P_Opt[3 * v_id + 0], 
             this->solver->P_Opt[3 * v_id + 0],
             this->solver->P_Opt[3 * v_id + 0];
    C.col(i) = v_pos;
  }

  // Unclear: it is C.transpose() * C in original paper
  Matrix3f C_cov = C * C.transpose();
  Matrix3f U;
  Vector3f W;
  Matrix3f V;
  wunderSVD3x3(C_cov, U, W, V);

  Eigen::Matrix3Xf temp = U.col(0) * U.col(0).transpose() * C;

  for (size_t i = 0; i < feature_line.size(); ++i)
  {
    int v_id = feature_line[i];
    this->right_hand[3 * v_id + 0] = temp(0, i);
    this->right_hand[3 * v_id + 1] = temp(1, i);
    this->right_hand[3 * v_id + 2] = temp(2, i);
  }
}

void LineConstraint::update()
{

}

void LineConstraint::getRightHand(VectorXf& right_hand)
{
  right_hand = this->lamd_feature_line * this->right_hand;
}

void LineConstraint::getLinearSys(SparseMatrix& linear_sys)
{
  linear_sys = this->lamd_feature_line * this->constraint_matrix;
}

void LineConstraint::setSolver(std::shared_ptr<Solver> solver)
{
  this->solver = solver;
}