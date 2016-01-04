#include "MoveConstraint.h"
#include "Solver.h"

#include <fstream>

MoveConstraint::MoveConstraint()
{
  this->init();
  std::cout << "create a move constraint.\n";
}

MoveConstraint::~MoveConstraint()
{

}

void MoveConstraint::init()
{
  this->lamd_move = 0.0;
  this->P_Num = 0;
  this->solver = nullptr;
}

void MoveConstraint::initMatrix(const STLVectori& vertex_id, const VertexList& vertex_list)
{
  this->P_Num = this->solver->problem_size / 3;

  // set matrix
  TripletList line_triplets;
  for (size_t i = 0; i < vertex_id.size(); ++i)
  {
    int v_id = vertex_id[i];
    line_triplets.push_back(Triplet(3 * v_id + 0, 3 * v_id + 0, 1.0));
    line_triplets.push_back(Triplet(3 * v_id + 1, 3 * v_id + 1, 1.0));
    line_triplets.push_back(Triplet(3 * v_id + 2, 3 * v_id + 2, 1.0));
  }
  this->constraint_matrix.resize(3 * this->P_Num, 3 * this->P_Num);
  this->constraint_matrix.setFromTriplets(line_triplets.begin(), line_triplets.end());

  // set right hand
  this->right_hand = VectorXf::Zero(3 * this->P_Num);
  for (size_t i = 0; i < vertex_id.size(); ++i)
  {
    int v_id = vertex_id[i];
    this->right_hand(3 * v_id + 0) = vertex_list[3 * i + 0];
    this->right_hand(3 * v_id + 1) = vertex_list[3 * i + 1];
    this->right_hand(3 * v_id + 2) = vertex_list[3 * i + 2];
  }

  //std::ofstream fdebug("constraint.mat");
  //if (fdebug)
  //{
  //  fdebug << constraint_matrix;
  //  fdebug.close();
  //}
  //fdebug.open("right_hand.mat");
  //if (fdebug)
  //{
  //  fdebug << right_hand;
  //  fdebug.close();
  //}

  if (vertex_id.size() * 3 != vertex_list.size())
  {
    std::cout << "Error, vertex size doesn't match the vertex list.\n";
  }
  else
  {
    std::ofstream fdebug("move_debug.txt");
    if (fdebug)
    {
      for (size_t i = 0; i < vertex_id.size(); ++i)
      {
        fdebug << vertex_id[i] << "\t" << vertex_list[3 * i + 0] << "\t" << vertex_list[3 * i + 1] << "\t" << vertex_list[3 * i + 2] << std::endl;
      }
      fdebug.close();
    }
  }
}

void MoveConstraint::update()
{
  // no need to update
}

void MoveConstraint::projection()
{
  // no need to do projection
}

void MoveConstraint::getRightHand(VectorXf& right_hand)
{
  right_hand = this->lamd_move * this->right_hand;
}

void MoveConstraint::getLinearSys(SparseMatrix& linear_sys)
{
  linear_sys = this->lamd_move * this->constraint_matrix;
}

void MoveConstraint::setSolver(std::shared_ptr<Solver> solver)
{
  this->solver = solver;
}