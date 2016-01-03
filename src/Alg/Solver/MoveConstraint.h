/*=========================================================================

* @file
* @author  Lin Ma <majcjc@gmail.com>
* @version 1.0
*
* @section LICENSE
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details at
* http://www.gnu.org/copyleft/gpl.html
*
* @section DESCRIPTION
*
* Line constraint: This constraint indicates that a set of vertices should
* lie on a continuous curve. It is described by the cosine between connected
* edges.

=========================================================================*/

#ifndef MoveConstraint_H
#define MoveConstraint_H

#include "Constraint.h"
#include "BasicHeader.h"

class Solver;

class MoveConstraint : public Constraint
{
public:
  MoveConstraint();
  virtual ~MoveConstraint();

  void initMatrix(const STLVectori& vertex_id, const VertexList& vertex_list);
  virtual void init();
  virtual void update();
  virtual void projection();
  virtual void getRightHand(VectorXf& right_hand);
  virtual void getLinearSys(SparseMatrix& linear_sys);
  virtual void setSolver(std::shared_ptr<Solver> solver);

  inline void setLamdMove(float lamd) { this->lamd_move = lamd; };

private:
  float lamd_move;
  int P_Num;

  SparseMatrix constraint_matrix;
  VectorXf right_hand;

  std::shared_ptr<Solver> solver;

private:
  MoveConstraint(const MoveConstraint&);
  void operator = (const MoveConstraint&);
};

#endif // !MoveConstraint_H
