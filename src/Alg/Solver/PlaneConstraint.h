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
* lie on a continuous plane. It is described by the cosine between connected
* edges.

=========================================================================*/

#ifndef PlaneConstraint_H
#define PlaneConstraint_H

#include "Constraint.h"
#include "BasicHeader.h"

class Solver;

class PlaneConstraint : public Constraint
{
public:
  PlaneConstraint();
  virtual ~PlaneConstraint();

  void initMatrix(STLVectori& _plane_vertex);
  virtual void init();
  virtual void update();
  virtual void projection();
  virtual void getRightHand(VectorXf& right_hand);
  virtual void getLinearSys(SparseMatrix& linear_sys);
  virtual void setSolver(std::shared_ptr<Solver> solver);

  inline void setLamdPlane(float lamd) { this->lamd_plane = lamd; };

private:
  float lamd_plane;
  int P_Num;

  SparseMatrix constraint_matrix;
  VectorXf right_hand;
  STLVectori plane_vertex;

  std::shared_ptr<Solver> solver;

private:
  PlaneConstraint(const PlaneConstraint&);
  void operator = (const PlaneConstraint&);
};

#endif // !PlaneConstraint_H
