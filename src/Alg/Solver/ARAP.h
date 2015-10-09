#ifndef ARAP_H
#define ARAP_H

#include "Constraint.h"
#include "BasicHeader.h"

class Solver;

class ARAP : public Constraint
{
public:
  ARAP();
  virtual ~ARAP();

  virtual void init();
  virtual void update();
  virtual void projection();
  virtual void getRightHand(VectorXf& right_hand);
  virtual void getLinearSys(SparseMatrix& linear_sys);
  virtual void setSolver(std::shared_ptr<Solver> solver);

  void initConstraint(VertexList& vertex_list, FaceList& face_list, AdjList& adj_list);
  inline void setLamdARAP(float lamd) { this->lamd_ARAP = lamd; };

private:
  // pre-build laplacian matrix
  void buildLaplacianMatrix();
  // find share vertex of an edge
  void findShareVertex(int i, int j, STLVectori& share_vertex);
  // pre-compute weights
  float computeWij(const float *p1, const float *p2, const float *p3, const float *p4 = nullptr);
  // update R
  void updateRi();
  // update d vec
  void updatedvec();

private:
  float lamd_ARAP;
  int P_Num;

  SparseMatrix L_matrix;
  SparseMatrix Weight_matrix;
  VectorXf d_vec;
  VectorXf P_vec;
  AdjList adj_list; // sorted
  std::vector<Vector3i> triangle_list; // sorted
  std::vector<Matrix3f> R;


  std::shared_ptr<Solver> solver;

private:
  ARAP(const ARAP&);
  void operator = (const ARAP&);
};

#endif // !ARAP_H
