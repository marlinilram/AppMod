#include "CurveGuidedVectorField.h"
#include "Model.h"
//#include "ARAP.h"
#include "GLActor.h"
#include "cholmod_matrix.h"
#include "tele_basicType.h"
#include "PolygonMesh.h"

using namespace LG;

typedef Cholmod_matrix<double>			CMatrix;

CurveGuidedVectorField::CurveGuidedVectorField()
{
  actors.push_back(GLActor(ML_POINT, 3.0f));
  actors.push_back(GLActor(ML_MESH, 1.0f));
  actors.push_back(GLActor(ML_LINE, 2.0f));
  actors[0].clearElement();
  actors[1].clearElement();
  actors[2].clearElement();
}

CurveGuidedVectorField::~CurveGuidedVectorField()
{

}

void CurveGuidedVectorField::computeVectorField(std::shared_ptr<Model> model)
{
  std::vector<STLVectori> allCurves = model->getShapeCrestLine();
  VertexList vertex = model->getShapeVertexList();
  vector_field.clear();
  vector_field.resize(vertex.size() / 3);

  // --------------------- mark constrained vertices ---------------------
  std::vector<int> constrained_vertices_mark;
  constrained_vertices_mark.resize(vertex.size() / 3);
  for(int i = 0; i < vertex.size() / 3; i ++)
  {
    constrained_vertices_mark[i] = 0;
  }
  for(int i = 0; i < 3; i ++)  // use only one line
  {
    for(int j = 0; j < allCurves[i].size() - 1; j ++)
    {
      constrained_vertices_mark[allCurves[i][j]] = 1;
      actors[0].addElement(vertex[3 * allCurves[i][j]], vertex[3 * allCurves[i][j] + 1], vertex[3 * allCurves[i][j] + 2], 1.0, 1.0, 1.0);
    }
  }

  // --------------------- compute Pb ---------------------
  std::vector<Vector3f> Pb;
  Pb.resize(vertex.size() / 3);
  for(int i = 0; i < vertex.size() / 3; i ++)
  {
    Pb[i] << 0, 0, 0;
  }
  for(int i = 0; i < 3; i ++)  // use only one line
  {
    for(int j = 0; j < allCurves[i].size(); j ++)
    {
      int pid1, pid2;
      //pid1 = j - 1 > 0 ? j - 1 : 0;
      //pid2 = j + 1 < allCurves[i].size() - 1 ? j + 1 : allCurves[i].size() - 1;
      float v1_x, v1_y, v1_z, v2_x, v2_y, v2_z, vertex_x, vertex_y, vertex_z, norm;
      if(j != allCurves[i].size() - 1)
      {
        pid1 = j;
        pid2 = j + 1;
        v1_x = vertex[3 * allCurves[i][pid1]];
        v1_y = vertex[3 * allCurves[i][pid1] + 1];
        v1_z = vertex[3 * allCurves[i][pid1] + 2];
        v2_x = vertex[3 * allCurves[i][pid2]];
        v2_y = vertex[3 * allCurves[i][pid2] + 1];
        v2_z = vertex[3 * allCurves[i][pid2] + 2];
        vertex_x = v2_x - v1_x;
        vertex_y = v2_y - v1_y;
        vertex_z = v2_z - v1_z;
        norm = sqrt(vertex_x * vertex_x + vertex_y * vertex_y + vertex_z * vertex_z);
        vertex_x /= norm;
        vertex_y /= norm;
        vertex_z /= norm;
        Pb[allCurves[i][j]] << vertex_x * 1.0e8, vertex_y * 1.0e8, vertex_z * 1.0e8;
        /*if(!(norm > 0 && norm < 1))
        {
          Pb[allCurves[i][j]] << 0, 0, 0;
        }
        else
        {
          Pb[allCurves[i][j]] << vertex_x * 1.0e8, vertex_y * 1.0e8, vertex_z * 1.0e8;
        }*/
      }
    }
  }

  // --------------------- L_add_P <- D - W + P ---------------------

  /*arap.reset(new ARAP);
  FaceList face = model->getShapeFaceList();
  AdjList adj = model->getShapeVertexAdjList();
  arap->initConstraint(vertex, face, adj);
  arap->setLamdARAP(1.0);
  SparseMatrix L_add_P;
  arap->getLinearSys(L_add_P);*/
  PolygonMesh* mesh = model->getPolygonMesh();
  PolygonMesh::Edge_attribute<Scalar> laplacian_cot = mesh->get_edge_attribute<Scalar>("e:laplacian_cot");
  PolygonMesh::Vertex_iterator vit = mesh->vertices_begin(), vend = mesh->vertices_end();
  double wi_sum = 0.0;
  cholmod_common c;
  cholmod_start(&c);
  CMatrix *SM = new CMatrix(vertex.size() / 3, true, &c);
  for(; vit != vend; ++ vit)
  {
    wi_sum = 0.0;
    int vi = (*vit).idx();
    PolygonMesh::Halfedge_around_vertex_circulator hec,hec_end;
    hec = hec_end = mesh->halfedges(*vit);
    do
    {
      int vj = mesh->to_vertex(*hec).idx();
      //double wij = laplacian_cot[mesh->edge(*hec)];
      double wij = 1;
      wi_sum += wij;
      SM->set_coef(vi, vj, -wij);
    }while(++hec != hec_end);
    if(constrained_vertices_mark[vi] == 1)
    {
      SM->set_coef(vi, vi, wi_sum + 1.0e8);
    }
    else
    {
      SM->set_coef(vi, vi, wi_sum);
    }
  }
  
  /*for(int i = 0; i < vertex.size() / 3; i ++)
  {
    if(constrained_vertices_mark[i] == 1)
    {
      L_add_P.coeffRef(i, i) += 1.0e8;
      L_add_P.coeffRef(i + vertex.size() / 3, i + vertex.size() / 3) += 1.0e8;
      L_add_P.coeffRef(i + 2 * vertex.size() / 3, i + 2 * vertex.size() / 3) += 1.0e8;
    }
  }*/

  // --------------------- solve the linear system with cholmod ---------------------
  cholmod_sparse *A;
  cholmod_dense *x, *y, *z, *b1;
  cholmod_factor *L;
  
  
  //for(int k = 0; k < L_add_P.outerSize(); k ++)
  //{
  //  for(SparseMatrix::InnerIterator it(L_add_P, k); it; it ++)
  //  {
  //    //std::cout << "row : " << it.row() << " , col : " << it.col() << " , value : " << it.value() << std::endl;
  //    SM->set_coef(it.row(), it.col(), it.value());
  //  }
  //}
  A = (cholmod_sparse*)SM->get_cholmod_sparse();
  //cholmod_print_sparse (A, "A", &c) ;
  b1 = cholmod_zeros(vertex.size() / 3, 1, CHOLMOD_REAL, &c);
  L = cholmod_analyze(A, &c);
  cholmod_factorize(A, L, &c);

  // --------------------- x demension -----------------------
  for(int i = 0; i < Pb.size(); i++)
  {
    ((float*)(b1->x))[i] = Pb[i].x();
  }
  x = cholmod_solve(CHOLMOD_A, L, b1, &c);

  // --------------------- y demension -----------------------
  for(int i = 0; i < Pb.size(); i++)
  {
    ((float*)(b1->x))[i] = Pb[i].y();
  }
  y = cholmod_solve(CHOLMOD_A, L, b1, &c);

  // --------------------- z demension -----------------------
  for(int i = 0; i < Pb.size(); i++)
  {
    ((float*)(b1->x))[i] = Pb[i].z();
  }
  z = cholmod_solve(CHOLMOD_A, L, b1, &c);

  /*for(int i = 0; i < Pb.size(); i ++)
  {
    ((float*)(b1->x))[i] = Pb[i].x();
    ((float*)(b1->x))[i + vertex.size() / 3] = Pb[i].y();
    ((float*)(b1->x))[i + 2 * vertex.size() / 3] = Pb[i].z();
  }*/
  
  /*X = cholmod_solve(CHOLMOD_A, L, b1, &c);*/

  // --------------------- write x,y,z into vector_field ---------------------
  for(int i = 0; i < vector_field.size(); i ++)
  {
    vector_field[i] << ((float*)(x->x))[i], ((float*)(y->x))[i], ((float*)(z->x))[i];
  }

  // --------------------- delete ---------------------
  cholmod_free_factor(&L, &c);
  cholmod_free_dense(&x, &c);
  cholmod_free_dense(&y, &c);
  cholmod_free_dense(&z, &c);
  cholmod_free_dense(&b1, &c);
  delete SM;
  cholmod_finish(&c);

  // --------------------- normalize vector field ---------------------
  for(int i = 0; i < vector_field.size(); i ++)
  {
    float norm = sqrt(vector_field[i].x() * vector_field[i].x() + vector_field[i].y() * vector_field[i].y() + vector_field[i].z() * vector_field[i].z());
    if(norm == 0)
    {
      vector_field[i] << vector_field[i].x() * 1.0e20, vector_field[i].y() * 1.0e20, vector_field[i].z()  * 1.0e20;
      norm = sqrt(vector_field[i].x() * vector_field[i].x() + vector_field[i].y() * vector_field[i].y() + vector_field[i].z() * vector_field[i].z());
    }
    if(norm > std::numeric_limits<float>::max())
    {
      vector_field[i] << vector_field[i].x() / 1.0e20, vector_field[i].y() / 1.0e20, vector_field[i].z() / 1.0e20;
      norm = sqrt(vector_field[i].x() * vector_field[i].x() + vector_field[i].y() * vector_field[i].y() + vector_field[i].z() * vector_field[i].z());
    }
    vector_field[i] << vector_field[i].x() / norm, vector_field[i].y() / norm, vector_field[i].z() / norm;
  }

  // --------------------- project the vector field onto the surface tangent planes ---------------------
  projectVectorField(model, 0.02, 0.03);

  // --------------------- put vector field into the GLActor in order to display ---------------------
  for(int i = 0; i < vertex.size() / 3; i ++)
  {
    Vector3f start;
    start << vertex[3 * i], vertex[3 * i + 1], vertex[3 * i + 2];
    Vector3f end;
    end = start + vector_field[i];
    actors[0].addElement(start.x(), start.y(), start.z(), 1.0, 0.0, 0.0);
    actors[2].addElement(start.x(), start.y(), start.z(), 0.0, 0.0, 0.0);
    actors[2].addElement(end.x(), end.y(), end.z(), 0.0, 0.0, 0.0);
  }
  std::cout << "Computing vector field is finished ! " << std::endl;
}

void CurveGuidedVectorField::projectVectorField(std::shared_ptr<Model> model, float low, float high)
{
  float mmax = std::numeric_limits<float>::min();
  float mmin = std::numeric_limits<float>::max();
  NormalList vertex_normal = model->getShapeNormalList();
  std::vector<float> mag;
  mag.resize(vector_field.size());
  Vector3f vn, pvec;
  for(int i = 0; i < vector_field.size(); i ++)
  {
    vn << vertex_normal[3 * i], vertex_normal[3 * i + 1], vertex_normal[3 * i + 2];
    pvec = vector_field[i].cross(vn);
    vector_field[i] = vn.cross(pvec);
    mag[i] = sqrt(vector_field[i].x() * vector_field[i].x() + vector_field[i].y() * vector_field[i].y() + vector_field[i].z() * vector_field[i].z());
    if(mag[i] > mmax)
    {
      mmax = mag[i];
    }
    if(mag[i] < mmin)
    {
      mmin = mag[i];
    }
  }
  float scale = (high - low) / (mmax - mmin);
  for(int i = 0; i < vector_field.size(); i ++)
  {
    vector_field[i] *= ((low + scale * (mag[i] - mmin)) / mag[i]);
  }
}

void CurveGuidedVectorField::getDrawableActors(std::vector<GLActor>& actors)
{
  actors = this->actors;
}
