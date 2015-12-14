#include "ShapeUtility.h"
#include "BasicHeader.h"

#include "Model.h"
#include "Shape.h"
#include "PolygonMesh.h"
#include "Bound.h"

#include "Ray.h"
#include "SAMPLE.h"
#include "GenerateSamples.h"

using namespace LG;

namespace ShapeUtility
{
  void computeBaryCentreCoord(float pt[3], float v0[3], float v1[3], float v2[3], float lambd[3])
  {
    // v presents triangle vertex, pt locate inside triangle
    // pt[0]->x pt[1]->y pt[2]->z
    // v0[0]->x1 v0[1]->y1 v0[2]->z1
    // v1[0]->x2 v1[1]->y2 v1[2]->z2
    // v2[0]->x3 v2[1]->y3 v2[2]->z3
    Eigen::Vector3f e0(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]);
    Eigen::Vector3f e1(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]);
    Eigen::Vector3f e2(pt[0] - v0[0], pt[1] - v0[1], pt[2] - v0[2]);

    float d00 = e0.dot(e0);
    float d01 = e0.dot(e1);
    float d11 = e1.dot(e1);
    float d20 = e2.dot(e0);
    float d21 = e2.dot(e1);
    float denom = d00*d11 - d01*d01;

    lambd[1] = (d11*d20 - d01*d21) / denom;

    lambd[2] = (d00*d21 - d01*d20) / denom;

    lambd[0] = 1.0f - lambd[1] - lambd[2];

  }

  void computeNormalizedHeight(std::shared_ptr<Model> model)
  {
    Bound* boundbox = model->getBoundBox();
    PolygonMesh* mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<Scalar> normalized_height = mesh->vertex_attribute<Scalar>("v:NormalizedHeight");

    for (auto vit : mesh->vertices())
    {
      normalized_height[vit] = (mesh->position(vit)[2] - boundbox->minZ) / (boundbox->maxZ - boundbox->minZ);
    }
  }

  void computeDirectionalOcclusion(std::shared_ptr<Model> model)
  {
    // only called when the shape is changed

    // 1. initialize BSPTree
    int num_band = 2;
    std::cout << "Initialize BSPTree.\n";
    std::shared_ptr<Ray> ray(new Ray);
    ray->passModel(model->getShapeVertexList(), model->getShapeFaceList());

    // 2. generate direction samples
    std::cout << "Generate samples.\n";
    int sqrtNumSamples = 50;
    int numSamples = sqrtNumSamples * sqrtNumSamples;
    std::vector<SAMPLE> samples(numSamples);
    GenerateSamples(sqrtNumSamples, num_band, &samples[0]);

    // 3. compute coeffs
    std::cout << "Compute Directional Occlusion Feature.\n";
    int numBand = num_band;
    int numFunctions = numBand * numBand;
    Bound* bound = model->getBoundBox();
    PolygonMesh* poly_mesh = model->getPolygonMesh();
    PolygonMesh::Vertex_attribute<STLVectorf> shadowCoeff = poly_mesh->vertex_attribute<STLVectorf>("v:DirectionalOcclusion");
    PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
    float perc = 0;
    for (auto i : poly_mesh->vertices())
    {
      shadowCoeff[i].clear();
      shadowCoeff[i].resize(numFunctions, 0.0f);
      for (int k = 0; k < numSamples; ++k)
      {
        double dot = (double)samples[k].direction.dot(v_normals[i]);

        if (dot > 0.0)
        {
          Eigen::Vector3d ray_start = (poly_mesh->position(i) + 2 * 0.01 * bound->getRadius() * v_normals[i]).cast<double>();
          Eigen::Vector3d ray_end   = ray_start + (5 * bound->getRadius() * samples[k].direction).cast<double>();
          if (ray->intersectModel(ray_start, ray_end))
          {
            for (int l = 0; l < numFunctions; ++l)
            {
              shadowCoeff[i][l] += dot * samples[k].shValues[l];
            }
          }
        }
      }

      // rescale
      for (int l = 0; l < numFunctions; ++l)
      {
        shadowCoeff[i][l] *= 4.0 * M_PI / numSamples;
      }

      float cur_perc = (float)i.idx() / poly_mesh->n_vertices();
      if (cur_perc - perc >= 0.05)
      {
        perc = cur_perc;
        std::cout << perc << "...";
      }
    }
    std::cout << "Compute Directional Occlusion Feature finished.\n";
  }

}