#include "Sphere.h"

#include "SAMPLE.h"
#include "GenerateSamples.h"

void Sphere::Init(Eigen::Vector3f _center, float _radius, int numSlices, int numStacks)
{
  poly_mesh.reset(new PolygonMesh());

  //First vertex at the top
  Vec3 sphereTop(0, 0, _radius);
  sphereTop += _center;
  poly_mesh->add_vertex(sphereTop);

  //Loop through stacks
  for(int stack=1; stack<numStacks; ++stack)
  {
    double theta=M_PI*(double(stack)/numStacks);

    for(int slice=0; slice<numSlices; ++slice)
    {
      double phi=2*M_PI*(double(slice)/numSlices);

      Vec3 currentVertex(	float(_radius*sin(theta)*cos(phi)),
        float(_radius*sin(theta)*sin(phi)),
        float(_radius*cos(theta)));

      //currentVertex.normal=currentVertex.position.GetNormalized();

      currentVertex += _center;

      //currentVertex.diffuseMaterial=diffuseMaterial;

      poly_mesh->add_vertex(currentVertex);
    }
  }

  //Finally add a vertex at the bottom
  Vec3 sphereBottom(0.0f, 0.0f, -_radius);
  sphereBottom += _center;
  //sphereBottom.normal.Set(0.0f, 0.0f,-1.0f);
  //sphereBottom.diffuseMaterial=diffuseMaterial;
  poly_mesh->add_vertex(sphereBottom);



  //Generate the sphere indices

  //The first stack is a triangle fan
  std::vector<PolygonMesh::Vertex> vertices;
  for(int slice=0; slice<numSlices; ++slice)
  {
    vertices.clear();
    vertices.push_back(PolygonMesh::Vertex(0));
    vertices.push_back(PolygonMesh::Vertex(1+slice));
    vertices.push_back(PolygonMesh::Vertex(1+(slice==numSlices-1 ? 0 : slice+1)));
    poly_mesh->add_face(vertices);
  }

  //Now do all but the last stack
  for(int stack=1; stack<numStacks-1; ++stack)
  {
    for(int slice=0; slice<numSlices; ++slice)
    {
      vertices.clear();
      vertices.push_back(PolygonMesh::Vertex(1+(stack-1)*numSlices+slice));
      vertices.push_back(PolygonMesh::Vertex(1+stack*numSlices+slice));
      vertices.push_back(PolygonMesh::Vertex(1+(stack-1)*numSlices+(slice==numSlices-1 ? 0 : slice+1)));
      poly_mesh->add_face(vertices);

      vertices.clear();
      vertices.push_back(PolygonMesh::Vertex(1+stack*numSlices+slice));
      vertices.push_back(PolygonMesh::Vertex(1+stack*numSlices+(slice==numSlices-1 ? 0 : slice+1)));
      vertices.push_back(PolygonMesh::Vertex(1+(stack-1)*numSlices+(slice==numSlices-1 ? 0 : slice+1)));
      poly_mesh->add_face(vertices);
    }
  }

  //The final stack is a triangle fan
  for(int slice=0; slice<numSlices; ++slice)
  {
    vertices.clear();
    vertices.push_back(PolygonMesh::Vertex(1+(numStacks-2)*numSlices+slice));
    vertices.push_back(PolygonMesh::Vertex(1+(numStacks-1)*numSlices));
    vertices.push_back(PolygonMesh::Vertex(1+(numStacks-2)*numSlices+(slice==numSlices-1 ? 0 : slice+1)));
    poly_mesh->add_face(vertices);
  }

  // now the normal
  PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
  for (auto vit : poly_mesh->vertices())
  {
    v_normals[vit] = (poly_mesh->position(vit) - _center).normalized();
  }

  computeSHCoeffs();

  model_view = Eigen::Matrix4f::Identity();
}

void Sphere::computeSHCoeffs()
{
  int sqrtNumSamples = 50;
  int numSamples = sqrtNumSamples * sqrtNumSamples;
  std::vector<SAMPLE> samples(numSamples);
  GenerateSamples(sqrtNumSamples, 3, &samples[0]);

  // 3. compute coeffs
  int numBand = 3;
  int numFunctions = numBand * numBand;
  PolygonMesh::Vertex_attribute<std::vector<float> > shadowCoeff = poly_mesh->vertex_attribute<std::vector<float> >("v:SHShadowCoeffs");
  PolygonMesh::Vertex_attribute<Vec3> v_normals = poly_mesh->vertex_attribute<Vec3>("v:normal");
  float perc = 0;
  for (auto i : poly_mesh->vertices())
  {
    shadowCoeff[i].resize(numFunctions, 0.0f);
    for (int k = 0; k < numSamples; ++k)
    {
      double dot = (double)samples[k].direction.dot(v_normals[i]);

      if (dot > 0.0)
      {
        for (int l = 0; l < numFunctions; ++l)
        {
          shadowCoeff[i][l] += dot * samples[k].shValues[l];
        }
      }
    }

    // rescale
    for (int l = 0; l < numFunctions; ++l)
    {
      shadowCoeff[i][l] *= 4.0 * M_PI / numSamples;
    }
  }
}