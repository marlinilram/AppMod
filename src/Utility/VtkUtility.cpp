#include "VtkUtility.h"

#include "PolygonMesh.h"

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCurvatures.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>

using namespace LG;

namespace VtkUtility
{
  void getCurvature(LG::PolygonMesh* mesh)
  {
    // initialize vtkPolyData from polygon mesh
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (auto i : mesh->vertices())
    {
      Vec3 pos = mesh->position(i);
      points->InsertNextPoint(pos[0], pos[1], pos[2]);
    }

    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
    for (auto f : mesh->faces())
    {
      vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
      std::vector<int> ids;
      for (auto vfc : mesh->vertices(f))
      {
        ids.push_back(vfc.idx());
      }
      triangle->GetPointIds()->SetId(0, ids[0]);
      triangle->GetPointIds()->SetId(1, ids[1]);
      triangle->GetPointIds()->SetId(2, ids[2]);
      triangles->InsertNextCell(triangle);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);

    // compute curvature
    vtkSmartPointer<vtkCurvatures> curvatures_filter = vtkSmartPointer<vtkCurvatures>::New();
    curvatures_filter->SetInputData(polyData);
    curvatures_filter->SetCurvatureTypeToMean();
    curvatures_filter->SetCurvatureTypeToGaussian();
    curvatures_filter->SetCurvatureTypeToMaximum();
    curvatures_filter->Update();

    vtkSmartPointer<vtkPolyData> new_poly_data = curvatures_filter->GetOutput();
    vtkSmartPointer<vtkDoubleArray> mean_array = vtkDoubleArray::SafeDownCast(new_poly_data->GetPointData()->GetArray("Mean_Curvature"));
    vtkSmartPointer<vtkDoubleArray> gaus_array = vtkDoubleArray::SafeDownCast(new_poly_data->GetPointData()->GetArray("Gauss_Curvature"));
    vtkSmartPointer<vtkDoubleArray> max_array = vtkDoubleArray::SafeDownCast(new_poly_data->GetPointData()->GetArray("Maximum_Curvature"));

    
    PolygonMesh::Vertex_attribute<Scalar> mean_curvature = mesh->vertex_attribute<Scalar>("v:mean_curvature");
    PolygonMesh::Vertex_attribute<Scalar> gaus_curvature = mesh->vertex_attribute<Scalar>("v:gaussian_curvature");
    Scalar min_mean_curv = std::numeric_limits<Scalar>::max();
    Scalar max_mean_curv = std::numeric_limits<Scalar>::min();
    Scalar min_gaus_curv = std::numeric_limits<Scalar>::max();
    Scalar max_gaus_curv = std::numeric_limits<Scalar>::min();
    std::vector<Scalar> mean_curv_cache;
    std::vector<Scalar> gaus_curv_cache;
    double h, k, k_max, k_min, tmp;

    for (auto vit : mesh->vertices())
    {
      k = gaus_array->GetTuple(vit.idx())[0];
      h = mean_array->GetTuple(vit.idx())[0];
      tmp = h * h - k;
      if (tmp >= 0)
      {
        k_min = h - sqrt(tmp);
        k_max = h + sqrt(tmp);
      }
      else
      {
        k_min = 0;
        k_max = 0;
      }

      // save original curvatures
      mean_curvature[vit] = k_min;
      gaus_curvature[vit] = k_max;

      // cache current max and min
      mean_curv_cache.push_back(k_min);
      gaus_curv_cache.push_back(k_max);
      //if (k_min > max_mean_curv) max_mean_curv = k_min;
      //if (k_min < min_mean_curv) min_mean_curv = k_min;
      //if (k_max > max_gaus_curv) max_gaus_curv = k_max;
      //if (k_max < min_gaus_curv) min_gaus_curv = k_max;
    }

    std::ofstream f_debug("non_normalized_mesh_curv.txt");
    if (f_debug)
    {
      for (auto vit : mesh->vertices())
      {
        f_debug << mean_curvature[vit] << "\t" << gaus_curvature[vit] << std::endl;
      }
      f_debug.close();
    }

    // discard upper and lower 5%
    unsigned int n = mean_curv_cache.size() - 1;
    unsigned int i = n / 20;
    std::sort(mean_curv_cache.begin(), mean_curv_cache.end());
    std::sort(gaus_curv_cache.begin(), gaus_curv_cache.end());
    min_mean_curv = mean_curv_cache[i];
    max_mean_curv = mean_curv_cache[n - 1 - i];
    min_gaus_curv = gaus_curv_cache[i];
    max_gaus_curv = gaus_curv_cache[n - 1 - i];


    for (auto vit : mesh->vertices())
    {
      mean_curvature[vit] = (mean_curvature[vit] - min_mean_curv) / (max_mean_curv - min_mean_curv);
      gaus_curvature[vit] = (gaus_curvature[vit] - min_gaus_curv) / (max_gaus_curv - min_gaus_curv);

      mean_curvature[vit] = mean_curvature[vit] < 0 ? 0 : (mean_curvature[vit] > 1 ? 1 : mean_curvature[vit]);
      gaus_curvature[vit] = gaus_curvature[vit] < 0 ? 0 : (gaus_curvature[vit] > 1 ? 1 : gaus_curvature[vit]);
    }

    f_debug.open("mesh_curv.txt");
    if (f_debug)
    {
      for (auto vit : mesh->vertices())
      {
        f_debug << mean_curvature[vit] << "\t" << gaus_curvature[vit] << std::endl;
      }
      f_debug.close();
    }

    std::cout << "max mean curvature: " << max_mean_curv << "\tmin mean curvature: " << min_mean_curv << std::endl;
    std::cout << "max gaussian curvature: " << max_gaus_curv << "\tmin gaussian curvature: " << min_gaus_curv << std::endl;


    //std::ofstream f_debug("curvature.txt");
    //if (f_debug)
    //{
    //  for (int i = 0; i < mean_array->GetNumberOfTuples(); ++i)
    //  {
    //    k = gaus_array->GetTuple(i)[0];
    //    h = mean_array->GetTuple(i)[0];
    //    tmp = h * h - k;
    //    if (tmp >= 0)
    //    {
    //      k_min = h - sqrt(tmp);
    //      k_max = h + sqrt(tmp);
    //    }
    //    else
    //    {
    //      k_min = 0;
    //      k_max = 0;
    //    }

    //    f_debug << h << "\t" << k << "\t" << k_max << "\t" << k_min << std::endl;
    //  }
    //  f_debug.close();
    //}

    //// test vis the mesh
    //vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    //mapper->SetInputData(polyData);

    //vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    //actor->SetMapper(mapper);

    //vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    //vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    //vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    //renderWindow->AddRenderer(renderer);
    //renderWindowInteractor->SetRenderWindow(renderWindow);

    //renderer->AddActor(actor);

    //renderWindow->Render();
    //renderWindowInteractor->Start();

  }


}