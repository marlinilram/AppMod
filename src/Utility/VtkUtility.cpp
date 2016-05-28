#include "VtkUtility.h"

#include <set>

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



  STLVectori getConnectedComponent(PolygonMesh* mesh, PolygonMesh::Face face)
  {
    STLVectori f_set;
    PolygonMesh::Face_attribute<int> f_visited = mesh->get_face_attribute<int>("face:visited");
    if (f_visited[face] == 1)
    {
      // visited return empty
      return f_set;
    }

    f_set.push_back(face.idx());
    f_visited[face] = 1;

    for (auto hfc : mesh->halfedges(face))
    {
      if (!mesh->is_boundary(mesh->opposite_halfedge(hfc)))
      {
        PolygonMesh::Face n_face = mesh->face(mesh->opposite_halfedge(hfc));
        STLVectori return_f_set = getConnectedComponent(mesh, n_face);
        f_set.insert(f_set.end(), return_f_set.begin(), return_f_set.end());
      }
    }
  }

  void cutMesh(LG::PolygonMesh* mesh, std::vector<STLVectori>& components)
  {
    PolygonMesh::Halfedge_attribute<int> f_uv_id = mesh->halfedge_attribute<int>("he:uv_id");
    std::vector<Vec2>& f_uv_coord = mesh->get_attribute<std::vector<Vec2> >("he:texcoord");
    int n_faces = mesh->n_faces();
    VertexList v_list(3 * f_uv_coord.size(), 0);
    FaceList f_list(3 * n_faces, 0);
    STLVectorf uv_list(2 * f_uv_coord.size(), 0);
    STLVectori vertex_set;
    vertex_set.resize(f_uv_coord.size(), 0);
    std::set<int> cut_faces; // face id in original model
    STLVectori face_set; // face id mapping from new id to old id
    //PolygonMesh::Halfedge_attribute<Vec2> f_uv_coord = poly_mesh->halfedge_attribute<Vec2>("he:face_uv");
    for (int i = 0; i < n_faces; ++i)
    {
      int n_face_pt = 0;
      for (auto hefc : mesh->halfedges(PolygonMesh::Face(i)))
      {
        int cur_uv_id = f_uv_id[hefc];

        v_list[3 * cur_uv_id + 0] = f_uv_coord[cur_uv_id][0];
        v_list[3 * cur_uv_id + 1] = f_uv_coord[cur_uv_id][1];
        v_list[3 * cur_uv_id + 2] = 0.0;
        uv_list[2 * cur_uv_id + 0] = f_uv_coord[cur_uv_id][0];
        uv_list[2 * cur_uv_id + 1] = f_uv_coord[cur_uv_id][1];
        f_list[3 * i + n_face_pt] = cur_uv_id;
        vertex_set[cur_uv_id] = mesh->to_vertex(hefc).idx();

        ++n_face_pt;
      }
      cut_faces.insert(i);
      face_set.push_back(i);
    }
    
    // initilized UV mesh
    PolygonMesh uv_mesh;
    for (size_t i = 0; i < v_list.size() / 3; ++i)
    {
      uv_mesh.add_vertex(Vec3(v_list[3 * i + 0], v_list[3 * i + 1], v_list[3 * i + 2]));
    }
    std::vector<PolygonMesh::Vertex> vertices;
    for (size_t i = 0; i < f_list.size() / 3; ++i)
    {
      vertices.clear();
      vertices.push_back(PolygonMesh::Vertex(f_list[3 * i + 0]));
      vertices.push_back(PolygonMesh::Vertex(f_list[3 * i + 1]));
      vertices.push_back(PolygonMesh::Vertex(f_list[3 * i + 2]));
      uv_mesh.add_face(vertices);
    }


    PolygonMesh::Face_attribute<int> f_visited = uv_mesh.add_face_attribute<int>("face:visited", 0);
    for (auto fit : uv_mesh.faces())
    {
      if (f_visited[fit] == 0)
      {
        // get connect component
        STLVectori component = getConnectedComponent(&uv_mesh, fit);

        // export this component
        STLVectori real_component;
        for (auto i : component)
        {
          real_component.push_back(face_set[i]);
        }
        components.push_back(real_component);
      }
    }
  }

}