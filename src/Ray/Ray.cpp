#include "Ray.h"

// TODO: use BSP tree or consider using GPU
bool intersectTriangle(Eigen::Vector3f &p, Eigen::Vector3f &d,
                       Eigen::Vector3f &v0, Eigen::Vector3f &v1, Eigen::Vector3f &v2) {

                           //float e1[3], e2[3], h[3], s[3], q[3];
                           float a, f, u, v;

                           Eigen::Vector3f e1 = v1 - v0;
                           Eigen::Vector3f e2 = v2 - v0;

                           Eigen::Vector3f h = d.cross(e2);
                           a = e1.dot(h);

                           if (a > -0.00001 && a < 0.00001)
                               return(false);

                           f = 1 / a;
                           Eigen::Vector3f s = p - v0;
                           u = f * (s.dot(h));

                           if (u < 0.0 || u > 1.0)
                               return(false);

                           Eigen::Vector3f q = s.cross(e1);
                           v = f * d.dot(q);

                           if (v < 0.0 || u + v > 1.0)
                               return(false);

                           // at this stage we can compute t to find out where
                           // the intersection point is on the line
                           float t = f * e2.dot(q);

                           if (t > 0.00001) // ray intersection
                               return(true);

                           else // this means that there is a line intersection
                               // but not a ray intersection
                               return (false);

}

bool intersectModel(Eigen::Vector3f &p, Eigen::Vector3f &d, std::vector<float> &vertices, std::vector<unsigned int> &faces)
{
    for (decltype(faces.size()) i = 0; i < faces.size() / 3; ++i)
    {
        int i_v0 = faces[3 * i + 0];
        Eigen::Vector3f v0(vertices[3 * i_v0 + 0], vertices[3 * i_v0 + 1], vertices[3 * i_v0 + 2]);

        int i_v1 = faces[3 * i + 1];
        Eigen::Vector3f v1(vertices[3 * i_v1 + 0], vertices[3 * i_v1 + 1], vertices[3 * i_v1 + 2]);

        int i_v2 = faces[3 * i + 2];
        Eigen::Vector3f v2(vertices[3 * i_v2 + 0], vertices[3 * i_v2 + 1], vertices[3 * i_v2 + 2]);

        if (intersectTriangle(p, d, v0, v1, v2)) return true;
    }

    return false;
}


void Ray::passModel(std::vector<float> &vertices, std::vector<unsigned int> &faces)
{
    std::cout<<"Init Ray class...\n";


    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

    points->SetNumberOfPoints(vertices.size()/3);

    for (size_t i = 0; i < vertices.size()/3; ++i)
    {
        points->SetPoint(i, vertices[3*i+0], vertices[3*i+1], vertices[3*i+2]);

    }

    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();

    for (size_t i = 0; i < faces.size()/3; ++i)
    {
        triangle->GetPointIds()->SetId ( 0, faces[3*i+0] );
        triangle->GetPointIds()->SetId ( 1, faces[3*i+1] );
        triangle->GetPointIds()->SetId ( 2, faces[3*i+2] );
        triangles->InsertNextCell(triangle);
    }

    //triangles->InsertNextCell(triangle);

    poly_mesh = vtkSmartPointer<vtkPolyData>::New();
    
    poly_mesh->SetPoints(points);
    //std::cout<<"Check mesh right2\n";
    poly_mesh->SetPolys(triangles);
    //std::cout<<"Check mesh right3\n";

    bsptree = vtkSmartPointer< vtkModifiedBSPTree >::New();
    bsptree->SetDataSet(poly_mesh);
    bsptree->BuildLocator();

    //vtkSmartPointer< vtkVertexGlyphFilter > vertex_glyph = vtkSmartPointer< vtkVertexGlyphFilter >::New();
    //vertex_glyph->SetInputData(poly_mesh);
    //vertex_glyph->Update();

    std::cout<<"BSP tree updated...\n";

    //vtkSmartPointer<vtkPolyDataMapper> mapper =	vtkSmartPointer<vtkPolyDataMapper>::New();
    //std::cout<<"Check mesh right4\n";
    //mapper->SetInputData(poly_mesh);
    //std::cout<<"Check mesh right5\n";
    //vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    //std::cout<<"Check mesh right6\n";
    //actor->SetMapper(mapper);
    //std::cout<<"Check mesh right7\n";
    //vtkSmartPointer<vtkRenderer> renderer =	vtkSmartPointer<vtkRenderer>::New();
    //std::cout<<"Check mesh right8\n";
    //vtkSmartPointer<vtkRenderWindow> renderWindow =	vtkSmartPointer<vtkRenderWindow>::New();
    //std::cout<<"Check mesh right9\n";
    //renderWindow->AddRenderer(renderer);
    //std::cout<<"Check mesh right10\n";
    //vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =	vtkSmartPointer<vtkRenderWindowInteractor>::New();
    //std::cout<<"Check mesh right11\n";
    //renderWindowInteractor->SetRenderWindow(renderWindow);
    //std::cout<<"Check mesh right12\n";
    //renderer->AddActor(actor);
    //std::cout<<"Check mesh right13\n";
    //renderWindow->Render();
    //renderWindowInteractor->Start();
}

bool Ray::intersectModel(Eigen::Vector3d &ray_start, Eigen::Vector3d &ray_end)
{
    int id = bsptree->IntersectWithLine(ray_start.data(), ray_end.data(), 0.001, t, intersect_point, pt_coord, sub_id);

    if (id == 0) return true; // no intersection so return true
    else return false;

}