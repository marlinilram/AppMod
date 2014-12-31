#include "GeometryPartAlg.h"

GeometryPartAlg::GeometryPartAlg()
{

}

GeometryPartAlg::~GeometryPartAlg()
{

}

void GeometryPartAlg::updateGeometry(Coarse *model)
{
    std::cout << "\nBegin update geometry...\n";

    typedef std::pair<int, int> Edge;

    std::vector<unsigned int> *face_list = model->getFaceList();
    std::vector<float> *new_normals = model->getModelNewNormal();
    std::vector<float> *vertex_list = model->getVertexList();
    std::vector<float> *normal_list = model->getNormalList();

    // build edge graph
    std::cout << "Building edge graph...\n";

    std::vector<Edge> edges;
    edges.clear();

    {
        for (decltype((*face_list).size()) i = 0; i < (*face_list).size() / 3; ++i)
        {
            int ptid[3] = { (*face_list)[3 * i + 0], (*face_list)[3 * i + 1], (*face_list)[3 * i + 2] };
            // the order of start point and end point doesn't matter
            edges.push_back(ptid[0] < ptid[1] ? Edge(ptid[0], ptid[1]) : Edge(ptid[1], ptid[0]));
            edges.push_back(ptid[1] < ptid[2] ? Edge(ptid[1], ptid[2]) : Edge(ptid[2], ptid[1]));
            edges.push_back(ptid[2] < ptid[0] ? Edge(ptid[2], ptid[0]) : Edge(ptid[0], ptid[2]));
        }
        std::sort(edges.begin(), edges.end());
        std::vector<Edge>::iterator iter = std::unique(edges.begin(), edges.end());
        edges.erase(iter, edges.end());
    }

    std::cout << "Edge number: " << edges.size() << "\n";


    // add bend edge to edge graph
    std::cout << "Building bending edge graph...\n";

    std::vector<Edge> bending_edges;
    bending_edges.clear();

    std::vector<std::vector<int>> *vertices_share_faces = model->getVertexShareFaces();
    for (auto &i : edges) 
    {
        int cross_pi = -1;
        int cross_pj = -1;
        if (findShareVertex(i.first, i.second, model, cross_pi, cross_pj))
        {
            bending_edges.push_back(cross_pi < cross_pj ? Edge(cross_pi, cross_pj) : Edge(cross_pj, cross_pi));
        }
        else std::cout << "Error: can not find a cross edge...\n";
    }

    std::cout << "Bending edge number: " << bending_edges.size() << "\n";

    // build L matrix

    std::cout << "Building L matrix...\n";

    float lambd_k_strech = 10.0f;
    float lambd_k_bend = 10.0f;
    float k = lambd_k_strech;
    int P_Num = (*vertex_list).size() / 3;

    std::cout << "Vertex number: " << P_Num << "\n";

    std::vector<Eigen::Triplet<float>> L_triplets;
    L_triplets.clear();

    for (auto &i : edges) 
    {
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 0, 3 * i.first + 0, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 1, 3 * i.first + 1, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 2, 3 * i.first + 2, k));

        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 0, 3 * i.second + 0, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 1, 3 * i.second + 1, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 2, 3 * i.second + 2, -k));

        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 0, 3 * i.first + 0, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 1, 3 * i.first + 1, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 2, 3 * i.first + 2, -k));

        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 0, 3 * i.second + 0, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 1, 3 * i.second + 1, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 2, 3 * i.second + 2, k));
    }

    k = lambd_k_bend;

    for (auto &i : bending_edges)
    {
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 0, 3 * i.first + 0, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 1, 3 * i.first + 1, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 2, 3 * i.first + 2, k));

        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 0, 3 * i.second + 0, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 1, 3 * i.second + 1, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.first + 2, 3 * i.second + 2, -k));

        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 0, 3 * i.first + 0, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 1, 3 * i.first + 1, -k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 2, 3 * i.first + 2, -k));

        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 0, 3 * i.second + 0, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 1, 3 * i.second + 1, k));
        L_triplets.push_back(Eigen::Triplet<float>(3 * i.second + 2, 3 * i.second + 2, k));
    }

    Eigen::SparseMatrix<float> L_matrix(3 * P_Num, 3 * P_Num);
    L_matrix.setFromTriplets(L_triplets.begin(), L_triplets.end());

    // build normal constrains and vertical move constrains

    std::cout << "Building normal constrains and vertical movement constrains...\n";

    float lambd_normal = 25.0f;
    float lambd_vertical_move = 10.0f;

    std::vector<Eigen::Triplet<float>> normal_triplets;
    normal_triplets.clear();
    std::vector<Eigen::Triplet<float>> vertical_move_triplets;
    vertical_move_triplets.clear();
    Eigen::VectorXf vertical_move = Eigen::VectorXf::Zero(3 * P_Num);

    for (int i = 0; i < P_Num; ++i)
    {
        std::vector<int> *point_share_faces = model->getVertexShareFaces(i);
        
        // for each face that share vertex i, we build its normal constrains
        for (decltype((*point_share_faces).size()) j = 0; j < (*point_share_faces).size(); ++j)
        {
            float n[3] = { (*new_normals)[3 * (*point_share_faces)[j] + 0],
                           (*new_normals)[3 * (*point_share_faces)[j] + 1],
                           (*new_normals)[3 * (*point_share_faces)[j] + 2] };

            int points_in_face[3] = { (*face_list)[3 * (*point_share_faces)[j] + 0],
                                      (*face_list)[3 * (*point_share_faces)[j] + 1],
                                      (*face_list)[3 * (*point_share_faces)[j] + 2] };

            int connect[2];
            getConnectedPtID(i, points_in_face, connect);

            // 2 edges first dimension
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 0, lambd_normal*n[0] * n[0] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 1, lambd_normal*n[0] * n[1] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 2, lambd_normal*n[0] * n[2] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * connect[0] + 0, -lambd_normal*n[0] * n[0]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * connect[0] + 1, -lambd_normal*n[0] * n[1]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * connect[0] + 2, -lambd_normal*n[0] * n[2]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * connect[1] + 0, -lambd_normal*n[0] * n[0]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * connect[1] + 1, -lambd_normal*n[0] * n[1]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * connect[1] + 2, -lambd_normal*n[0] * n[2]));
            // 2 edges second dimension
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 0, lambd_normal*n[1] * n[0] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 1, lambd_normal*n[1] * n[1] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 2, lambd_normal*n[1] * n[2] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * connect[0] + 0, -lambd_normal*n[1] * n[0]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * connect[0] + 1, -lambd_normal*n[1] * n[1]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * connect[0] + 2, -lambd_normal*n[1] * n[2]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * connect[1] + 0, -lambd_normal*n[1] * n[0]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * connect[1] + 1, -lambd_normal*n[1] * n[1]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * connect[1] + 2, -lambd_normal*n[1] * n[2]));
            // 2 edges third dimension
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 0, lambd_normal*n[2] * n[0] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 1, lambd_normal*n[2] * n[1] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 2, lambd_normal*n[2] * n[2] * 2));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * connect[0] + 0, -lambd_normal*n[2] * n[0]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * connect[0] + 1, -lambd_normal*n[2] * n[1]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * connect[0] + 2, -lambd_normal*n[2] * n[2]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * connect[1] + 0, -lambd_normal*n[2] * n[0]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * connect[1] + 1, -lambd_normal*n[2] * n[1]));
            normal_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * connect[1] + 2, -lambd_normal*n[2] * n[2]));
        }

        // vertical move towards original normal direction
        float n[3] = { (*normal_list)[3 * i + 0],
                       (*normal_list)[3 * i + 1],
                       (*normal_list)[3 * i + 2] };

        float pt[3] = { (*vertex_list)[3 * i + 0],
                        (*vertex_list)[3 * i + 1],
                        (*vertex_list)[3 * i + 2] };

        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 0, lambd_vertical_move*(n[1] * n[1] + n[2] * n[2])));
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 1, lambd_vertical_move*(-n[0] * n[1])));
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 0, 3 * i + 2, lambd_vertical_move*(-n[0] * n[2])));
        vertical_move(3 * i + 0) = lambd_vertical_move*((n[1] * n[1] + n[2] * n[2]) * pt[0] - n[0] * n[1] * pt[1] - n[0] * n[2] * pt[2]);
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 0, lambd_vertical_move*(-n[1] * n[0])));
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 1, lambd_vertical_move*(n[0] * n[0] + n[2] * n[2])));
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 1, 3 * i + 2, lambd_vertical_move*(-n[1] * n[2])));
        vertical_move(3 * i + 1) = lambd_vertical_move*(-n[1] * n[0] * pt[0] + (n[0] * n[0] + n[2] * n[2]) * pt[1] - n[1] * n[2] * pt[2]);
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 0, lambd_vertical_move*(-n[2] * n[0])));
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 1, lambd_vertical_move*(-n[2] * n[1])));
        vertical_move_triplets.push_back(Eigen::Triplet<float>(3 * i + 2, 3 * i + 2, lambd_vertical_move*(n[0] * n[0] + n[1] * n[1])));
        vertical_move(3 * i + 2) = lambd_vertical_move*(-n[2] * n[0] * pt[0] - n[2] * n[1] * pt[1] + (n[0] * n[0] + n[1] * n[1]) * pt[2]);
    }
    Eigen::SparseMatrix<float> normal_matrix(3 * P_Num, 3 * P_Num);
    normal_matrix.setFromTriplets(normal_triplets.begin(), normal_triplets.end());
    Eigen::SparseMatrix<float> vertical_move_matrix(3 * P_Num, 3 * P_Num);
    vertical_move_matrix.setFromTriplets(vertical_move_triplets.begin(), vertical_move_triplets.end());

    // build linear system
    std::cout << "Building linear system...\n";

    Eigen::SparseMatrix<float> linear_sys_matrix = L_matrix + normal_matrix + vertical_move_matrix;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<float>> chol;
    chol.analyzePattern(linear_sys_matrix);
    chol.factorize(linear_sys_matrix);

    // build J matrix
    std::cout << "Building J matrix...\n";

    std::vector<Eigen::Triplet<float>> J_triplets;
    J_triplets.clear();
    k = lambd_k_strech;
    for (decltype(edges.size()) i = 0; i != edges.size(); ++i) {
        J_triplets.push_back(Eigen::Triplet<float>(3 * edges[i].first + 0, 3 * i + 0, k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * edges[i].first + 1, 3 * i + 1, k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * edges[i].first + 2, 3 * i + 2, k));

        J_triplets.push_back(Eigen::Triplet<float>(3 * edges[i].second + 0, 3 * i + 0, -k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * edges[i].second + 1, 3 * i + 1, -k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * edges[i].second + 2, 3 * i + 2, -k));
    }
    k = lambd_k_bend;
    for (decltype(bending_edges.size()) i = 0; i != bending_edges.size(); ++i) {
        J_triplets.push_back(Eigen::Triplet<float>(3 * bending_edges[i].first + 0, 3 * (i + edges.size()) + 0, k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * bending_edges[i].first + 1, 3 * (i + edges.size()) + 1, k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * bending_edges[i].first + 2, 3 * (i + edges.size()) + 2, k));

        J_triplets.push_back(Eigen::Triplet<float>(3 * bending_edges[i].second + 0, 3 * (i + edges.size()) + 0, -k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * bending_edges[i].second + 1, 3 * (i + edges.size()) + 1, -k));
        J_triplets.push_back(Eigen::Triplet<float>(3 * bending_edges[i].second + 2, 3 * (i + edges.size()) + 2, -k));
    }
    Eigen::SparseMatrix<float> J_matrix(3 * P_Num, 3 * (edges.size() + bending_edges.size()));
    J_matrix.setFromTriplets(J_triplets.begin(), J_triplets.end());

    // build d
    //edges.insert(edges.end(), bendingEdges.begin(), bendingEdges.end());
    std::cout << "Building d vector...\n";

    Eigen::VectorXf d = Eigen::VectorXf::Zero(3 * (edges.size() + bending_edges.size()));
    for (decltype(edges.size()) i = 0; i != edges.size(); ++i) {
        float pt1[3] = { (*vertex_list)[3 * edges[i].first + 0],
                         (*vertex_list)[3 * edges[i].first + 1],
                         (*vertex_list)[3 * edges[i].first + 2] };

        float pt2[3] = { (*vertex_list)[3 * edges[i].second + 0],
                         (*vertex_list)[3 * edges[i].second + 1],
                         (*vertex_list)[3 * edges[i].second + 2] };

        d(3 * i + 0) = pt1[0] - pt2[0];
        d(3 * i + 1) = pt1[1] - pt2[1];
        d(3 * i + 2) = pt1[2] - pt2[2];
    }
    for (decltype(bending_edges.size()) i = 0; i != bending_edges.size(); ++i) {
        float pt1[3] = { (*vertex_list)[3 * bending_edges[i].first + 0],
                         (*vertex_list)[3 * bending_edges[i].first + 1],
                         (*vertex_list)[3 * bending_edges[i].first + 2] };
        
        float pt2[3] = { (*vertex_list)[3 * bending_edges[i].second + 0],
                         (*vertex_list)[3 * bending_edges[i].second + 1],
                         (*vertex_list)[3 * bending_edges[i].second + 2] };

        d(3 * (i + edges.size()) + 0) = pt1[0] - pt2[0];
        d(3 * (i + edges.size()) + 1) = pt1[1] - pt2[1];
        d(3 * (i + edges.size()) + 2) = pt1[2] - pt2[2];
    }

    // alternative solve vertex and d iteratively
    std::cout << "Iterative alternative optimization...\n";

    int max_iter = 20;
    int cur_iter = 0;
    Eigen::VectorXf P_Opt;

    do {
        // solve sparse matrix
        //chol.factorize(L_matrix);
        Eigen::VectorXf right_hand = J_matrix*d;
        P_Opt = chol.solve(right_hand + vertical_move);

        // update d
        for (decltype(edges.size()) i = 0; i != edges.size(); ++i) {
            double r = std::sqrt(d(3 * i + 0)*d(3 * i + 0) + d(3 * i + 1)*d(3 * i + 1) + d(3 * i + 2)*d(3 * i + 2));
            //if (i == 0) cout << r << endl;
            Eigen::Vector3f p12(P_Opt(3 * edges[i].first + 0) - P_Opt(3 * edges[i].second + 0),
                P_Opt(3 * edges[i].first + 1) - P_Opt(3 * edges[i].second + 1),
                P_Opt(3 * edges[i].first + 2) - P_Opt(3 * edges[i].second + 2));
            p12.normalize();
            d(3 * i + 0) = r*p12(0);
            d(3 * i + 1) = r*p12(1);
            d(3 * i + 2) = r*p12(2);
        }

        for (decltype(bending_edges.size()) i = 0; i != bending_edges.size(); ++i) {
            double r = std::sqrt(d(3 * (i + edges.size()) + 0)*d(3 * (i + edges.size()) + 0) + d(3 * (i + edges.size()) + 1)*d(3 * (i + edges.size()) + 1) + d(3 * (i + edges.size()) + 2)*d(3 * (i + edges.size()) + 2));
            //if (i == 0) cout << r << endl;
            Eigen::Vector3f p12(P_Opt(3 * bending_edges[i].first + 0) - P_Opt(3 * bending_edges[i].second + 0),
                P_Opt(3 * bending_edges[i].first + 1) - P_Opt(3 * bending_edges[i].second + 1),
                P_Opt(3 * bending_edges[i].first + 2) - P_Opt(3 * bending_edges[i].second + 2));
            p12.normalize();
            d(3 * (i + edges.size()) + 0) = r*p12(0);
            d(3 * (i + edges.size()) + 1) = r*p12(1);
            d(3 * (i + edges.size()) + 2) = r*p12(2);
        }

        ++cur_iter;

        std::cout << "The " << cur_iter << "th iteration finished" << std::endl;

    } while (cur_iter < max_iter);

    for (decltype((*vertex_list).size()) i = 0; i < (*vertex_list).size(); ++i)
    {
        (*vertex_list)[i] = P_Opt(i);
    }

	model->computeFaceNormal();

    std::ofstream f_P_Opt(model->getDataPath() + "/P_Opt.mat");
    if (f_P_Opt)
    {
        f_P_Opt << P_Opt;
        f_P_Opt.close();
    }

    std::cout << "Update geometry finished...\n";
}

bool GeometryPartAlg::findShareVertex(int pi, int pj, Coarse *model, int &cross_pi, int &cross_pj)
{
    //std::vector<int> vertices;
    //std::vector<Eigen::Vector3i> *face_list = model->getFaceListCompact();
    //std::vector<std::vector<int>> *adj_list = model->getVertexAdj();
    //std::set_intersection((*adj_list)[pi].begin(), (*adj_list)[pi].end(), (*adj_list)[pj].begin(), (*adj_list)[pj].end(), std::back_inserter(vertices));

    //for (auto &i : vertices)
    //{
    //    std::vector<int> f;
    //    f.push_back(pi);
    //    f.push_back(pj);
    //    f.push_back(i);
    //    std::sort(f.begin(), f.end());
    //    std::vector<Eigen::Vector3i>::iterator iter = std::find();
    //}

    std::vector<std::vector<int>> *vertex_share_faces = model->getVertexShareFaces();
    std::vector<unsigned int> *face_list = model->getFaceList();

    // find share faces of an edge
    std::vector<int> share_face;
    std::set_intersection((*vertex_share_faces)[pi].begin(), (*vertex_share_faces)[pi].end(), (*vertex_share_faces)[pj].begin(), (*vertex_share_faces)[pj].end(), std::back_inserter(share_face));

    if (share_face.size() == 2)
    {
        // set edge points
        std::vector<int> edgePoints;
        edgePoints.clear();
        edgePoints.push_back(pi);
        edgePoints.push_back(pj);

        // set all points in the two faces
        int f0_0 = (*face_list)[3 * share_face[0] + 0];
        int f0_1 = (*face_list)[3 * share_face[0] + 1];
        int f0_2 = (*face_list)[3 * share_face[0] + 2];

        int f1_0 = (*face_list)[3 * share_face[1] + 0];
        int f1_1 = (*face_list)[3 * share_face[1] + 1];
        int f1_2 = (*face_list)[3 * share_face[1] + 2];

        if (f0_0 != pi && f0_0 != pj) cross_pi = f0_0;
        else if (f0_1 != pi && f0_1 != pj) cross_pi = f0_1;
        else if (f0_2 != pi && f0_2 != pj) cross_pi = f0_2;

        if (f1_0 != pi && f1_0 != pj) cross_pj = f1_0;
        else if (f1_1 != pi && f1_1 != pj) cross_pj = f1_1;
        else if (f1_2 != pi && f1_2 != pj) cross_pj = f1_2;

        return true;
    }
    else
    {
        std::cout << "Error: number of shared face is more than 2...\n";
        return false;
    }
}

void GeometryPartAlg::getConnectedPtID(int i_pt, int points_in_face[3], int connect_pt[2])
{
    if (points_in_face[0] == i_pt) {
        connect_pt[0] = points_in_face[1];
        connect_pt[1] = points_in_face[2];
    }
    else if (points_in_face[1] == i_pt) {
        connect_pt[0] = points_in_face[0];
        connect_pt[1] = points_in_face[2];
    }
    else if (points_in_face[2] == i_pt)
    {
        connect_pt[0] = points_in_face[0];
        connect_pt[1] = points_in_face[1];
    } 
    else
    {
        std::cout << "Error: can not find point in the given face!\n";
        // get the other point's id in this cell
    }
}