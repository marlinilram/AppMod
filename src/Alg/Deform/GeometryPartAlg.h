#ifndef GeometryPartAlg_H
#define GeometryPartAlg_H

#include <Eigen\Eigen>
#include <Eigen\Sparse>

class Coarse;
class MPara;

class GeometryPartAlg
{
public:
    GeometryPartAlg();
    ~GeometryPartAlg();

    void updateGeometry(Coarse *model);
    void updateScreenShape(Coarse *model, Eigen::VectorXf& P_Opt);
    bool findShareVertex(int pi, int pj, Coarse *model, int &cross_pi, int &cross_pj);
    void getConnectedPtID(int i_pt, int points_in_face[3], int connect_pt[2]);
    void updateWithExNormal(Coarse* model);
    void unprojectVector(Coarse* model, float* vec);
    void test(Coarse* model);

public:
    MPara *m_para;
};

#endif