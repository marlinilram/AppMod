#ifndef GeometryPartAlg_H
#define GeometryPartAlg_H

#include <cv.h>

#include "Coarse.h"

class GeometryPartAlg
{
public:
    GeometryPartAlg();
    ~GeometryPartAlg();

    void updateGeometry(Coarse *model);
    bool findShareVertex(int pi, int pj, Coarse *model, int &cross_pi, int &cross_pj);
    void getConnectedPtID(int i_pt, int points_in_face[3], int connect_pt[2]);
    void test(Coarse* model);
    void unprojectVector(Coarse* model, float* vec);

public:
    MPara *m_para;
};

#endif