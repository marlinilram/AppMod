#ifndef Groundtruth_H
#define Groundtruth_H

#include "model.h"
#include "Coarse.h"

class Coarse;

class Groundtruth : public Model
{

public:
    Groundtruth() {};
    ~Groundtruth() {};

    Groundtruth(Coarse *model);
    
    void computeErrorColor(Coarse *model);

};


#endif