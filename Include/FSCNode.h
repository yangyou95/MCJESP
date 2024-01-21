/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _FSCNODE_H_
#define _FSCNODE_H_

#include "BeliefParticles.h"
using namespace std;

struct FSCNode
{
    BeliefParticles belief;
    int best_aI;
    int JaI;
    double weight;
    int depth;
    map<int, bool> _bool_local_obs;
};

#endif
