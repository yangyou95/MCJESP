/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _SIMINTERFACE_H_
#define _SIMINTERFACE_H_

#include "PomdpInterface.h"
#include "DecPomdpInterface.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>
#include <cmath>
#include "FSCNode.h"
using namespace std;

class SimInterface
{
private:
    /* data */
public:
    SimInterface(){};
    virtual ~SimInterface(){};

    // ------- obligate functions ----------
    virtual tuple<int, int, double, bool> Step(int sI, int aI) = 0; // sI_next, oI, Reward, Done
    virtual int SampleStartState() = 0;
    virtual int GetSizeOfObs() const = 0;
    virtual int GetSizeOfA() const = 0;
    virtual double GetDiscount() const = 0;
    virtual int GetNbAgent() const = 0;
    // --------------------------------------------------------

    //------- for heuristic MPOMDP -----------
    virtual int IndividualToJointActionIndex(vector<int> &action_indices) const
    {
        (void)(action_indices);
        return -1;
    }
    virtual vector<int> JointToIndividualObsIndices(int JoI) const
    {
        (void)(JoI);
        return {};
    }
    virtual vector<int> JointToIndividualActionIndices(int JaI) const
    {
        (void)(JaI);
        return {};
    }

    virtual int GiveLocalObsIndex(int JoI, int agentI) const
    {
        (void)(JoI);
        (void)(agentI);
        return -1;
    }
    virtual int GiveLocalActionIndex(int JaI, int agentI) const
    {
        (void)(JaI);
        (void)(agentI);
        return -1;
    }

    virtual int GetSizeOfJointObs() const
    {
        return -1;
    }
    virtual int GetSizeOfLocalObs(int agentI) const
    {
        (void)(agentI);
        return -1;
    }

    virtual int GetSizeOfLocalA(int agentI) const
    {
        (void)(agentI);
        return -1;
    }

    // for Extended Generator

    virtual double PolicyEvaluation(vector<vector<vector<double>>> &eta_fsc_optimizing_agent,
                                    vector<FSCNode> &FscNodes_optimizing_agent)
    {
        (void)(eta_fsc_optimizing_agent);
        (void)(FscNodes_optimizing_agent);
        return 0;
    }
};

#endif