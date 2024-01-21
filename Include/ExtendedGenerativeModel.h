/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _EXTENDEDGENERATOR_H_
#define _EXTENDEDGENERATOR_H_

#include "SimInterface.h"
#include "SimModel.h"

class ExtendedGenerativeModel : public SimInterface
{
private:
    SimInterface *sim_model_decpomdp;
    vector<vector<vector<double>>> eta_fsc;
    vector<FSCNode> FscNodes;
    int size_FSC = -1;
    vector<vector<int>> _m_ExtendedStateIndicies;          // vector (eI -> {sI,NI,oI})
    map<vector<int>, int> _m_IndiciesToExtendedStateIndex; // map ({sI,NI,oI} -> eI)
    map<int, double> init_eI_dist;

    int fsc_agent = -1;
    int optimizing_agent = -1;

    int GetNextNI(int nI, int oI, int size_optimizing_fsc, vector<vector<vector<double>>> &Eta_fsc);
    int GiveExtendedStateIndex(vector<int> &e);

public:
    ExtendedGenerativeModel(){};
    ~ExtendedGenerativeModel(){};
    ExtendedGenerativeModel(SimInterface *sim_model_decpomdp, vector<vector<vector<double>>> &eta_fsc,
                            vector<FSCNode> &FscNodes, int fsc_agentI, int optimizing_agentI);

    // ------- obligate functions from sim interface ----------
    // be careful here, sI is the extended state index, aI is the optimizing agent's action index
    tuple<int, int, double, bool> Step(int sI, int aI); // sI_next, oI, Reward, Done
    int SampleStartState();
    int GetSizeOfObs() const;
    int GetSizeOfA() const;
    double GetDiscount() const;
    int GetNbAgent() const;

    // --------------------------------------------------------

    double PolicyEvaluation(vector<vector<vector<double>>> &eta_fsc_optimizing_agent,
                            vector<FSCNode> &FscNodes_optimizing_agent);
};

#endif