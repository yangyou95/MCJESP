/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _MCJESP_H_
#define _MCJESP_H_

#include "FSC.h"
// #include "SimModel.h"
#include "SimInterface.h"
#include "ExtendedGenerativeModel.h"

using namespace std;

class MCJESP
{
private:
    SimInterface *sim_decpomdp;
    double V_max = -DBL_MAX;
    vector<FSC> FSCs;
    vector<double> V_history;
    int OptimizingAgentIndex = -1;
    int iter_max = 1000;

    // ---- parameters ----
    int size_max_fsc_node;
    int depth_max_pomcp;
    double pomcp_c;
    int pomcp_nb_rollout;
    double pomcp_timeout;
    double error_gap;
    double max_accept_belief_gap;
    // --------------------

    // ---- storing best response values -----
    vector<vector<vector<double>>> eta_fsc; // nI-> oI -> nI'
    vector<FSCNode> FscNodes;
    int fsc_agentI = -1;
    // ---------------------------------------

    // ---- storing best FSCs ----
    map<int, FSC> best_fscs;
    // ---------------------------

public:
    // using heuristics by default
    MCJESP(SimInterface *sim_decpomdp);
    ~MCJESP(){};

    void Init(int max_fsc_node_size,
              int pomcp_max_depth,
              double pomcp_c,
              int pomcp_nb_rollout,
              double pomcp_timeout,
              double epsilon,
              double max_accept_belief_gap);

    void Init_random(int max_fsc_size);

    void Init_heuristic(SimInterface *sim, int heuristic_type);

    double Plan(int current_restart, ofstream &out);
    int GetNextAgentIndex();

    vector<FSC> GetFinalFSCs();
};

#endif