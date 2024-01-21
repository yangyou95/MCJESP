/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _FSC_H_
#define _FSC_H_

#include <map>
#include <vector>
#include "Utils.h"
#include "Planner.h"

class FSC
{
private:
    // parameters
    int max_node_size = 200;
    int size_FSC = -1;
    int size_local_A = -1;
    int size_local_Obs = -1;
    int current_agentI = -1;
    int max_fsc_reached_depth = -1;
    double epsilon = 0.0;

    SimInterface *sim;
    PomcpPlanner pomcp;
    double max_belief_gap = 0.02;
    // ---- fsc elements ----
    vector<vector<vector<double>>> eta_fsc; // nI-> oI -> nI'
    vector<FSCNode> FscNodes;
    // ---- fsc elements ----
    int heuristic_agent_index = -1; // -1 is no heuristic, solving a POMDP
    int heuristic_type = 0;         // 0 is default M-A; 1 is M-D; 2 is M-S
    // process functions
    void ProcessNodeWithPomcp(BeliefParticles &b, FSCNode &n);
    BeliefParticles FscUpdateBelief(BeliefParticles &b, FSCNode &n, int oI, double &out_pr_o_ba);

public:
    FSC(){};
    FSC(SimInterface *sim, int agentI);
    // used for random a FSC
    FSC(int MaxNodeSize, int ActionsSize, int ObsSize);
    ~FSC(){};
    void Init(double max_accept_belief_gap, int max_node_size,
              int pomcp_depth, double pomcp_time_out, double pomcp_c, int pomcp_nb_rollout, double epsilon);
    void SetHeuristicAgent(int heuristic_agent_index, int heuristic_type);
    void BuildFSC();
    int GetNodeIndex(FSCNode &n);
    void ProcessNodeWithMaxSize(FSCNode &node_process, map<double, vector<FSCNode>> &UnProcessedSet);
    void ProcessNodeWithMaxSizeMD(FSCNode &node_process, map<double, vector<FSCNode>> &UnProcessedSet);
    void ProcessNodeWithMaxSizeMS(FSCNode &node_process, map<double, vector<FSCNode>> &UnProcessedSet);
    int CheckBeliefExist(BeliefParticles &b, double max_accept_gap);
    int GiveSimilarNodeIndex(BeliefParticles &b_temp, double &min_distance);
    int GetSizeNodes();

    // tool functions for having the FSC result
    void ExportFSC(string filename);
    void PrintGraph();
    void GetFSC(vector<vector<vector<double>>> &out_eta_fsc, vector<FSCNode> &out_FscNodes, int &out_fsc_agentI);

    // used for generate random FSC
    void GenerateFSCwithParameters(int random_FSC_size, int ActionsSize, int ObsSize, vector<double> &matrix_nodes_actions_pb_dist, vector<double> &matrix_nodes_transition_pb_dist);
};

#endif