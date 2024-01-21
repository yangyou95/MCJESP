/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _POMCP_H_
#define _POMCP_H_

// #include "SimModel.h"
#include "SimInterface.h"
#include "TreeNode.h"
#include "Utils.h"
#include "time.h"

class PomcpPlanner
{
private:
      int max_depth = 100;
      TreeNode *rootnode = nullptr;
      SimInterface *simulator; // should change it to const
      int size_A;
      int nb_restarts_simulation = 1; // default
      double epsilon = 0.01;
      double discount;
      vector<TreeNode> all_nodes;
      // BeliefParticles b0;
      double timeout = 0;
      double c;

      map<int, bool> Root_best_action_possible_obs;

public:
      PomcpPlanner(){};
      PomcpPlanner(SimInterface *sim, double discount);
      ~PomcpPlanner();
      void Init(double c, int pomcp_nb_rollout, double timeout, double threshold, int max_depth);
      int Search(BeliefParticles b);
      double Rollout(int sampled_sI, int node_depth); // random policy simulation
      double Simulate(int sampled_sI, TreeNode *node, int depth);
      TreeNode *CreateNewNode(TreeNode *parent_node, int aI, int oI);
      int UcbActionSelection(TreeNode *node) const;
      map<int, bool> GiveRootActionPossibleObservation();
      // tool functions for building the FSC result
      // int GiveSimilarNodeIndex(BeliefParticles &b_temp, double &min_distance);
      // int CheckBeliefExist(BeliefParticles &b, double max_accept_gap);
      // bool CheckChildNodeExist(int belief_id, int aI, int oI);
      // void BuildFSC();
      // void ProcessNode(int node_index);
      // int CheckBeliefExistInFSC(int belief_id);
      // void ExportFSC(string filename);
      // void PrintGraph();
      // void BuildHeuristicFSC(int agent_Index);
      // void ProcessNodeHeuristic(int node_index, int agent_Index);
      // void GetFSC(vector<vector<vector<int>>> &out_eta_fsc, vector<FSCNode> &out_FscNodes);
};

#endif