#include <iostream>
#include "Include/ParserPOMDPSparse.h"
#include "Include/ParserDecPOMDPSparse.h"
#include "Include/Planner.h"
#include "Include/SimModel.h"
#include "Include/BeliefParticles.h"
#include "Include/TreeNode.h"
#include "Include/ExtendedGenerativeModel.h"
#include "Include/FSC.h"
#include "Include/MCJESP.h"

// 1. First, include your own Dec-POMDP header file

using namespace std;

int main()
{
    srand(time(NULL));

    // ----- 2. Loading your Dec-POMDP simulator -------
    // SimModel sim_decpomdp(custom_decpomdp); // using a existing dec-pomdp file (explicit model);
    // SimInterface *sim = new SimModel(); // or using the SimInterface to model a custom dec-pomdp simulator
    // string decpomdp_name = "?";


    // ----- 3. (Optional) Tunning parameters -------
    int restart = 1;
    double pomcp_c = 3.0;       // default
    int nb_rollout = 1;         // default
    double epsilon = 0.01;      // default
    int max_pomcp_depth = 30;   // default
    double timeout = 5;         // default
    int max_fsc_node_size = 50; // default
    int heuristic_type = 0;     // 0 is default M-A; 1 is M-D; 2 is M-S
    bool random_init = false;
    int max_fsc_size = 5;
    double max_belief_gap = 0.1;
    // -----------------------------------

    string outfile;
    string out_name = "MCJESP_log_" + decpomdp_name + "_";
    outfile += out_name + "maxFsc_" + to_string(max_fsc_node_size) + "_" + to_string(restart) + heuristic_type_names[heuristic_type] + ".csv";
    ofstream outlogs;
    outlogs.open(outfile.c_str());
    outlogs << "Restart"
            << ","
            << "Iteration"
            << ","
            << "AgentI"
            << ","
            << "Value"
            << ","
            << "IterTime"
            << ","
            << "V_max_Simulation, TotalTime, Iterations, FSC size" << endl;

    for (int i = 0; i < restart; i++)
    {
        MCJESP mcjesp(sim_decpomdp);
        mcjesp.Init(max_fsc_node_size, max_pomcp_depth, pomcp_c, nb_rollout, timeout, epsilon, max_belief_gap);

        if (!random_init)
        {
            mcjesp.Init_heuristic(sim, heuristic_type);
        }
        else
        {
            mcjesp.Init_random(max_fsc_size);
        }

        mcjesp.Plan(i, outlogs);
        vector<FSC> final_fscs = mcjesp.GetFinalFSCs();
        for (size_t j = 0; j < final_fscs.size(); j++)
        {
            string fsc_path = "./TempFiles/" + decpomdp_name + "_fsc" + to_string(j) + "timeout" + to_string(timeout) + "maxN" + to_string(max_fsc_node_size) + "restart" + to_string(i);
            final_fscs[j].ExportFSC(fsc_path);
        }
    }
    delete sim;

    return 0;
}
