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
#include <getopt.h>
extern int optind, opterr, optopt;
extern char *optargi;
static struct option long_options[] =
    {
        {"help", no_argument, NULL, 'h'},
        {"initRand", required_argument, NULL, 'r'},
        {"initMA", no_argument, NULL, 'a'},
        {"initMS", no_argument, NULL, 's'},
        {"initMD", no_argument, NULL, 'd'},
        {"belief_gap", required_argument, NULL, 'b'},
        {"number_restarts", required_argument, NULL, 'n'},
        {"timeout_pomcp", required_argument, NULL, 't'},
        {"max_fsc_size", required_argument, NULL, 'f'},
};

using namespace std;

//  --- Define experiments paths, parameters used for the UAI submission ---
// FOR REPRODUCING THE MCJESP RESULTS IN UAI, PLEASE DO NOT CHANGE FOLLOWING VALUES!
const string decpomdp_path0 = "./Problems/Grid3x3corners.dpomdp";
const string decpomdp_path1 = "./Problems/boxPushingUAI07.dpomdp";
const string decpomdp_path2 = "./Problems/dectiger.dpomdp";
const string decpomdp_path3 = "./Problems/Mars.dpomdp";
const string decpomdp_path4 = "./Problems/recycling.dpomdp";
const vector<string> benchmarks = {decpomdp_path0, decpomdp_path1, decpomdp_path2, decpomdp_path3, decpomdp_path4};
const vector<string> names = {"grid", "box", "dectiger", "mars", "recycling"};
const vector<string> heuristic_type_names = {"MA", "MD", "MS"};
const vector<double> params_pomcp_c = {2.0, 5.0, 2.0, 2.0, 0.5};
const vector<int> params_pomcp_nb_rollout = {1, 200, 100, 200, 1};
const vector<int> params_pomcp_depth = {20, 40, 20, 15, 10};
const double pomcp_epsilon = 0.01;

void UsageManual();
int main(int argc, char *argv[])
{
    srand(time(NULL));
    // ----- MC-JESP parameters -------
    int restart = 1;            // default
    double timeout = 1;         // default
    int max_fsc_node_size = 50; // default
    bool random_init = false;   // default
    int heuristic_type = 0;     // 0 is default M-A; 1 is M-D; 2 is M-S; 3 is random
    if (heuristic_type == 3)
    {
        random_init = true;
    }
    int max_random_init_fsc_size = 5; // default
    double max_belief_gap = 0.1;      // default
    // -----------------------------------

    int index = 0;
    int c = 0;
    while (EOF != (c = getopt_long(argc, argv, "hasdr:b:n:t:f:", long_options, &index)))
    {
        switch (c)
        {
        case 'h':
            // printf("we get option -h，index %d\n",index);
            UsageManual();
            return 0;
        case 'a':
            heuristic_type = 0;
            break;
        case 's':
            heuristic_type = 2;
            break;
        case 'd':
            heuristic_type = 1;
            break;
        case 'f':
            max_fsc_node_size = stoi(optarg);
            break;
        case 't':
            timeout = stod(optarg);
            break;
        case 'n':
            restart = stoi(optarg);
            break;
        case 'b':
            max_belief_gap = stod(optarg);
            break;
        case 'r':
            heuristic_type = 3;
            max_random_init_fsc_size = stoi(optarg);
            break;
        case '?':
            printf("unknow option:%c\n", optopt);
            break;
        default:
            break;
        }
    }

    for (size_t i_benchmark = 0; i_benchmark < names.size(); i_benchmark++)
    {
        string decpomdp_path = benchmarks[i_benchmark];
        string decpomdp_name = names[i_benchmark];

        DecPomdpInterface *decpomdp = new ParsedDecPOMDPSparse(decpomdp_path);
        double pomcp_c = params_pomcp_c[i_benchmark];
        int nb_rollout = params_pomcp_nb_rollout[i_benchmark];
        int max_pomcp_depth = params_pomcp_depth[i_benchmark];

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

        SimModel sim_decpomdp(decpomdp);

        SimInterface *sim = new SimModel(decpomdp);

        for (int i = 0; i < restart; i++)
        {
            MCJESP mcjesp(&sim_decpomdp);
            mcjesp.Init(max_fsc_node_size, max_pomcp_depth, pomcp_c, nb_rollout, timeout, pomcp_epsilon, max_belief_gap);

            if (!random_init)
            {
                mcjesp.Init_heuristic(sim, heuristic_type);
            }
            else
            {
                mcjesp.Init_random(max_random_init_fsc_size);
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
        delete decpomdp;
    }

    return 0;
}

void UsageManual()
{
    printf("MC-JESP Experiments for Dec-POMDP Benchamrks:\n\n");
    printf("=================================\n");
    printf("COMMAND-LINE OPTIONS\n");
    printf("=================================\n\n");
    printf("Miscellaneous options:\n\n");
    printf("    -h or --help                        Print user guide.\n\n");
    printf("Inf-JESP solver options:\n\n");
    printf("    -a or --initMA                      Initialization with MA heuristic method. No additional args needed.\n");
    printf("    -s or --initMS                      Initialization with MS heuristic method. No additional args needed.\n");
    printf("    -d or --initMD                      Initialization with MD heuristic method. No additional args needed.\n");
    printf("    -r or --initRand                    Initialization with random FSCs. Need to give the max random FSC size.\n\n");
    printf("Planner and evaluation options:\n\n");
    printf("    -f or --max_fsc_size                Set a max FSC size for each agent. Default is 50.\n\n");
    printf("POMCP options:\n\n");
    printf("    -t or --timeout_pomcp               Set a POMCP timeout value. Default is 5s.\n\n");
    printf("Restarts options:\n\n");
    printf("    -n or --number_restarts             Set a restarts number. Default is 1.\n\n");
    printf("Belief gap accepted:\n\n");
    printf("    -b or --belief_gap                  Define a belief gap, belief points within this distance gap will be merged.\n\n");
};