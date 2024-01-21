#include "../Include/MCJESP.h"

MCJESP::MCJESP(SimInterface *sim_decpomdp)
{
    this->sim_decpomdp = sim_decpomdp;
}

void MCJESP::Init(
    int max_fsc_node_size,
    int pomcp_max_depth,
    double pomcp_c,
    int pomcp_nb_rollout,
    double pomcp_timeout,
    double epsilon,
    double max_accept_belief_gap)
{
    // this->size_belief_particles = belief_particles_nb;
    this->size_max_fsc_node = max_fsc_node_size;
    this->depth_max_pomcp = pomcp_max_depth;
    this->pomcp_c = pomcp_c;
    this->pomcp_nb_rollout = pomcp_nb_rollout;
    this->pomcp_timeout = pomcp_timeout;
    this->error_gap = epsilon;
    this->max_accept_belief_gap = max_accept_belief_gap;
}

void MCJESP::Init_heuristic(SimInterface *sim, int heuristic_type)
{
    cout << "------ Init Process of MCJESP -------" << endl;
    string fsc_heuristic_path = "fsc_heuristic";

    this->OptimizingAgentIndex = this->GetNextAgentIndex();
    FSC fsc_heuristic(sim, OptimizingAgentIndex);
    fsc_heuristic.Init(max_accept_belief_gap, size_max_fsc_node, depth_max_pomcp, pomcp_timeout, pomcp_c, pomcp_nb_rollout, error_gap);
    fsc_heuristic.SetHeuristicAgent(OptimizingAgentIndex, heuristic_type);
    fsc_heuristic.BuildFSC();
    fsc_heuristic.ExportFSC(fsc_heuristic_path);
    // fsc_heuristic.PrintGraph();
    fsc_heuristic.GetFSC(this->eta_fsc, this->FscNodes, this->fsc_agentI);

    this->best_fscs[OptimizingAgentIndex] = fsc_heuristic;
}

void MCJESP::Init_random(int max_fsc_size)
{
    cout << "------ Init Process of MCJESP -------" << endl;
    string fsc_random_path = "fsc_random";

    this->OptimizingAgentIndex = this->GetNextAgentIndex();
    int local_action_size = this->sim_decpomdp->GetSizeOfLocalA(OptimizingAgentIndex);
    int local_obs_size = this->sim_decpomdp->GetSizeOfLocalObs(OptimizingAgentIndex);

    FSC fsc_random(max_fsc_size, local_action_size, local_obs_size);
    fsc_random.ExportFSC(fsc_random_path);
    // fsc_random.PrintGraph();
    fsc_random.GetFSC(this->eta_fsc, this->FscNodes, this->fsc_agentI);
    this->fsc_agentI = OptimizingAgentIndex;
    this->best_fscs[OptimizingAgentIndex] = fsc_random;

    cout << "Init random finish!" << endl;
}

double MCJESP::Plan(int current_restart, ofstream &out)
{

    time_t StartTime, EndTime;
    StartTime = time(NULL);
    int iter = 0;
    int nr_non_improving_agents = 0;
    vector<string> log_res;

    // ------------ Main Algo of MCJESP -------------------
    while (nr_non_improving_agents < this->sim_decpomdp->GetNbAgent() - 1 && iter < this->iter_max)
    {
        cout << "------ iter " << iter << " -------" << endl;

        time_t iter_StartTime, iter_EndTime;
        iter_StartTime = time(NULL);

        // Get The Current Optimizing agent
        this->OptimizingAgentIndex = this->GetNextAgentIndex();
        // ---- init a fsc for current optimizing agent -----
        FSC fsc_temp;
        string fsc_temp_path = "fsc_temp_iter" + to_string(iter) + "_agent" + to_string(OptimizingAgentIndex);
        // ------------------------------------------------------

        SimInterface *sim = new ExtendedGenerativeModel(this->sim_decpomdp, eta_fsc, FscNodes, fsc_agentI, OptimizingAgentIndex);

        fsc_temp = FSC(sim, this->OptimizingAgentIndex);
        fsc_temp.Init(max_accept_belief_gap, size_max_fsc_node, depth_max_pomcp, pomcp_timeout, pomcp_c, pomcp_nb_rollout, error_gap);
        fsc_temp.BuildFSC();
        fsc_temp.GetFSC(eta_fsc, FscNodes, fsc_agentI);
        // fsc_temp.ExportFSC(fsc_temp_path);
        fsc_temp.PrintGraph();
        // evaluate the value
        cout << "--- evaluate the learnt fsc ---" << endl;
        double value_evaluated = sim->PolicyEvaluation(eta_fsc, FscNodes);
        this->V_history.push_back(value_evaluated);

        //  ------------ Value Improvment Check -------------
        if (value_evaluated > this->V_max)
        {

            cout << " --- New Best Policy Found, V:" << value_evaluated << " --- " << endl;
            this->best_fscs[OptimizingAgentIndex] = fsc_temp;
            // fsc_temp.ExportFSC(fsc_temp_path);
            // fsc_temp.PrintGraph();
            this->V_max = value_evaluated;
            nr_non_improving_agents = 0;
        }
        else
        {
            nr_non_improving_agents++;
        }

        delete sim;

        iter_EndTime = time(NULL);
        iter += 1;
        double iter_time = (double)(iter_EndTime - iter_StartTime);

        // log result
        if (value_evaluated >= this->V_max)
        // if (V_alphavecs > this->V_max)
        {
            out << current_restart << "," << iter << "," << this->OptimizingAgentIndex << "," << value_evaluated << "," << iter_time << endl;
            // should not contain the last iteration's information if there is no improvement (or decerase of value)
        }
    }

    EndTime = time(NULL);
    double total_time = (double)(EndTime - StartTime);

    out << " , , , , ," << this->V_max << "," << total_time << "," << iter;

    out << " , ";

    // output optimal fscs
    for (size_t i = 0; i < this->best_fscs.size(); i++)
    {
        string fsc_result_path = "final_fsc_agent" + to_string(i);
        this->best_fscs[i].ExportFSC(fsc_result_path);
        out << best_fscs[i].GetSizeNodes() << " ";
    }
    out << endl;
    cout << "--- MCJESP Planning Finished --- " << endl;

    cout << "V final: " << this->V_max << endl;
    cout << "V history: ";
    for (int i = 0; i < int(this->V_history.size()); i++)
    {
        cout << this->V_history[i] << ",";
    }
    cout << endl;

    return this->V_max;
}

int MCJESP::GetNextAgentIndex()
{
    int next_agentI = (this->OptimizingAgentIndex + 1) % this->sim_decpomdp->GetNbAgent();
    return (next_agentI);
}

vector<FSC> MCJESP::GetFinalFSCs()
{
    vector<FSC> res;
    for (size_t i = 0; i < this->best_fscs.size(); i++)
    {
        res.push_back(this->best_fscs[i]);
    }
    return res;
}