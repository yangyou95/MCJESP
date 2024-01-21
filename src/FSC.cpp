#include "../Include/FSC.h"
#define Randmod(x) rand() % x

FSC::FSC(SimInterface *sim, int agentI)
{
    this->sim = sim;
    this->current_agentI = agentI;
}

void FSC::Init(double max_accept_belief_gap, int max_node_size,
               int pomcp_depth, double pomcp_time_out, double pomcp_c, int pomcp_nb_rollout, double epsilon)
{
    this->size_local_A = sim->GetSizeOfA();
    this->size_local_Obs = sim->GetSizeOfObs();
    this->max_belief_gap = max_accept_belief_gap;
    this->max_node_size = max_node_size;
    double discount = sim->GetDiscount();
    PomcpPlanner pomcp_planner(sim, discount);
    pomcp_planner.Init(pomcp_c, pomcp_nb_rollout, pomcp_time_out, epsilon, pomcp_depth);
    this->pomcp = pomcp_planner;
    this->epsilon = epsilon;
}

void FSC::SetHeuristicAgent(int heuristic_agent_index, int heuristic_type)
{
    this->heuristic_agent_index = heuristic_agent_index;
    this->heuristic_type = heuristic_type;
    this->size_local_Obs = this->sim->GetSizeOfLocalObs(heuristic_agent_index);
    if (heuristic_agent_index != current_agentI)
    {
        cout << "heuristic agent is not the agent of FSC, wrong argument!" << endl;
        throw "";
    }
}

int FSC::GetSizeNodes()
{
    return this->size_FSC;
}

void FSC::BuildFSC()
{
    map<double, vector<FSCNode>> UnProcessedSet;                                                                   // Initlize a map to store the unprocessed nodes, Call it openlist
    this->eta_fsc.resize(max_node_size, vector<vector<double>>(size_local_Obs, vector<double>(max_node_size, 0))); // resize the eta

    BeliefParticles b0 = BuildInitDist(sim);

    FSCNode n0;
    double weight_init = 1.0;
    n0.weight = weight_init;
    n0.belief = b0;
    n0.depth = 0;
    ProcessNodeWithPomcp(b0, n0);
    this->FscNodes.push_back(n0);
    UnProcessedSet[weight_init].push_back(n0);

    while (!UnProcessedSet.empty())
    {
        FSCNode n = UnProcessedSet.rbegin()->second.front();
        int n_index = this->GetNodeIndex(n);

        UnProcessedSet.rbegin()->second.erase(UnProcessedSet.rbegin()->second.begin());
        if (UnProcessedSet.rbegin()->second.size() == 0)
        {
            double key = UnProcessedSet.rbegin()->first;
            UnProcessedSet.erase(key);
        }

        if (heuristic_agent_index >= 0 && heuristic_type == 1)
        {
            ProcessNodeWithMaxSizeMD(n, UnProcessedSet);
        }
        else if (heuristic_agent_index >= 0 && heuristic_type == 2)
        {
            ProcessNodeWithMaxSizeMS(n, UnProcessedSet);
        }
        else
        {
            ProcessNodeWithMaxSize(n, UnProcessedSet);
        }

        cout << "max node size: " << this->max_node_size << endl;
        cout << "fsc node size: " << this->FscNodes.size() << endl;
        cout << "Open list size: " << UnProcessedSet.size() << endl;
        cout << "Processed node index: " << n_index << endl;
    }
    cout << "fsc max reached depth is " << this->max_fsc_reached_depth << endl;
    this->size_FSC = this->FscNodes.size();
}

void FSC::ProcessNodeWithMaxSize(FSCNode &node_process, map<double, vector<FSCNode>> &UnProcessedSet)
{
    FSCNode n = node_process;
    int n_index = this->GetNodeIndex(node_process);
    BeliefParticles b = n.belief;
    map<int, bool> possible_oIs = n._bool_local_obs;
    double weight = n.weight;
    int node_depth = n.depth;

    map<int, double> all_pr_oba;
    map<int, BeliefParticles> all_next_beliefs;
    if (heuristic_agent_index != -1)
    {
        GetNextBeliefsOneSidedUpdate(*sim, b, n.JaI, heuristic_agent_index, all_pr_oba, all_next_beliefs);
    }
    else
    {
        GetNextBeliefs(*sim, b, n.best_aI, all_pr_oba, all_next_beliefs);
    }

    for (int oI = 0; oI < this->size_local_Obs; oI++)
    {
        if (!all_pr_oba.count(oI))
        {
            this->eta_fsc[n_index][oI][n_index] = 1; // Point back to the current node
        }
        else
        {
            double pr_o_ba = all_pr_oba[oI];
            BeliefParticles b_new = all_next_beliefs[oI];
            double new_node_weight = pr_o_ba * weight;

            int FLAG_Exist = CheckBeliefExist(b_new, this->max_belief_gap);
            // if belief not exist and node size is OK
            if (FLAG_Exist == -1 && int(this->FscNodes.size()) < this->max_node_size)
            {
                FSCNode new_node;
                new_node.weight = new_node_weight;
                new_node.belief = b_new;
                new_node.depth = node_depth + 1;
                ProcessNodeWithPomcp(b_new, new_node);
                this->FscNodes.push_back(new_node);
                UnProcessedSet[new_node_weight].push_back(new_node);
                int new_node_Index = this->FscNodes.size() - 1;
                this->eta_fsc[n_index][oI][new_node_Index] = 1;

                if (new_node.depth > this->max_fsc_reached_depth)
                {
                    this->max_fsc_reached_depth = new_node.depth;
                }
            }
            else
            {
                // if max node size is reached
                if (int(this->FscNodes.size()) >= this->max_node_size)
                {
                    double similiar_distance = 0.0;
                    int similiar_nI = this->GiveSimilarNodeIndex(b_new, similiar_distance);
                    this->eta_fsc[n_index][oI][similiar_nI] = 1;
                    double w_updated = this->FscNodes[similiar_nI].weight;
                    w_updated += new_node_weight;
                    this->FscNodes[similiar_nI].weight = w_updated;
                }
                // if belief already exist
                else
                {
                    double w_updated = this->FscNodes[FLAG_Exist].weight;
                    w_updated += new_node_weight;
                    this->FscNodes[FLAG_Exist].weight = w_updated;
                    // if this new belief already exist, link to the node with existed alpha vector
                    this->eta_fsc[n_index][oI][FLAG_Exist] = 1;
                }
            }
        }
    }
}

// MD heuristic
void FSC::ProcessNodeWithMaxSizeMD(FSCNode &node_process, map<double, vector<FSCNode>> &UnProcessedSet)
{
    FSCNode n = node_process;
    int n_index = this->GetNodeIndex(node_process);
    BeliefParticles b = n.belief;
    map<int, bool> possible_oIs = n._bool_local_obs;
    double weight = n.weight;
    int node_depth = n.depth;
    // int best_aI = n.best_aI;

    map<int, double> all_pr_oba;
    map<int, BeliefParticles> all_next_beliefs;
    GetNextBeliefs(*sim, b, n.JaI, all_pr_oba, all_next_beliefs);
    for (int oI = 0; oI < this->size_local_Obs; oI++)
    {
        // if (!possible_oIs[oI])
        if (!all_pr_oba.count(oI))
        {
            this->eta_fsc[n_index][oI][n_index] = 1; // Point back to the current node
        }
        else
        {
            // Find JOI with highest pb to update a new belief
            map<int, double>::iterator it_pr_oba;
            int JOI_selected = -1;
            double highest_pr_oba = 0.0;
            for (it_pr_oba = all_pr_oba.begin(); it_pr_oba != all_pr_oba.end(); it_pr_oba++)
            {
                if (it_pr_oba->second > highest_pr_oba)
                {
                    highest_pr_oba = it_pr_oba->second;
                    JOI_selected = it_pr_oba->first;
                }
            }

            BeliefParticles b_new = all_next_beliefs[JOI_selected];
            double new_node_weight = highest_pr_oba * weight;

            int FLAG_Exist = CheckBeliefExist(b_new, this->max_belief_gap);
            // if belief not exist and node size is OK
            if (FLAG_Exist == -1 && int(this->FscNodes.size()) < this->max_node_size)
            {
                FSCNode new_node;
                new_node.weight = new_node_weight;
                new_node.belief = b_new;
                new_node.depth = node_depth + 1;
                ProcessNodeWithPomcp(b_new, new_node);
                this->FscNodes.push_back(new_node);
                UnProcessedSet[new_node_weight].push_back(new_node);
                int new_node_Index = this->FscNodes.size() - 1;
                this->eta_fsc[n_index][oI][new_node_Index] = 1;

                if (new_node.depth > this->max_fsc_reached_depth)
                {
                    this->max_fsc_reached_depth = new_node.depth;
                }
            }
            else
            {
                // if max node size is reached
                if (int(this->FscNodes.size()) >= this->max_node_size)
                {
                    double similiar_distance = 0.0;
                    int similiar_nI = this->GiveSimilarNodeIndex(b_new, similiar_distance);
                    this->eta_fsc[n_index][oI][similiar_nI] = 1;
                    double w_updated = this->FscNodes[similiar_nI].weight;
                    w_updated += new_node_weight;
                    this->FscNodes[similiar_nI].weight = w_updated;
                }
                // if belief already exist
                else
                {
                    double w_updated = this->FscNodes[FLAG_Exist].weight;
                    w_updated += new_node_weight;
                    this->FscNodes[FLAG_Exist].weight = w_updated;
                    // if this new belief already exist, link to the node with existed alpha vector
                    this->eta_fsc[n_index][oI][FLAG_Exist] = 1;
                }
            }
        }
    }
}

// MS heuristic
void FSC::ProcessNodeWithMaxSizeMS(FSCNode &node_process, map<double, vector<FSCNode>> &UnProcessedSet)
{
    FSCNode n = node_process;
    int n_index = this->GetNodeIndex(node_process);
    BeliefParticles b = n.belief;
    map<int, bool> possible_oIs = n._bool_local_obs;
    double weight = n.weight;
    int node_depth = n.depth;
    // int best_aI = n.best_aI;

    map<int, double> all_pr_oba;
    map<int, BeliefParticles> all_next_beliefs;
    GetNextBeliefs(*sim, b, n.JaI, all_pr_oba, all_next_beliefs);
    for (int oI = 0; oI < this->size_local_Obs; oI++)
    {
        // if (!possible_oIs[oI])
        if (!all_pr_oba.count(oI))
        {
            this->eta_fsc[n_index][oI][n_index] = 1; // Point back to the current node
        }
        else
        {
            // Find JOI with highest pb to update a new belief
            map<int, double>::iterator it_pr_oba;
            int JOI_selected = -1;
            double pr_oba = 0.0;
            for (it_pr_oba = all_pr_oba.begin(); it_pr_oba != all_pr_oba.end(); it_pr_oba++)
            {
                pr_oba = it_pr_oba->second;
                JOI_selected = it_pr_oba->first;
                BeliefParticles b_new = all_next_beliefs[JOI_selected];
                double new_node_weight = pr_oba * weight;

                int FLAG_Exist = CheckBeliefExist(b_new, this->max_belief_gap);
                // if belief not exist and node size is OK
                if (FLAG_Exist == -1 && int(this->FscNodes.size()) < this->max_node_size)
                {
                    FSCNode new_node;
                    new_node.weight = new_node_weight;
                    new_node.belief = b_new;
                    new_node.depth = node_depth + 1;
                    ProcessNodeWithPomcp(b_new, new_node);
                    this->FscNodes.push_back(new_node);
                    UnProcessedSet[new_node_weight].push_back(new_node);
                    int new_node_Index = this->FscNodes.size() - 1;
                    this->eta_fsc[n_index][oI][new_node_Index] = pr_oba;

                    if (new_node.depth > this->max_fsc_reached_depth)
                    {
                        this->max_fsc_reached_depth = new_node.depth;
                    }
                }
                else
                {
                    // if max node size is reached
                    if (int(this->FscNodes.size()) >= this->max_node_size)
                    {
                        double similiar_distance = 0.0;
                        int similiar_nI = this->GiveSimilarNodeIndex(b_new, similiar_distance);
                        this->eta_fsc[n_index][oI][similiar_nI] = pr_oba;
                        double w_updated = this->FscNodes[similiar_nI].weight;
                        w_updated += new_node_weight;
                        this->FscNodes[similiar_nI].weight = w_updated;
                    }
                    // if belief already exist
                    else
                    {
                        double w_updated = this->FscNodes[FLAG_Exist].weight;
                        w_updated += new_node_weight;
                        this->FscNodes[FLAG_Exist].weight = w_updated;
                        // if this new belief already exist, link to the node with existed alpha vector
                        this->eta_fsc[n_index][oI][FLAG_Exist] = pr_oba;
                    }
                }
            }
        }
    }
}

int FSC::GetNodeIndex(FSCNode &n)
{
    for (size_t nI = 0; nI < this->FscNodes.size(); nI++)
    {
        if (this->FscNodes[nI].belief == n.belief)
        {
            return nI;
        }
    }

    cout << "did not found! this node" << endl;
    throw "";

    return -1;
}

int FSC::CheckBeliefExist(BeliefParticles &b, double max_accept_gap)
{
    double min_distance = 0;
    int similiar_node_index = this->GiveSimilarNodeIndex(b, min_distance);
    if (min_distance < max_accept_gap)
    {
        return similiar_node_index;
    }
    else
    {
        return -1;
    }
}

int FSC::GiveSimilarNodeIndex(BeliefParticles &b_temp, double &min_distance)
{
    int similiar_nI = -1;
    double most_close_distance = __DBL_MAX__;
    for (size_t nI = 0; nI < this->FscNodes.size(); nI++)
    {
        BeliefParticles b_nI = this->FscNodes[nI].belief;
        double dis_temp = ComputeNorm1Distance(b_temp, b_nI);
        if (dis_temp < most_close_distance)
        {
            most_close_distance = dis_temp;
            similiar_nI = nI;
        }
    }

    min_distance = most_close_distance;

    return similiar_nI;
}

// process node with best_aI, (JaI), possible_oIs
void FSC::ProcessNodeWithPomcp(BeliefParticles &b, FSCNode &n)
{
    int best_aI = this->pomcp.Search(b);
    map<int, bool> all_obs_possible = this->pomcp.GiveRootActionPossibleObservation();

    if (heuristic_agent_index != -1)
    {
        map<int, bool> possible_local_oIs;

        // prcoess JAI
        int local_best_aI = this->sim->GiveLocalActionIndex(best_aI, heuristic_agent_index);
        n.JaI = best_aI;
        n.best_aI = local_best_aI;
        // process each JOI
        for (auto const &x : all_obs_possible)
        {
            int JoI = x.first;
            int local_oI = this->sim->GiveLocalObsIndex(JoI, heuristic_agent_index);
            if (x.second)
            {
                possible_local_oIs[local_oI] = true;
            }
        }
        n._bool_local_obs = possible_local_oIs;
    }
    else
    {
        n._bool_local_obs = all_obs_possible;
        n.best_aI = best_aI;
    }
}

BeliefParticles FSC::FscUpdateBelief(BeliefParticles &b, FSCNode &n, int oI, double &out_pr_o_ba)
{
    int process_aI = n.best_aI;
    if (heuristic_agent_index != -1)
    {
        process_aI = n.JaI;
        return UpdateOneSideBeliefandProb(*sim, b, process_aI, oI, heuristic_agent_index, out_pr_o_ba);
    }
    else
    {
        return UpdateBeliefandProb(*sim, b, process_aI, oI, out_pr_o_ba);
    }
}

// tool functions for having the FSC result
void FSC::ExportFSC(string filename)
{

    // cout << "--- exporting fsc---" << endl;

    ofstream fp(filename.c_str());
    if (fp.fail())
    {
        cerr << "ERROR" << endl;
        throw std::runtime_error("..");
    }

    fp << "agent: "
       << "AgentI" << endl;
    fp << "nodes: ";

    int nI_start = 0;

    for (int nI = nI_start; nI < size_FSC; nI++)
    {
        fp << this->FscNodes[nI].best_aI << " ";
    }
    fp << endl;

    // T: obs_I : start-node_I : end-node_I %f

    for (int nI = 0; nI < this->size_FSC; nI++)
    {
        for (int oI = 0; oI < this->size_local_Obs; oI++)
        {
            for (int n_newI = 0; n_newI < size_FSC; n_newI++)
            {
                // check prob is not 0
                double temp_pr = this->eta_fsc[nI][oI][n_newI];
                if (temp_pr > 0)
                {
                    fp << "T: " << oI << " : " << nI << " : " << n_newI << " " << temp_pr << endl;
                }
            }
        }
    }

    fp.close();
}
void FSC::PrintGraph()
{
    cout << endl;
    cout << " -------- " << endl;
    cout << "digraph FSC {" << endl;
    // define nodes in Graph
    for (int i = 0; i < this->size_FSC; i++)
    {
        // cout << "n" << i << "[label = \" aH:  " << this->Nodes[i].GetDescript() << "\"]" << endl;
        cout << "n" << i << "[label = \" aI:  " << this->FscNodes[i].best_aI << "\"]" << endl;
    }
    cout << endl;

    for (int nI = 0; nI < this->size_FSC; nI++)
    {
        for (int OI = 0; OI < this->size_local_Obs; OI++)
        {
            for (int n_newI = 0; n_newI < size_FSC; n_newI++)
            {
                double pr_trans = this->eta_fsc[nI][OI][n_newI];
                if (pr_trans > 0 && nI != n_newI)
                {
                    // cout << "n" << nI << " -> "
                    //      << "n" << n_newI << "[label = \"oh: " << pomdp->GetAllObservations()[OI] << ", pb: " << pr_trans << " \"]" << endl;
                    cout << "n" << nI << " -> "
                         << "n" << n_newI << "[label = \"oI: " << OI << ", pb: " << pr_trans << " \"]" << endl;
                }
            }
        }
    }
    cout << "}" << endl;
}

void FSC::GetFSC(vector<vector<vector<double>>> &out_eta_fsc, vector<FSCNode> &out_FscNodes, int &out_fsc_agentI)
{
    out_eta_fsc = this->eta_fsc;
    out_FscNodes = this->FscNodes;
    out_fsc_agentI = this->current_agentI;
}

void FSC::GenerateFSCwithParameters(int random_FSC_size, int ActionsSize, int SizeObs, vector<double> &matrix_nodes_actions_pb_dist, vector<double> &matrix_nodes_transition_pb_dist)
{
    srand(time(NULL));
    rand();
    // cout << "------" << endl;
    for (int ni = 0; ni < random_FSC_size; ni++)
    {
        // init empty node
        FSCNode new_node;
        this->FscNodes.push_back(new_node);
        // cout << "- ni:" << ni << "-" << endl;
        // Sample an action for each node
        double random_p_ai = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
        double temp_p_ai = 0;

        // cout << "random_p_ai:" << random_p_ai << endl;

        for (int ai = 0; ai < ActionsSize; ai++)
        {
            int Index = ni * ActionsSize + ai;
            temp_p_ai += matrix_nodes_actions_pb_dist[Index];

            // cout << temp_p_ai << endl;

            if (temp_p_ai >= random_p_ai)
            {
                // cout << "found!"
                //      << "ni:" << ni << ",ai:" << ai << endl;
                this->FscNodes[ni].best_aI = ai;
                break;
            }
        }

        // Sample a transition for each observation

        // cout << "random_p_trans:" << random_p_ai << endl;

        for (int oi = 0; oi < SizeObs; oi++)
        {
            double temp_p_trans_max = 0;
            for (int n_nexti = 0; n_nexti < random_FSC_size; n_nexti++)
            {
                int Index = ni * SizeObs * random_FSC_size + oi * random_FSC_size + n_nexti;
                temp_p_trans_max += matrix_nodes_transition_pb_dist[Index];
            }

            double random_p_trans = RandT<double>(0.0, temp_p_trans_max);
            double temp_p_trans = 0;

            for (int n_nexti = 0; n_nexti < random_FSC_size; n_nexti++)
            {
                int Index = ni * SizeObs * random_FSC_size + oi * random_FSC_size + n_nexti;
                temp_p_trans += matrix_nodes_transition_pb_dist[Index];

                // cout << temp_p_trans << endl;

                if (temp_p_trans >= random_p_trans)
                {
                    // cout << "found!"
                    //      << "ni:" << ni << ",oi:" << oi << ", n_nexti:" << n_nexti << endl;
                    this->eta_fsc[ni][oi][n_nexti] = 1;
                    break;
                }
            }
        }
    }

    cout << "random fsc complete" << endl;
};

// used for random a FSC
FSC::FSC(int MaxNodeSize, int ActionsSize, int ObsSize)
{
    // type 0 for Momdp formalization, 1 for "init node" method
    // this->AgentName = "RandomAgentFSC";
    this->size_local_Obs = ObsSize;
    this->size_local_A = ActionsSize;

    // At least we need to have one start node
    int random_FSC_size = Randmod(MaxNodeSize) + 1; //
    if (random_FSC_size > MaxNodeSize)
    {
        // Make sure the FSC nodes number (not include the preliminary node) < MaxNodesSize
        random_FSC_size = MaxNodeSize;
    }
    this->size_FSC = random_FSC_size;
    this->eta_fsc.resize(random_FSC_size, vector<vector<double>>(size_local_Obs, vector<double>(random_FSC_size, 0))); // resize the eta

    // vector<int> Nodes_Init(this->SizeNodes);
    // this->Nodes = Nodes_Init;

    int element_size_matrix_nodes_actions_pb_dist = random_FSC_size * ActionsSize;
    vector<double> matrix_nodes_actions_pb_dist(element_size_matrix_nodes_actions_pb_dist, (1.0 / element_size_matrix_nodes_actions_pb_dist));

    int element_size_matrix_nodes_transition_pb_dist = random_FSC_size * ObsSize * random_FSC_size;

    vector<double> matrix_nodes_transition_pb_dist(element_size_matrix_nodes_transition_pb_dist, (1.0 / element_size_matrix_nodes_transition_pb_dist));
    GenerateFSCwithParameters(random_FSC_size, ActionsSize, ObsSize, matrix_nodes_actions_pb_dist, matrix_nodes_transition_pb_dist);
};
