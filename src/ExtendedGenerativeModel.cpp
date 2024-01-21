#include "../Include/ExtendedGenerativeModel.h"

ExtendedGenerativeModel::ExtendedGenerativeModel(SimInterface *sim_model_decpomdp, vector<vector<vector<double>>> &eta_fsc,
                                                 vector<FSCNode> &FscNodes, int fsc_agentI, int optimizing_agentI)
{

    // ---- clear previous values ----
    this->eta_fsc.clear();
    this->FscNodes.clear();
    this->_m_ExtendedStateIndicies.clear();
    this->_m_IndiciesToExtendedStateIndex.clear();
    // --------------------------------

    this->sim_model_decpomdp = sim_model_decpomdp;
    this->FscNodes = FscNodes;
    this->eta_fsc = eta_fsc;
    this->size_FSC = this->FscNodes.size();
    this->fsc_agent = fsc_agentI;
    this->optimizing_agent = optimizing_agentI;
}

// ------- obligate functions from sim interface ----------
tuple<int, int, double, bool> ExtendedGenerativeModel::Step(int sI, int aI) // sI_next, oI, Reward, Done
{
    bool done = false;

    // be careful here, sI is the extended state index, aI is the optimizing agent's action index
    vector<int> e = this->_m_ExtendedStateIndicies[sI];
    int decpomdp_sI = e[0];
    int nI = e[1];
    // int oI = e[2];
    int aI_nI = this->FscNodes[nI].best_aI;

    vector<int> joint_action(2);
    joint_action[fsc_agent] = aI_nI;
    joint_action[optimizing_agent] = aI;
    int JaI = this->sim_model_decpomdp->IndividualToJointActionIndex(joint_action);
    tuple<int, int, double, bool> res_step = this->sim_model_decpomdp->Step(decpomdp_sI, JaI);

    int decpomdp_s_newI = get<0>(res_step);
    int Jo_newI = get<1>(res_step);
    double reward = get<2>(res_step);

    vector<int> obs_indices = this->sim_model_decpomdp->JointToIndividualObsIndices(Jo_newI);
    int oI_fsc_agent = obs_indices[fsc_agent];
    int oI_optimizing_agent = obs_indices[optimizing_agent];

    int nI_next = this->GetNextNI(nI, oI_fsc_agent, this->size_FSC, this->eta_fsc);

    vector<int> e_next = {decpomdp_s_newI, nI_next, oI_optimizing_agent};

    // check if e_next exist
    int e_newI = GiveExtendedStateIndex(e_next);

    // Build the result
    tuple<int, int, double, bool> res(e_newI, oI_optimizing_agent, reward, done);

    return res;
}

int ExtendedGenerativeModel::SampleStartState()
{
    int sI = this->sim_model_decpomdp->SampleStartState();
    int nI = 0;
    int oI = -1; // empty observation init
    vector<int> e_init = {sI, nI, oI};

    int eI_init = GiveExtendedStateIndex(e_init);

    return eI_init;
}

int ExtendedGenerativeModel::GetSizeOfObs() const
{
    return this->sim_model_decpomdp->GetSizeOfLocalObs(optimizing_agent);
}
int ExtendedGenerativeModel::GetSizeOfA() const
{
    return this->sim_model_decpomdp->GetSizeOfLocalA(optimizing_agent);
}

int ExtendedGenerativeModel::GetNextNI(int nI, int oI, int size_optimizing_fsc, vector<vector<vector<double>>> &Eta_fsc)
{
    double random_p = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
    double sum_p = 0.0;
    for (int n_newI = 0; n_newI < size_optimizing_fsc; n_newI++)
    {
        double pr_trans = Eta_fsc[nI][oI][n_newI];
        sum_p += pr_trans;
        // cout << "nI: " << nI << ", oI: " << oI << ", n_newI:" << n_newI << ", pb:" << pr_trans << endl;
        if (sum_p >= random_p)
        {
            return n_newI;
        }
    }

    // self node 
    return nI;
    // cout << "Cannot reach to next node!" << endl;
    // cout << "nI: " << nI << ", oI: " << oI << endl;

    // throw "";
    // return -1;
}

double ExtendedGenerativeModel::PolicyEvaluation(vector<vector<vector<double>>> &eta_fsc_optimizing_agent,
                                                 vector<FSCNode> &FscNodes_optimizing_agent)
{
    double sum_accumlated_rewards = 0.0;
    int restart_evaluation = 1e6;
    double discount = this->sim_model_decpomdp->GetDiscount();
    double epsilon = 0.0001;

    int size_optimizing_fsc = FscNodes_optimizing_agent.size();
    double average_acc_rewards = 0;

    tuple<int, int, double, bool> res_step;
    int i = 0;
    // for (int i = 0; i < restart_evaluation; i++)
    while (i < restart_evaluation)
    {
        i += 1;
        int depth = 0;
        int sampled_eI_start = this->SampleStartState();
        int eI = sampled_eI_start;
        int nI_optimizing_agent = 0;
        int oI = -1;
        double total_discount = pow(discount, depth);
        double temp_res = 0;
        while (total_discount > epsilon)
        {
            int aI = FscNodes_optimizing_agent[nI_optimizing_agent].best_aI;
            res_step = this->Step(eI, aI);
            double reward = get<2>(res_step);
            temp_res += reward * total_discount;
            total_discount *= discount;
            eI = get<0>(res_step);
            oI = get<1>(res_step);
            nI_optimizing_agent = this->GetNextNI(nI_optimizing_agent, oI, size_optimizing_fsc, eta_fsc_optimizing_agent);
            depth += 1;
        }
        sum_accumlated_rewards += temp_res;
        average_acc_rewards = sum_accumlated_rewards / i;
    }

    return average_acc_rewards;
}

int ExtendedGenerativeModel::GiveExtendedStateIndex(vector<int> &e)
{
    // check if e_next exist
    int eI = -1;
    // if key absent
    if (this->_m_IndiciesToExtendedStateIndex.find(e) == this->_m_IndiciesToExtendedStateIndex.end())
    {
        this->_m_ExtendedStateIndicies.push_back(e);
        eI = _m_ExtendedStateIndicies.size() - 1;
        this->_m_IndiciesToExtendedStateIndex[e] = eI;
    }
    else
    {
        eI = _m_IndiciesToExtendedStateIndex[e];
    }

    return eI;
}

double ExtendedGenerativeModel::GetDiscount() const
{
    return this->sim_model_decpomdp->GetDiscount();
}

int ExtendedGenerativeModel::GetNbAgent() const
{
    return this->sim_model_decpomdp->GetNbAgent();
}
