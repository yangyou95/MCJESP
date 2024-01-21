#include "../Include/ParserDecPOMDPSparse.h"

ParsedDecPOMDPSparse::ParsedDecPOMDPSparse(const string filename)
{
    ifstream infile;
    infile.open(filename);
    if (!infile.is_open())
        cout << "open file failure" << endl;

    string temp;

    int actions_read_index = -1;
    int obs_read_index = -1;
    bool ReadActions = false;
    bool ReadObservations = false;
    bool ReadStart = false; // end at here
    // First Get agents number, discount and all the state, action and observation space
    while (getline(infile, temp) && !ReadStart)
    {
        istringstream is(temp);
        string s;
        int temp_num = 0;
        bool ReadAgents = false;
        bool ReadDiscount = false;
        bool ReadStates = false;
        while (is >> s)
        {

            if (s == "agents:")
            {
                ReadAgents = true;
            }

            else if (s == "discount:")
            {
                ReadDiscount = true;
            }
            else if (s == "states:")
            {
                ReadStates = true;
            }
            else if (s == "actions:")
            {
                ReadActions = true;
            }
            else if (s == "observations:")
            {
                ReadObservations = true;
            }
            else if (s == "start:")
            {
                ReadStart = true;
            }

            // Get Agents number， init Actions and Observations
            if (ReadAgents && temp_num == 1)
            {
                this->AgentsNb = stoi(s);
                // vector<vector<string>> A(this->AgentsNb);
                // vector<vector<string>> O(this->AgentsNb);
                this->Actions.resize(this->AgentsNb);
                this->Observations.resize(this->AgentsNb);
            }
            // Get discount factor
            if (ReadDiscount && temp_num == 1)
            {
                this->discount = stod(s);
            }
            // Get all the States
            if (ReadStates && temp_num > 0)
            {
                this->States.push_back(s);
            }
            // Get all actions
            // if (ReadActions && temp_num >0)
            if (ReadActions && actions_read_index < this->AgentsNb && actions_read_index > -1)
            {

                this->Actions[actions_read_index].push_back(s);
            }
            // Get all observations
            // if (ReadObservations && temp_num >0)
            if (ReadObservations && obs_read_index < this->AgentsNb && obs_read_index > -1)
            {
                this->Observations[obs_read_index].push_back(s);
            }
            // Get intial belief
            if (ReadStart && temp_num > 0)
            {
                double pb = stod(s);
                b0.push_back(pb);
                if (pb > 0)
                {
                    // b0_sparse.InsertValue(b0.size()-1,pb);
                    b0_sparse[b0.size() - 1] = pb;
                }
            }
            temp_num += 1;
        }

        if (ReadActions)
        {
            actions_read_index += 1;
        }
        if (ReadObservations)
        {
            obs_read_index += 1;
        }
    }
    infile.close();

    int temp_JA_size = 1;
    int temp_JO_size = 1;

    for (int i = 0; i < this->AgentsNb; i++)
    {
        temp_JA_size *= this->Actions[i].size();
        // cout << "Action vector size:" << this->Actions[i].size() << endl;
        temp_JO_size *= this->Observations[i].size();
    }

    this->JointA_size = temp_JA_size;
    this->JointObs_size = temp_JO_size;
    this->S_size = this->States.size();
    // b0_sparse.SetSize(this->S_size);

    cout << "JointA Size:" << this->JointA_size << endl;
    cout << "JointObs Size:" << this->JointObs_size << endl;

    // Process the maps (J to I and I to J)
    BuildAllCombination(this->m_IndividualToJointActionIndex, this->m_JointToIndividualActionsIndices, this->Actions, {}, 0);
    BuildAllCombination(this->m_IndividualToJointObsIndex, this->m_JointToIndividualObsIndices, this->Observations, {}, 0);

    // vector< vector< vector<double> > > T(this->JointA_size, vector<vector<double> >(States.size(), vector<double>(States.size()) ));
    // vector< vector< vector<double> > > O(this->JointA_size, vector<vector<double> >(States.size(), vector<double>(this->JointObs_size) ));
    // vector< vector<double> > R(this->JointA_size, vector<double>(States.size()));
    this->TransFuncVecs.resize(JointA_size, vector<map<int, double>>(States.size()));
    this->ObsFuncVecs.resize(JointA_size, vector<map<int, double>>(States.size()));
    this->RewardFuncVecs.resize(JointA_size, vector<double>(States.size()));

    infile.open(filename);
    // Get T,O and R
    while (getline(infile, temp))
    {
        // cout << temp << endl;
        istringstream is(temp);
        string s;
        int temp_num = 0;
        bool buildTrans = false;
        bool buildObs = false;
        bool buildReward = false;
        int aI = 0;
        int sI = 0;
        int oI = 0;
        int snewI = 0;
        double pb = 0;
        vector<int> act_indicies(this->AgentsNb);
        vector<int> obs_indicies(this->AgentsNb);
        // cout << endl;
        while (is >> s)
        {

            // cout << s <<" " ;

            // Get Transition Function
            if (s == "T:")
            {
                buildTrans = true;
            }
            else if (s == "O:")
            {
                buildObs = true;
            }
            else if (s == "R:")
            {
                buildReward = true;
            }

            if (0 < temp_num && temp_num < 1 + this->AgentsNb)
            {
                if (buildTrans || buildObs || buildReward)
                {
                    act_indicies[temp_num - 1] = stoi(s);
                }
            }
            else if (temp_num == 4)
            {
                if (buildTrans || buildObs || buildReward)
                {
                    sI = stoi(s);
                }
            }
            else if (5 < temp_num && temp_num < 6 + this->AgentsNb)
            {
                if (buildTrans)
                {
                    if (temp_num == 6)
                    {
                        snewI = stoi(s);
                    }
                }
                if (buildObs)
                {
                    obs_indicies[temp_num - 6] = stoi(s);
                }
            }
            else if (temp_num == 8)
            {
                // build T now
                if (buildTrans)
                {
                    pb = stod(s);
                    aI = this->IndividualToJointActionIndex(act_indicies);
                    // int Index = aI*S_size*S_size + sI*S_size + snewI;
                    // TransFuncVecs[Index] = pb;
                    TransFuncVecs[aI][sI].insert(std::make_pair(snewI, pb));
                }
            }
            else if (temp_num == 9)
            {
                if (buildObs)
                {
                    pb = stod(s);
                    aI = this->IndividualToJointActionIndex(act_indicies);
                    oI = this->IndividualToJointObsIndex(obs_indicies);

                    // int Index = aI*S_size*JointObs_size + sI*JointObs_size + oI;
                    // ObsFuncVecs[Index] = pb;
                    ObsFuncVecs[aI][sI].insert(std::make_pair(oI, pb));
                }
            }
            else if (temp_num == 10)
            {
                if (buildReward)
                {
                    pb = stod(s);
                    aI = this->IndividualToJointActionIndex(act_indicies);
                    // int Index = aI*S_size + sI;
                    // RewardFuncVecs[Index] = pb;
                    RewardFuncVecs[aI][sI] = pb;
                }
            }

            temp_num += 1;
        }
    }

    infile.close();

    // this->TransFuncVecs = T;
    // this->ObsFuncVecs = O;
    // this->RewardFuncVecs = R;
}

void ParsedDecPOMDPSparse::BuildAllCombination(map<vector<int>, int> &IndToJoint_map, map<int, vector<int>> &JointToInd_map, vector<vector<string>> &input_space, vector<int> indicies, int depth)
{
    if (depth != int(input_space.size()))
    {
        for (int i = 0; i < int(input_space[depth].size()); i++)
        {
            indicies.push_back(i);
            BuildAllCombination(IndToJoint_map, JointToInd_map, input_space, indicies, depth + 1);
            indicies.pop_back();
        }
    }
    else
    {
        int JI = 0;
        for (int i = 0; i < int(input_space.size()); i++)
        {
            int temp = 1;
            for (int j = i + 1; j < int(input_space.size()); j++)
            {
                temp *= input_space[j].size();
            }

            // cout << indicies[i]<< " " <<JI << endl;
            JI += indicies[i] * temp;
        }
        IndToJoint_map[indicies] = JI;
        JointToInd_map[JI] = indicies;
    }
}

int ParsedDecPOMDPSparse::GetSizeOfS() const { return this->S_size; };
int ParsedDecPOMDPSparse::GetSizeOfJointA() const { return this->JointA_size; };
int ParsedDecPOMDPSparse::GetSizeOfJointObs() const { return this->JointObs_size; };
vector<int> ParsedDecPOMDPSparse::JointToIndividualActionsIndices(int JI) const { return this->m_JointToIndividualActionsIndices.find(JI)->second; };
vector<int> ParsedDecPOMDPSparse::JointToIndividualObsIndices(int JI) const { return this->m_JointToIndividualObsIndices.find(JI)->second; };
int ParsedDecPOMDPSparse::IndividualToJointActionIndex(vector<int> &Indicies) const { return this->m_IndividualToJointActionIndex.find(Indicies)->second; };
int ParsedDecPOMDPSparse::IndividualToJointObsIndex(vector<int> &Indicies) const { return this->m_IndividualToJointObsIndex.find(Indicies)->second; };

double ParsedDecPOMDPSparse::TransFunc(int sI, int JaI, int s_newI) const
{
    // if key absent
    if ((this->TransFuncVecs[JaI][sI]).find(s_newI) == this->TransFuncVecs[JaI][sI].end())
    {
        // returns proba 0
        return 0.;
    }
    // key present
    else
    {
        // returns associated value
        return this->TransFuncVecs[JaI][sI].find(s_newI)->second;
    }
};

double ParsedDecPOMDPSparse::ObsFunc(int JoI, int s_newI, int JaI) const
{
    // if key absent
    if ((this->ObsFuncVecs[JaI][s_newI]).find(JoI) == this->ObsFuncVecs[JaI][s_newI].end())
    {
        // returns proba 0
        return 0.;
    }
    // key present
    else
    {
        // returns associated value
        return this->ObsFuncVecs[JaI][s_newI].find(JoI)->second;
    }
};
double ParsedDecPOMDPSparse::Reward(int sI, int JaI) const
{

    return this->RewardFuncVecs[JaI][sI];
};

const vector<vector<string>> &ParsedDecPOMDPSparse::GetAllActionsVecs() const { return this->Actions; };
const vector<vector<string>> &ParsedDecPOMDPSparse::GetAllObservationsVecs() const { return this->Observations; };

double ParsedDecPOMDPSparse::GetDiscount() const
{
    return this->discount;
};

const vector<string> &ParsedDecPOMDPSparse::GetActionVec(int agentI) const
{
    return this->Actions[agentI];
};
const vector<string> &ParsedDecPOMDPSparse::GetObservationVec(int agentI) const
{
    return this->Observations[agentI];
};
string ParsedDecPOMDPSparse::GetActionName(int agentI, int aI) const
{
    return this->Actions[agentI][aI];
};
string ParsedDecPOMDPSparse::GetObservationName(int agentI, int oI) const
{
    return this->Observations[agentI][oI];
};

/* return a prob distribution for transition function*/
const map<int, double> *ParsedDecPOMDPSparse::GetTransProbDist(int sI, int aI) const
{
    return &this->TransFuncVecs[aI][sI];
};

/* return a prob distribution for obs function*/
const map<int, double> *ParsedDecPOMDPSparse::GetObsFuncProbDist(int sI, int aI) const
{
    return &this->ObsFuncVecs[aI][sI];
};

const map<int, double> *ParsedDecPOMDPSparse::GetInitialBeliefSparse() const
{
    return &this->b0_sparse;
};

void ParsedDecPOMDPSparse::ResetTrans(int sI, int JaI, int s_newI, double pr)
{
    this->TransFuncVecs[JaI][sI][s_newI] = pr;
};