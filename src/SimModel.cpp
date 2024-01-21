#include "../Include/SimModel.h"

SimModel::SimModel(PomdpInterface *PomdpModel, string VisfilePath)
{
      this->PomdpModel = PomdpModel;
      this->interactive = true;
      // Sample the init state
      // this->stateI = this->SampleStartState();
      this->LoadVisualizationFile(VisfilePath);
      this->all_states = this->PomdpModel->GetAllStates();
      this->action_size = this->PomdpModel->GetSizeOfA();
      this->all_actions_pomdp = this->PomdpModel->GetAllActions();
      this->all_observations_pomdp = this->PomdpModel->GetAllObservations();
      this->discount = this->PomdpModel->GetDiscount();
}

SimModel::SimModel(PomdpInterface *PomdpModel)
{
      this->PomdpModel = PomdpModel;
      this->all_states = this->PomdpModel->GetAllStates();
      this->action_size = this->PomdpModel->GetSizeOfA();
      this->all_actions_pomdp = this->PomdpModel->GetAllActions();
      this->all_observations_pomdp = this->PomdpModel->GetAllObservations();
      this->discount = this->PomdpModel->GetDiscount();
}

SimModel::SimModel(DecPomdpInterface *DecPomdpModel, string VisfilePath)
{
      this->DecPomdpModel = DecPomdpModel;
      this->interactive = true;
      this->LoadVisualizationFile(VisfilePath);
      this->mode = 1;
      this->action_size = this->DecPomdpModel->GetSizeOfJointA();
      this->all_states = this->DecPomdpModel->GetAllStates();
      this->all_actions_decpomdp = this->DecPomdpModel->GetAllActionsVecs();
      this->all_observations_decpomdp = this->DecPomdpModel->GetAllObservationsVecs();
      this->discount = this->DecPomdpModel->GetDiscount();
}
SimModel::SimModel(DecPomdpInterface *DecPomdpModel)
{
      this->DecPomdpModel = DecPomdpModel;
      this->mode = 1;
      this->action_size = this->DecPomdpModel->GetSizeOfJointA();
      this->all_actions_decpomdp = this->DecPomdpModel->GetAllActionsVecs();
      this->all_observations_decpomdp = this->DecPomdpModel->GetAllObservationsVecs();
      this->all_states = this->DecPomdpModel->GetAllStates();
      this->discount = this->DecPomdpModel->GetDiscount();
}

SimModel::~SimModel() {}

int SimModel::SampleStartState()
{
      const map<int, double> *b0;
      if (this->mode == 0)
      {
            b0 = this->PomdpModel->GetInitBeliefSparse();
      }

      if (this->mode == 1)
      {
            b0 = this->DecPomdpModel->GetInitialBeliefSparse();
      }

      double random_p = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
      double temp_p = 0;

      map<int, double>::const_iterator it_sI;

      for (it_sI = b0->begin(); it_sI != b0->end(); it_sI++)
      {
            temp_p += it_sI->second;
            if (temp_p >= random_p)
            {
                  return it_sI->first;
            }
      }

      // return impossibe output
      return -1;
};

int SimModel::GetObsFromState(int sI, int jaI) const
{

      double random_p = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
      // cout << "rand in GetObsFromState is: " << random_p << endl;
      double temp_p = 0;
      const map<int, double> *dist_obsI;

      if (this->mode == 0)
      {
            dist_obsI = this->PomdpModel->GetObsFuncProbDist(sI, jaI);
      }

      if (this->mode == 1)
      {
            dist_obsI = this->DecPomdpModel->GetObsFuncProbDist(sI, jaI);
      }

      map<int, double>::const_iterator it_oI;

      for (it_oI = dist_obsI->begin(); it_oI != dist_obsI->end(); it_oI++)
      {
            int joI = it_oI->first;
            double pb_obs = it_oI->second;
            if (pb_obs > 0)
            {
                  temp_p += pb_obs;
                  // if the current sum_pb > rand, then select this observation
                  if (temp_p >= random_p)
                  {
                        return joI;
                  }
            }
      }

      // return impossible output
      return -1;
};

// sI, oI, Reward， done
// tuple<int, int, bool, double, string> SimModel::Step(int current_sI, int aI) const
tuple<int, int, double, bool> SimModel::Step(int current_sI, int aI)

{

      // Need a check Done part
      bool done = false;
      // Currently cannot check done for the model in Cassendra .pomdp format
      int sI_next = -1;

      double reward = 0;

      if (this->mode == 0)
      {
            reward = this->PomdpModel->Reward(current_sI, aI);
      }

      if (this->mode == 1)
      {
            // cout << "current sI: " << current_sI << endl;
            // cout << "JaI: " << aI << endl;
            reward = this->DecPomdpModel->Reward(current_sI, aI);
      }

      // cout << "something" << endl;

      double random_p = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);
      // cout << "rand in Step is: " << random_p << endl;
      double temp_p = 0;

      if (!done)
      {
            const map<int, double> *dist_trans;
            if (this->mode == 0)
            {
                  dist_trans = this->PomdpModel->GetTransProbDist(current_sI, aI);
            }

            if (this->mode == 1)
            {
                  dist_trans = this->DecPomdpModel->GetTransProbDist(current_sI, aI);
            }

            map<int, double>::const_iterator it_trans;
            for (it_trans = dist_trans->begin(); it_trans != dist_trans->end(); it_trans++)
            {
                  int s_newI = it_trans->first;
                  double pb_snew = it_trans->second;
                  if (pb_snew > 0)
                  {
                        temp_p += pb_snew;
                        // if the current sum_pb > rand, then select this new state
                        if (temp_p >= random_p)
                        {
                              // this->stateI = s_newI;
                              sI_next = s_newI;
                              break;
                        }
                  }
            }
      }

      int obsI = this->GetObsFromState(sI_next, aI);

      // info store the current state string
      // string info = this->PomdpModel->GetAllStates()[this->stateI];

      // Build the result
      tuple<int, int, double, bool> res(sI_next, obsI, reward, done);

      return res;
};

// void SimModel::Reset()
// {
//       this->stateI = this->SampleStartState();
// }

void addNewLines(std::string *text)
{
      for (size_t i = 0; i < text->length(); i++)
      {
            if ((*text)[i] == '.')
            {
                  (*text)[i] = '\n';
            }
      }
}

void SimModel::VisualizationState(int sI) const
{
      string res = this->visualization_states[sI];
      addNewLines(&res);
      cout << res << endl;
};

void SimModel::LoadVisualizationFile(string fileName)
{
      // Open the File
      std::ifstream in(fileName.c_str());
      // Check if object is valid
      if (!in)
      {
            std::cerr << "Cannot open the File : " << fileName << std::endl;
      }
      std::string str;
      // Read the next line from File until it reaches the end.
      while (std::getline(in, str))
      {
            // Line contains string of length > 0 then save it in vector
            if (str.size() > 0)
                  this->visualization_states.push_back(str);
      }
}

int SimModel::SelectActions() const
{
      int aI;
      if (this->mode == 0)
      {
            cout << "Please enter an action index for the player in this POMDP " << endl;
            for (int i = 0; i < this->action_size; i++)
            {
                  cout << i << ":" << this->all_actions_pomdp[i] << " ";
            }
      }

      if (this->mode == 1)
      {
            cout << "Please enter an joint action index for all the players in this DecPOMDP " << endl;
            int Nb_agent = this->DecPomdpModel->GetNbAgents();
            for (int i = 0; i < this->action_size; i++)
            {
                  cout << "JAI " << i << ": ";
                  vector<int> action_indices = this->DecPomdpModel->JointToIndividualActionsIndices(i);
                  for (int agent_I = 0; agent_I < Nb_agent; agent_I++)
                  {
                        cout << "agent " << agent_I << ": " << all_actions_decpomdp[agent_I][action_indices[agent_I]] << ", ";
                  }
                  cout << endl;
            }
      }

      cout << endl;
      cin >> aI;
      return aI;
};

double SimModel::GetDiscount() const
{
      return this->DecPomdpModel->GetDiscount();
}

int SimModel::GetNbAgent() const
{
      return this->DecPomdpModel->GetNbAgents();
}

void SimModel::SimulateNsteps(int start_sI, int N)
{
      double sum_rewards = 0;
      int current_sI = start_sI;
      for (int step = 0; step < N; step++)
      {
            // Print current information
            cout << " --------  Current step is: " << step << "  ---------" << endl;
            // cout << "Current state is: " << this->PomdpModel->GetAllStates()[current_sI] << endl;
            cout << "Current state is: " << this->all_states[current_sI] << endl;

            // visualization!
            // need different visulization for interactive interface
            if (this->visualization_states.size() != 0)
            {
                  this->VisualizationState(current_sI);
            }

            // Select an action
            int aI = this->SelectActions();
            if (aI >= this->action_size)
            {
                  cout << "Error input aI" << endl;
                  throw "";
            }

            // cout << "Selected action is: " << this->PomdpModel->GetAllActions()[aI] << " for agent " << endl;
            cout << "Selected action is: " << aI << endl;

            // Step
            tuple<int, int, double, bool> temp_res = this->Step(current_sI, aI); // sI_next, oI, Reward, Done

            // visulization for interactive interface should be done here
            // Get new obs and print
            int ObsI = get<1>(temp_res);

            if (mode == 0)
            {
                  cout << "Recieved obs is: " << this->all_observations_pomdp[ObsI] << endl;
            }

            if (mode == 1)
            {
                  int Nb_agent = this->DecPomdpModel->GetNbAgents();
                  vector<int> obs_indices = this->DecPomdpModel->JointToIndividualObsIndices(ObsI);
                  for (int agent_I = 0; agent_I < Nb_agent; agent_I++)
                  {
                        cout << "agent " << agent_I << ": " << all_observations_decpomdp[agent_I][obs_indices[agent_I]] << ", ";
                  }
                  cout << endl;
            }

            // Get reward and print
            double reward = get<2>(temp_res);
            cout << "Recieved instant reward: " << reward << endl;
            sum_rewards += pow(this->discount, step) * reward;
            current_sI = get<0>(temp_res);

            cout << endl;
      }

      cout << "--------- Overall Information ----------" << endl;
      cout << "Final state after " << N << " steps is: " << this->all_states[current_sI] << endl;
      cout << "Accumulated rewards: " << sum_rewards << endl;
}

vector<int> SimModel::JointToIndividualObsIndices(int JoI) const
{
      return this->DecPomdpModel->JointToIndividualObsIndices(JoI);
}

vector<int> SimModel::JointToIndividualActionIndices(int JaI) const
{
      return this->DecPomdpModel->JointToIndividualActionsIndices(JaI);
}

int SimModel::IndividualToJointActionIndex(vector<int> &action_indices) const
{
      return this->DecPomdpModel->IndividualToJointActionIndex(action_indices);
}

int SimModel::GetSizeOfLocalObs(int agentI) const
{
      return this->DecPomdpModel->GetSizeOfObs(agentI);
}

int SimModel::GetSizeOfLocalA(int agentI) const
{
      return this->DecPomdpModel->GetSizeOfA(agentI);
}

int SimModel::GetSizeOfObs() const
{
      if (this->mode == 0)
      {
            return this->PomdpModel->GetSizeOfObs();
      }
      else
      {
            return this->GetSizeOfJointObs();
      }
}

int SimModel::GetSizeOfA() const
{
      if (this->mode == 0)
      {
            return this->PomdpModel->GetSizeOfA();
      }
      else
      {
            return this->DecPomdpModel->GetSizeOfJointA();
      }
}

int SimModel::GetSizeOfJointObs() const
{
      return this->DecPomdpModel->GetSizeOfJointObs();
}

// DecPomdpInterface *SimModel::GetDecPomdpModel()
// {
//       return this->DecPomdpModel;
// }

int SimModel::GiveLocalObsIndex(int JoI, int agentI) const
{
      return this->DecPomdpModel->JointToIndividualObsIndices(JoI)[agentI];
}
int SimModel::GiveLocalActionIndex(int JaI, int agentI) const
{
      return this->DecPomdpModel->JointToIndividualActionsIndices(JaI)[agentI];
}