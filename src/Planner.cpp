#include "../Include/Planner.h"

PomcpPlanner::PomcpPlanner(SimInterface *sim, double discount)
{
      this->simulator = sim;
      this->size_A = sim->GetSizeOfA();
      this->discount = discount;
}

PomcpPlanner::~PomcpPlanner()
{
      // delete this->rootnode;
}

double PomcpPlanner::Rollout(int sampled_sI, int node_depth)
{
      double sum_accumlated_rewards = 0.0;
      tuple<int, int, double, bool> res_step;
      for (int i = 0; i < this->nb_restarts_simulation; i++)
      {
            int sI = sampled_sI;
            double total_discount = pow(this->discount, node_depth);
            double temp_res = 0;
            while (total_discount > epsilon || node_depth < max_depth)
            {
                  int aI = SampleOneNumber(0, this->size_A);
                  // while (aI < 0 || aI >= this->size_A)
                  // {
                  //       aI = RandT<int>(0, this->size_A);
                  // }
                  res_step = this->simulator->Step(sI, aI);
                  double reward = get<2>(res_step);
                  temp_res += reward * total_discount;
                  total_discount *= this->discount;
                  sI = get<0>(res_step);
                  node_depth += 1;
            }
            sum_accumlated_rewards += temp_res;
      }
      double res = sum_accumlated_rewards / nb_restarts_simulation;
      return res;
}

int PomcpPlanner::Search(BeliefParticles b)
{

      time_t PlanStartTime, PlanEndTime;
      PlanStartTime = time(NULL);
      PlanEndTime = time(NULL);
      double PlanSpentTime = (double)(PlanEndTime - PlanStartTime);
      // TreeNode *new_node = new TreeNode(b, 0);
      TreeNode *new_node = new TreeNode(0);

      this->rootnode = new_node;
      while (PlanSpentTime < this->timeout)
      {
            int sampled_sI = b.SampleOneState();
            this->Simulate(sampled_sI, this->rootnode, 0);
            PlanEndTime = time(NULL);
            PlanSpentTime = (double)(PlanEndTime - PlanStartTime);
      }
      int best_aI = ChooseArgMaxKey(this->rootnode->GetAllActionQ());
      this->Root_best_action_possible_obs.clear();
      int size_obs = this->simulator->GetSizeOfObs();
      for (int oI = 0; oI < size_obs; oI++)
      {
            this->Root_best_action_possible_obs[oI] = this->rootnode->CheckChildNodeExist(best_aI, oI);
      }

      delete this->rootnode;

      return best_aI;
}

double PomcpPlanner::Simulate(int sampled_sI, TreeNode *node, int depth)
{
      double esti_V = 0;
      node->AddVisit();
      double total_discount = pow(this->discount, depth);
      if (total_discount < epsilon || depth == max_depth)
      {
            return 0;
      }

      int aI = UcbActionSelection(node);
      if (aI < 0 || aI >= this->size_A)
      {
            cout << "simulate" << endl;
            cout << "aI: " << aI << endl;
            throw "";
      }

      tuple<int, int, double, bool> res_step = this->simulator->Step(sampled_sI, aI);
      double reward = get<2>(res_step);
      int next_sI = get<0>(res_step);
      int oI = get<1>(res_step);

      // check if have child node with this new history "hao"
      if (node->CheckChildNodeExist(aI, oI))
      {
            TreeNode *child_node = node->GetChildNode(aI, oI);
            esti_V = reward + this->discount * Simulate(next_sI, child_node, depth + 1);
      }
      else
      {
            CreateNewNode(node, aI, oI);
            esti_V = reward + this->discount * Rollout(next_sI, depth + 1);
      }

      node->AddActionCount(aI);
      int aI_count = node->GetActionCount(aI);
      double current_Qba = node->GetActionQ(aI);
      double updated_Qba = current_Qba + (esti_V - current_Qba) / aI_count;
      node->SetActionQ(aI, updated_Qba);
      node->SetValue(esti_V);
      return esti_V;
}

void PomcpPlanner::Init(double c, int pomcp_nb_rollout, double timeout, double threshold, int max_depth)
{
      // this->b0 = b0;
      this->c = c;
      this->timeout = timeout;
      this->epsilon = threshold;
      this->max_depth = max_depth;
      this->nb_restarts_simulation = pomcp_nb_rollout;
}

TreeNode *PomcpPlanner::CreateNewNode(TreeNode *parent_node, int aI, int oI)
{

      int parent_depth = parent_node->GetDepth();
      TreeNode *child_node = new TreeNode(parent_depth + 1);
      child_node->AddParentNode(parent_node);
      parent_node->AddChildNode(aI, oI, child_node);
      return child_node;
}

int PomcpPlanner::UcbActionSelection(TreeNode *node) const
{
      int nb_node_visit = node->GetVisitNumber();
      double Max_value = -DBL_MAX;
      int selected_aI = -1;
      for (int aI = 0; aI < size_A; aI++)
      {
            double ratio_visit = 0;
            int nb_aI_visit = node->GetActionCount(aI);
            if (nb_aI_visit == 0)
            {
                  ratio_visit = DBL_MAX;
            }
            else
            {
                  ratio_visit = nb_node_visit / nb_aI_visit;
            }

            double value = node->GetActionQ(aI) + this->c * sqrt(ratio_visit);

            if (value > Max_value)
            {
                  Max_value = value;
                  selected_aI = aI;
            }
      }

      return selected_aI;
}

map<int, bool> PomcpPlanner::GiveRootActionPossibleObservation()
{

      return this->Root_best_action_possible_obs;
}