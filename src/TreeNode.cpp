#include "../Include/TreeNode.h"

TreeNode::TreeNode(int depth)
{
      this->depth = depth;
}

void TreeNode::SetBeliefID(int label)
{
      this->belief_id = label;
}
int TreeNode::GetBeliefID()
{
      return this->belief_id;
}

void TreeNode::AddParentNode(TreeNode *ParentNode)
{
      this->ParentNode_ = ParentNode;
}
void TreeNode::AddChildNode(int aI, int oI, TreeNode *ChildNode)
{
      // pair<int, int> t(aI, oI);
      this->ChildNodes_[aI][oI] = ChildNode;
}

map<int, map<int, TreeNode *>> *TreeNode::GetChildNodes()
{
      return &this->ChildNodes_;
}
double TreeNode::GetValue()
{
      return this->value_;
}

void TreeNode::SetValue(double value)
{
      this->value_ = value;
}

double TreeNode::GetChildNodeValue(int aI, int oI)
{
      if (this->ChildNodes_.size() == 0)
      {
            cout << "error, no childs!" << endl;
            throw "";
      }
      else
      {
            TreeNode *child = this->ChildNodes_[aI][oI];
            double v = child->GetValue();
            return v;
      }
}

TreeNode *TreeNode::GetChildNode(int aI, int oI)
{
      return this->ChildNodes_[aI][oI];
}

bool TreeNode::CheckChildNodeExist(int aI, int oI)
{
      return this->ChildNodes_[aI].count(oI);
}

bool TreeNode::isRoot()
{
      if (!this->ParentNode_)
      {
            return true;
      }
      else
      {
            return false;
      }
}

bool TreeNode::isLeaf()
{
      if (!this->ChildNodes_.size())
      {
            return true;
      }
      else
      {
            return false;
      }
}

TreeNode *TreeNode::GetParentNode()
{
      return this->ParentNode_;
}

void TreeNode::SetBestAction(int aI)
{
      this->best_aI = aI;
};
int TreeNode::GetBestAction()
{
      return this->best_aI;
};

int TreeNode::GetVisitNumber()
{
      return this->visits_;
}

void TreeNode::AddVisit()
{
      this->visits_ += 1;
}

void TreeNode::AddActionCount(int aI)
{
      this->action_counts[aI] += 1;
}

int TreeNode::GetActionCount(int aI) const
{
      if (this->action_counts.count(aI))
      {
            return this->action_counts.find(aI)->second;
      }
      else
      {
            return 0;
      }
}

// delete all child
TreeNode::~TreeNode()
{
      map<int, map<int, TreeNode *>>::iterator it; // aI, oI -> n_new
      for (it = ChildNodes_.begin(); it != ChildNodes_.end(); it++)
      {
            map<int, TreeNode *>::iterator it_node;
            for (it_node = it->second.begin(); it_node != it->second.end(); it_node++)
            {
                  delete it_node->second;
            }
      }
};

int TreeNode::GetAllChildsSize()
{
      return this->all_childs_size;
}
void TreeNode::SetAllChildsSize(int childs_size)
{
      this->all_childs_size = childs_size;
}

void TreeNode::SetActionR(int aI, double R)
{
      this->all_action_R[aI] = R;
}
double TreeNode::GetActionR(int aI)
{
      return this->all_action_R[aI];
}

void TreeNode::SetActionQ(int aI, double Q)
{
      this->all_action_Q[aI] = Q;
}
double TreeNode::GetActionQ(int aI) const
{
      if (this->all_action_Q.count(aI))
      {
            return this->all_action_Q.find(aI)->second;
      }
      else
      {
            return 0.0;
      }
}

void TreeNode::AddAllChildsSize(int childs_size)
{
      this->all_childs_size += childs_size;
}

void TreeNode::AbandonChilds()
{
      this->ChildNodes_.clear();
}

map<int, double> TreeNode::GetAllActionQ()
{
      return this->all_action_Q;
}

int TreeNode::GetDepth()
{
      return this->depth;
}
