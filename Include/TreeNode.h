/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _TREENODE_H_
#define _TREENODE_H_

#include <iostream>
#include <fstream>
#include <string>
#include "BeliefParticles.h"

class TreeNode
{
private:
      TreeNode *ParentNode_ = nullptr;
      map<int, map<int, TreeNode *>> ChildNodes_; // aI, oI -> n_new

      map<int, int> action_counts;
      map<int, double> all_action_Q;
      map<int, double> all_action_R; // compute this value when doing a expansion
      double value_ = 0;
      int visits_ = 0;
      int best_aI;
      int all_childs_size = 0;
      int belief_id = -1;
      int depth = 0;

public:
      TreeNode(){};
      TreeNode(int depth);

      void SetBeliefID(int label);
      int GetBeliefID();

      void AddParentNode(TreeNode *ParentNode);
      void AddChildNode(int aI, int oI, TreeNode *ChildNode);

      map<int, map<int, TreeNode *>> *GetChildNodes();
      void SetValue(double value);
      void AddVisit();
      void AddActionCount(int aI);
      int GetActionCount(int aI) const;
      double GetValue();

      void SetBestAction(int aI);
      int GetBestAction();
      TreeNode *GetParentNode();
      TreeNode *GetChildNode(int aI, int oI);
      double GetChildNodeValue(int aI, int oI);
      bool CheckChildNodeExist(int aI, int oI);

      bool isRoot();
      bool isLeaf();

      int GetVisitNumber();
      int GetAllChildsSize();
      void SetAllChildsSize(int childs_size);
      void AddAllChildsSize(int childs_size);

      void SetActionR(int aI, double R);
      double GetActionR(int aI);

      void SetActionQ(int aI, double Q);
      double GetActionQ(int aI) const;
      map<int, double> GetAllActionQ();

      int GetDepth();
      void AbandonChilds();

      ~TreeNode();
};

#endif