/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */

#ifndef _SIMMODEL_H_
#define _SIMMODEL_H_

#include "SimInterface.h"

using namespace std;

class SimModel : public SimInterface
{
private:
      PomdpInterface *PomdpModel;
      DecPomdpInterface *DecPomdpModel;
      bool interactive = false;
      vector<string> visualization_states;
      int mode = 0; // 0: pomdp mode (default), 1: depomdp mode
      int action_size = -1;
      double discount = 0;

      vector<string> all_states;
      vector<vector<string>> all_actions_decpomdp;
      vector<vector<string>> all_observations_decpomdp;
      vector<string> all_actions_pomdp;
      vector<string> all_observations_pomdp;

public:
      SimModel(){};
      SimModel(PomdpInterface *PomdpModel, string VisfilePath); // used for human control
      SimModel(PomdpInterface *PomdpModel);
      SimModel(DecPomdpInterface *DecPomdpModel, string VisfilePath); // used for human control
      SimModel(DecPomdpInterface *DecPomdpModel);
      ~SimModel();
      int SelectActions() const;
      void VisualizationState(int sI) const;
      void LoadVisualizationFile(string fileName);
      int GetObsFromState(int sI, int jaI) const;
      void SimulateNsteps(int start_sI, int N);

      // ------- obligate functions from sim interface ----------
      tuple<int, int, double, bool> Step(int sI, int aI); // sI_next, oI, Reward, Done
      int SampleStartState();
      int GetSizeOfObs() const;
      int GetSizeOfA() const;
      double GetDiscount() const;
      int GetNbAgent() const;

      // --------------------------------------------------------

      int IndividualToJointActionIndex(vector<int> &action_indices) const;
      vector<int> JointToIndividualObsIndices(int JoI) const;
      vector<int> JointToIndividualActionIndices(int JaI) const;
      int GetSizeOfJointObs() const;
      int GetSizeOfLocalObs(int agentI) const;
      int GetSizeOfLocalA(int agentI) const;
      int GiveLocalObsIndex(int JoI, int agentI) const;
      int GiveLocalActionIndex(int JaI, int agentI) const;

      // DecPomdpInterface *GetDecPomdpModel();
};

#endif