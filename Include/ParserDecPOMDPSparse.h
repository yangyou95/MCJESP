/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 * 
 */
#ifndef _PARSERDECPOMDPSPARSE_H_
#define _PARSERDECPOMDPSPARSE_H_ 1

#include "DecPomdpInterface.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <map>

using namespace std;

class ParsedDecPOMDPSparse : public DecPomdpInterface
{
private:
    vector<string> States;
    vector<vector<string>> Actions;
    vector<vector<string>> Observations;
    int AgentsNb;
    int S_size;
    vector<int> SizeActions;
    vector<int> SizeObservations;
    int JointA_size;
    int JointObs_size;
    vector<double> b0;
    map<int, double> b0_sparse;

    void BuildAllCombination(map<vector<int>, int> &IndToJoint_map, map<int, vector<int>> &JointToInd_map, vector<vector<string>> &input_space, vector<int> indicies, int depth);

    // transition function as A -> S -> P(S)
    vector<vector<map<int, double>>> TransFuncVecs;

    // observation as A -> S' -> O -> proba
    vector<vector<map<int, double>>> ObsFuncVecs;

    // reward function as A -> S -> reward
    vector<vector<double>> RewardFuncVecs;

    map<vector<int>, int> m_IndividualToJointActionIndex;
    map<vector<int>, int> m_IndividualToJointObsIndex;
    map<int, vector<int>> m_JointToIndividualActionsIndices;
    map<int, vector<int>> m_JointToIndividualObsIndices;
    double discount;

public:
    ParsedDecPOMDPSparse(const string filename);
    ~ParsedDecPOMDPSparse(){};
    virtual int GetNbAgents() const { return this->AgentsNb; };
    double GetDiscount() const;
    int GetSizeOfS() const;
    int GetSizeOfJointA() const;
    int GetSizeOfJointObs() const;
    int GetSizeOfA(int agentI) const { return this->Actions[agentI].size(); };
    int GetSizeOfObs(int agentI) const { return this->Observations[agentI].size(); };
    vector<int> JointToIndividualActionsIndices(int JI) const;
    vector<int> JointToIndividualObsIndices(int JI) const;
    int IndividualToJointActionIndex(vector<int> &Indicies) const;
    int IndividualToJointObsIndex(vector<int> &Indicies) const;
    const vector<vector<string>> &GetAllActionsVecs() const;
    const vector<vector<string>> &GetAllObservationsVecs() const;
    const vector<string> &GetActionVec(int agentI) const;
    const vector<string> &GetObservationVec(int agentI) const;
    string GetActionName(int agentI, int aI) const;
    string GetObservationName(int agentI, int oI) const;
    const vector<string> &GetAllStates() const { return this->States; };
    // vector<double> GetInitBelief();
    double TransFunc(int sI, int JaI, int s_newI) const;
    double ObsFunc(int JoI, int s_newI, int JaI) const;
    double Reward(int sI, int JaI) const;
    void ResetTrans(int sI, int JaI, int s_newI, double pr);
    // sparse representation, return a prob dist
    const map<int, double> *GetTransProbDist(int sI, int JaI) const;
    const map<int, double> *GetObsFuncProbDist(int s_newI, int JaI) const;
    const map<int, double> *GetInitialBeliefSparse() const;
};

#endif