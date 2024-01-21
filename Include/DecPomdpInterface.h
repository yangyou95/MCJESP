/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 *
 */

#ifndef _DECPOMDPINTERFACE_H_
#define _DECPOMDPINTERFACE_H_

#include <vector>
#include <string>
#include <map>

using namespace std;

class DecPomdpInterface
{
private:
    /* data */
public:
    DecPomdpInterface(){};
    virtual ~DecPomdpInterface(){};
    virtual int GetNbAgents() const = 0;
    virtual double GetDiscount() const = 0;
    virtual int GetSizeOfS() const = 0;
    virtual int GetSizeOfJointA() const = 0;
    virtual int GetSizeOfJointObs() const = 0;
    virtual int GetSizeOfA(int agentI) const = 0;
    virtual int GetSizeOfObs(int agentI) const = 0;
    virtual const vector<vector<string>> &GetAllActionsVecs() const = 0;
    virtual const vector<vector<string>> &GetAllObservationsVecs() const = 0;
    virtual const vector<string> &GetActionVec(int agentI) const = 0;
    virtual const vector<string> &GetObservationVec(int agentI) const = 0;
    virtual string GetActionName(int agentI, int aI) const = 0;
    virtual string GetObservationName(int agentI, int oI) const = 0;
    virtual const vector<string> &GetAllStates() const = 0;
    virtual vector<int> JointToIndividualActionsIndices(int JI) const = 0;
    virtual vector<int> JointToIndividualObsIndices(int JI) const = 0;
    virtual int IndividualToJointActionIndex(vector<int> &Indicies) const = 0;
    virtual int IndividualToJointObsIndex(vector<int> &Indicies) const = 0;
    // virtual vector<double> GetInitBelief()=0;
    virtual double TransFunc(int sI, int JaI, int s_newI) const = 0;
    virtual double ObsFunc(int JoI, int s_newI, int JaI) const = 0;
    virtual double Reward(int sI, int JaI) const = 0;
    // for sparse representation
    virtual const map<int, double> *GetTransProbDist(int sI, int JaI) const
    {
        (void)(sI);
        (void)(JaI);
        return nullptr;
    };
    virtual const map<int, double> *GetObsFuncProbDist(int s_newI, int JaI) const
    {
        (void)(s_newI);
        (void)(JaI);
        return nullptr;
    };
    virtual const map<int, double> *GetInitialBeliefSparse() const
    {
        return nullptr;
    };
    virtual void ResetTrans(int sI, int JaI, int s_newI, double pr)
    {
        (void)(sI);
        (void)(pr);
        (void)(s_newI);
        (void)(JaI);
    };
};

#endif /* !_DECPOMDPINTERFACE_H_ */