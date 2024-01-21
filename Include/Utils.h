/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 *
 */

#ifndef _UTILS_H_
#define _UTILS_H_

#include <math.h>
#include <float.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <set>
#include "PomdpInterface.h"
#include "BeliefParticles.h"
#include "SimInterface.h"

template <typename T>
T RandT(T _min, T _max)
{
	T temp;
	if (_min > _max)
	{
		temp = _max;
		_max = _min;
		_min = temp;
	}
	return rand() / (double)RAND_MAX * (_max - _min) + _min;
};

BeliefParticles UpdateBeliefandProb(SimInterface &sim, BeliefParticles &b, int aI, int oI, double &out_pb);
BeliefParticles UpdateOneSideBeliefandProb(SimInterface &sim, BeliefParticles &b, int JaI, int local_oI, int local_agent_index, double &out_pb);
BeliefParticles BuildInitDist(SimInterface *sim);
void GetNextBeliefs(SimInterface &sim, BeliefParticles &b, int JaI, map<int, double> &out_pr_oba, map<int, BeliefParticles> &out_beliefs);
void GetNextBeliefsOneSidedUpdate(SimInterface &sim, BeliefParticles &b, int JaI, int local_agent_index, map<int, double> &out_pr_oba, map<int, BeliefParticles> &out_beliefs);

int ChooseArgMaxKey(const map<int, double> &m);

void PrintMap(const map<int, double> &m);

void PrintDecPomdpJointObs(DecPomdpInterface *pb, int JOI);
void PrintDecPomdpJointAction(DecPomdpInterface *pb, int JAI);

double ComputeNorm1Distance(BeliefParticles &belief1, BeliefParticles &belief2);
double ComputeNorm1Distance(map<int, double> &belief1, map<int, double> &belief2);
bool CheckBeliefConvergence(map<int, double> &previous_belief, map<int, double> &current_belief, double epsilon);

int SampleOneNumber(int min, int max);

// BeliefParticles BuildInitDist(SimInterface *sim, int nb_particles);

void RenormalizeMap(map<int, double> &m, double min_pb);

#endif /* !_UTILS_H_ */