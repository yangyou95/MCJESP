#include "../Include/Utils.h"

const int max_try = 1e6;
const double epsilon = 0.0001; // for belief

void PrintMap(const map<int, double> &m)
{
	map<int, double>::const_iterator it;
	for (it = m.begin(); it != m.end(); it++)
	{
		cout << it->first << ": " << it->second << endl;
	}
}

int ChooseArgMaxKey(const map<int, double> &m)
{
	double v_max = -DBL_MAX;
	int max_index = -1;
	for (auto const &x : m)
	{
		if (x.second > v_max)
		{
			v_max = x.second;
			max_index = x.first;
		}
	}

	return max_index;
}

void PrintDecPomdpJointObs(DecPomdpInterface *pb, int JOI)
{
	int Nb_agent = pb->GetNbAgents();
	vector<int> obs_indices = pb->JointToIndividualObsIndices(JOI);
	for (int agent_I = 0; agent_I < Nb_agent; agent_I++)
	{
		cout << "agent " << agent_I << ": " << pb->GetAllObservationsVecs()[agent_I][obs_indices[agent_I]] << ", ";
	}
	cout << endl;
}
void PrintDecPomdpJointAction(DecPomdpInterface *pb, int JAI)
{
	int Nb_agent = pb->GetNbAgents();
	vector<int> action_indices = pb->JointToIndividualActionsIndices(JAI);
	for (int agent_I = 0; agent_I < Nb_agent; agent_I++)
	{
		cout << "agent " << agent_I << ": " << pb->GetAllActionsVecs()[agent_I][action_indices[agent_I]] << ", ";
	}
	cout << endl;
}

double ComputeNorm1Distance(BeliefParticles &belief1, BeliefParticles &belief2)
{
	map<int, double> *b1 = belief1.GetPbDistState();
	map<int, double> *b2 = belief2.GetPbDistState();

	return ComputeNorm1Distance(*b1, *b2);
}

double ComputeNorm1Distance(map<int, double> &belief1, map<int, double> &belief2)
{
	set<int> all_sI;

	double res_distance = 0;

	map<int, double>::iterator it;
	for (it = belief1.begin(); it != belief1.end(); it++)
	{
		int sI = it->first;
		all_sI.insert(sI);
	}
	for (it = belief2.begin(); it != belief2.end(); it++)
	{
		int sI = it->first;
		all_sI.insert(sI);
	}

	set<int>::iterator it_sI;
	for (it_sI = all_sI.begin(); it_sI != all_sI.end(); ++it_sI)
	{
		int sI = *it_sI;
		double pb_sI_b1 = 0;
		double pb_sI_b2 = 0;
		if (belief1.count(sI))
		{
			pb_sI_b1 = belief1.find(sI)->second;
		}
		if (belief2.count(sI))
		{
			pb_sI_b2 = belief2.find(sI)->second;
		}

		double norm1_dis_sI = abs(pb_sI_b1 - pb_sI_b2);
		res_distance += norm1_dis_sI;
	}

	return res_distance;
}

int SampleOneNumber(int min, int max)
{
	int sampled_int = RandT(min, max);
	int i = 0;
	// int max_try = 1e4;
	while (sampled_int < min || sampled_int >= max)
	{
		sampled_int = RandT(min, max);
		i++;
		if (i == max_try)
		{
			cout << "sample failure in belief update!" << endl;
			throw "";
		}
	}

	return sampled_int;
}

BeliefParticles BuildInitDist(SimInterface *sim)
{
	vector<int> particles;
	map<int, int> sI_counts;
	map<int, double> current_pb_dist;
	map<int, double> previous_pb_dist;
	// check convergence after each a batch number particles
	// int batch_nb = 100;
	// int j = 0;
	for (int i = 0; i < max_try; i++)
	{
		int sI_sampled = sim->SampleStartState();
		particles.push_back(sI_sampled);
		sI_counts[sI_sampled] += 1;
		current_pb_dist[sI_sampled] = (double)sI_counts[sI_sampled] / i;
	}

	return BeliefParticles(particles, current_pb_dist);
}

BeliefParticles UpdateBeliefandProb(SimInterface &sim, BeliefParticles &b, int aI, int oI, double &out_pb)
{
	vector<int> particles;
	map<int, int> sI_counts;
	map<int, double> current_pb_dist;
	map<int, double> previous_pb_dist;
	tuple<int, int, double, bool> temp_res;
	int sampled_sI;
	// check convergence after each a batch number particles
	int oI_counts = 0;
	// int batch_nb = 100;
	// int j = 0;
	for (int i = 0; i < max_try; i++)
	{
		sampled_sI = b.SampleOneState();
		temp_res = sim.Step(sampled_sI, aI);
		int s_newI = get<0>(temp_res);
		int o_newI = get<1>(temp_res);
		if (o_newI == oI)
		{
			oI_counts += 1;
			particles.push_back(s_newI);
			sI_counts[s_newI] += 1;
			current_pb_dist[s_newI] = (double)sI_counts[s_newI] / oI_counts;
		}
	}

	if (particles.size() == 0)
	{
		cout << "error in belief update!" << endl;
		throw "";
	}

	out_pb = (double)oI_counts / max_try;
	RenormalizeMap(current_pb_dist, epsilon);
	return BeliefParticles(particles, current_pb_dist);
}

BeliefParticles UpdateOneSideBeliefandProb(SimInterface &sim, BeliefParticles &b,
										   int JaI, int local_oI, int local_agent_index, double &out_pb)
{
	int local_oI_counts = 0;

	vector<int> particles;
	map<int, int> sI_counts;
	map<int, double> current_pb_dist;
	// map<int, double> previous_pb_dist;
	tuple<int, int, double, bool> temp_res;
	int sampled_sI;
	for (int i = 0; i < max_try; i++)
	{
		sampled_sI = b.SampleOneState();
		temp_res = sim.Step(sampled_sI, JaI);
		int s_newI = get<0>(temp_res);
		int o_newI = get<1>(temp_res);
		vector<int> obs_indices = sim.JointToIndividualObsIndices(o_newI);
		int local_o_newI = obs_indices[local_agent_index];
		if (local_o_newI == local_oI)
		{
			particles.push_back(s_newI);
			local_oI_counts += 1;
			sI_counts[s_newI] += 1;
			current_pb_dist[s_newI] = (double)sI_counts[s_newI] / local_oI_counts;
		}
	}

	if (particles.size() == 0)
	{
		cout << "error in belief update!" << endl;
		throw "";
	}

	out_pb = (double)local_oI_counts / max_try;
	RenormalizeMap(current_pb_dist, epsilon);
	return BeliefParticles(particles, current_pb_dist);
}

void GetNextBeliefs(SimInterface &sim, BeliefParticles &b, int JaI, map<int, double> &out_pr_oba, map<int, BeliefParticles> &out_beliefs)
{
	map<int, vector<int>> map_particles_states;
	map<int, map<int, double>> map_pb_dist;
	map<int, double> JoI_counts;
	// vector<int> particles_obs;
	tuple<int, int, double, bool> temp_res;
	int sampled_sI;
	out_pr_oba.clear();
	out_beliefs.clear();
	for (int i = 0; i < max_try; i++)
	{
		sampled_sI = b.SampleOneState();
		temp_res = sim.Step(sampled_sI, JaI);
		int s_newI = get<0>(temp_res);
		int o_newI = get<1>(temp_res);
		JoI_counts[o_newI] += 1.0;
		if (map_particles_states.count(o_newI))
		{
			map_particles_states[o_newI].push_back(s_newI);
			map_pb_dist[o_newI][s_newI] += 1.0;
		}
		else
		{
			map_particles_states[o_newI] = {s_newI};
			map_pb_dist[o_newI][s_newI] = 1.0;
		}
		// particles_obs.push_back(o_newI);
	}

	map<int, double>::iterator it_JOI;
	map<int, double>::iterator it_sI;
	for (it_JOI = JoI_counts.begin(); it_JOI != JoI_counts.end(); it_JOI++)
	{
		int JoI = it_JOI->first;
		out_pr_oba[JoI] = it_JOI->second / max_try;
		for (it_sI = map_pb_dist[JoI].begin(); it_sI != map_pb_dist[JoI].end(); it_sI++)
		{
			it_sI->second /= it_JOI->second;
		}
		out_beliefs[JoI] = BeliefParticles(map_particles_states[JoI], map_pb_dist[JoI]);
	}
}

void GetNextBeliefsOneSidedUpdate(SimInterface &sim, BeliefParticles &b, int JaI, int local_agent_index, map<int, double> &out_pr_oba, map<int, BeliefParticles> &out_beliefs)
{
	map<int, vector<int>> map_particles_states;
	map<int, map<int, double>> map_pb_dist;
	map<int, double> Local_oI_counts;
	tuple<int, int, double, bool> temp_res;
	int sampled_sI;
	out_pr_oba.clear();
	out_beliefs.clear();
	for (int i = 0; i < max_try; i++)
	{
		sampled_sI = b.SampleOneState();
		temp_res = sim.Step(sampled_sI, JaI);
		int s_newI = get<0>(temp_res);
		int o_newI = get<1>(temp_res);
		vector<int> obs_indices = sim.JointToIndividualObsIndices(o_newI);
		int local_o_newI = obs_indices[local_agent_index];
		// cout << "local oI:" << local_o_newI << endl;
		// cout << "JOI:" << o_newI << endl;
		Local_oI_counts[local_o_newI] += 1.0;
		if (map_particles_states.count(local_o_newI))
		{
			map_particles_states[local_o_newI].push_back(s_newI);
			map_pb_dist[local_o_newI][s_newI] += 1.0;
		}
		else
		{
			map_particles_states[local_o_newI] = {s_newI};
			map_pb_dist[local_o_newI][s_newI] = 1.0;
		}
		// particles_obs.push_back(o_newI);
	}

	map<int, double>::iterator it_local_oI;
	map<int, double>::iterator it_sI;
	for (it_local_oI = Local_oI_counts.begin(); it_local_oI != Local_oI_counts.end(); it_local_oI++)
	{
		int local_oI = it_local_oI->first;
		out_pr_oba[local_oI] = it_local_oI->second / max_try;
		for (it_sI = map_pb_dist[local_oI].begin(); it_sI != map_pb_dist[local_oI].end(); it_sI++)
		{
			it_sI->second /= it_local_oI->second;
		}
		out_beliefs[local_oI] = BeliefParticles(map_particles_states[local_oI], map_pb_dist[local_oI]);
	}
}

bool CheckBeliefConvergence(map<int, double> &previous_belief, map<int, double> &current_belief, double epsilon)
{
	double distance = ComputeNorm1Distance(previous_belief, current_belief);
	if (distance < epsilon)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void RenormalizeMap(map<int, double> &m, double min_pb)
{
	map<int, double> res;
	map<int, double>::iterator it;
	double sum_pb = 0;
	int nb = 0;
	for (it = m.begin(); it != m.end(); it++)
	{
		int key = it->first;
		double pb = it->second;
		if (pb > min_pb)
		{
			res[key] = pb;
			sum_pb += pb;
			nb += 1;
		}
	}

	for (it = res.begin(); it != res.end(); it++)
	{
		int key = it->first;
		double pb = it->second;
		res[key] = pb / sum_pb;
	}
	m = res;
}