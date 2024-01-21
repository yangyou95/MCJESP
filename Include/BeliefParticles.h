/* This file has been written and/or modified by the following people:
 *
 * Anonymous for UAI reviewing process
 *
 */

#ifndef _BELIEFPARTICLES_H_
#define _BELIEFPARTICLES_H_

#include <iostream>
#include <vector>
#include <map>

using namespace std;

// weights particles (state) to represent the belief
class BeliefParticles
{
private:
    vector<int> particles;           // a vector of sI
    std::map<int, double> pb_states; // computed from the particles
    int size_particles = -1;
    // int size_states;

public:
    BeliefParticles(){};
    ~BeliefParticles(){};
    BeliefParticles(vector<int> &particles);
    BeliefParticles(vector<int> &particles, std::map<int, double> &b);
    int SampleOneState() const;
    int RandT(int min, int max) const;
    int SampleOneNumber(int min, int max) const;
    int GetParticleSize();
    double operator[](int i);
    bool operator==(BeliefParticles &o);
    void BuildBeliefSparse();
    map<int, double> *GetPbDistState();
};

#endif