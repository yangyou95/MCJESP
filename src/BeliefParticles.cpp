#include "../Include/BeliefParticles.h"

BeliefParticles::BeliefParticles(vector<int> &particles)
{
    this->particles = particles;
    this->size_particles = particles.size();
    BuildBeliefSparse();
}

BeliefParticles::BeliefParticles(vector<int> &particles, std::map<int, double> &b)
{
    this->particles = particles;
    this->size_particles = particles.size();
    this->pb_states = b;
}

void BeliefParticles::BuildBeliefSparse()
{
    this->pb_states.clear();
    for (size_t i_particle = 0; i_particle < this->particles.size(); i_particle++)
    {
        int sI = this->particles[i_particle];
        this->pb_states[sI] += (double)1 / size_particles;
    }
}

int BeliefParticles::SampleOneState() const
{
    int random_i_particle = SampleOneNumber(0, this->size_particles);
    return this->particles[random_i_particle];
}

int BeliefParticles::RandT(int min, int max) const
{
    int temp;
    if (min > max)
    {
        temp = max;
        max = min;
        min = temp;
    }
    return rand() / (double)RAND_MAX * (max - min) + min;
}

int BeliefParticles::SampleOneNumber(int min, int max) const
{
    int sampled_int = RandT(min, max);
    int max_try = 1e7;
    int i = 0;
    while (sampled_int < min || sampled_int >= max)
    {
        sampled_int = RandT(min, max);
        i++;

        if (i == max_try)
        {
            cout << "sample failure in belief particle!" << endl;
            throw "";
        }
    }

    return sampled_int;
}

int BeliefParticles::GetParticleSize()
{
    return this->size_particles;
}

map<int, double> *BeliefParticles::GetPbDistState()
{
    if (!this->pb_states.size())
    {
        cout << "empty belief particles?!!" << endl;
        this->BuildBeliefSparse();
    }
    return &this->pb_states;
}

bool BeliefParticles::operator==(BeliefParticles &o)
{
    bool res = true;

    map<int, double>::iterator it;
    for (it = this->pb_states.begin(); it != this->pb_states.end(); it++)
    {
        if (it->second != o[it->first])
        {
            res = false;
            return res;
        }
    }
    return res;
}
double BeliefParticles::operator[](int i)
{
    // if key absent
    if ((this->pb_states).find(i) == this->pb_states.end())
    {
        // returns proba 0
        return 0.;
    }
    // key present
    else
    {
        // returns associated value
        return this->pb_states[i];
    }
}
