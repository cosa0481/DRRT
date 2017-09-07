#ifndef KDNODE_H
#define KDNODE_H

#include <DRRT/libraries.h>

class Kdnode
{
    bool in_tree_;
    double LMC_;

public:
    Kdnode(double lmc) : in_tree_(false), LMC_(lmc) {}
    Kdnode() : in_tree_(false), LMC_(INF) {}

    double GetLmc() { return LMC_; }
    void SetLmc(double val) { LMC_ = val; }
};

#endif // KDNODE_H
