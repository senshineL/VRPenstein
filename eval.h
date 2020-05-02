#pragma once
#include "solution.h"
#include "data.h"
#include "move.h"

void chk_nl_node_pos_O_n(std::vector<int> &nl, int inserted_node, int pos, Data &data, bool &flag, double &cost);

void chk_route_O_n(Route &r, Data &data, bool &flag, double &cost);

bool eval_move(Solution &s, Move &m, Data &data);

static inline bool check_capacity(const Attr &a, const Attr &b, Data &data)
{
    return std::max(a.C_H + b.C_E, a.C_L + b.C_H) - data.vehicle.capacity <= 0;
}

static inline bool check_tw(const Attr &a, const Attr &b, Data &data)
{
    return (a.T_E + a.T_D + data.time[a.e][b.s] - b.T_L) <= 0;
}