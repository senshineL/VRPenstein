#pragma once

/* Sequence in a route. If r_index == -1, meaning not exists in any current route */
struct Seq
{
    int r_index;
    int start_point;
    int end_point;
};
/* Move incured by a local search opt. */
struct Move
{
    // -1: means a new route, -2 means not used
    int r_indice[2] = {-2, -2};
    Seq seqList_1[4];
    // 0: means invalid, -1 means infeasible
    int len_1 = 0;
    Seq seqList_2[4];
    int len_2 = 0;
    double delta_cost = double(INFINITY); // the smaller, the better
};
