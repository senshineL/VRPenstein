/*
Leave as much space as possible for adding more implementation options of each
function, such that maybe we could use this as an algorithm
framework for further automatic algorithm design
*/

#pragma once
#include <numeric>
#include <algorithm>
#include <unordered_set>
#include "stdio.h"
#include "data.h"
#include "solution.h"
#include "time.h"
#include "operator.h"
#include "config.h"
using namespace std;

void search_framework(Data &data, Solution &s);
void initialization(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data);

static inline bool termination(int no_improve, Data &data)
{
    return (no_improve > data.g_1);
}
