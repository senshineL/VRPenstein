#pragma once
#include <functional>
#include <map>
#include <numeric>
#include <cmath>
#include "solution.h"
#include "data.h"
#include "config.h"
#include "eval.h"
#include "move.h"

/* construct a complete solution by inserting unrouted nodes into s 
without considering existing routes in s*/
void new_route_insertion(Solution &s, Data &data);

void new_route_insertion(Solution &s, Data &data, int initial_node);

/* do local search to s */
void do_local_search(Solution &s, Data &data);
/* perturb solution s by destroy and repair */
void perturb(std::vector<Solution> &s_vector, Data &data);
void random_removal(Solution &s, Data &data);
void related_removal(Solution &s, Data &data);
void regret_insertion(Solution &s, Data &data);
void greedy_insertion(Solution &s, Data &data);

/* In future, we could use memory to store the best solutions in sub neighborhoods,
such that we could avoid repeated computation
 */
void two_opt(int r1, int r2, Solution &s, Data &data, Move &m);
void two_opt_star(int r1, int r2, Solution &s, Data &data, Move &m);
void or_opt_single(int r1, int r2, Solution &s, Data &data, Move &m);
void or_opt_double(int r1, int r2, Solution &s, Data &data, Move &m);
void two_exchange(int r1, int r2, Solution &s, Data &data, Move &m);
