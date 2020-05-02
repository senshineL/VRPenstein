/*
An efficient memetic algorithm for solving the 
Simaultanes Delivery and Pick-up Vehicle Routing
Problem with Time Windows (VRPSDPTW)

Created on: 2020.3.13
by Shengcai Liu
*/
#include "VRPenstein.h"

Solution best_s;
double best_s_cost;
clock_t find_best_time;
int find_best_run;
int find_best_gen;
clock_t find_bks_time;
int find_bks_run;
int find_bks_gen;
bool find_better;

void signalHandler(int signum)
{
    printf("Interrupt signal (%d) received.\n", signum);
    printf("Best cost: %.4f.\n", best_s_cost);
    printf("Time to find this solution: %d.\n", int(find_best_time));
    printf("Time to surpass BKS: %d.\n", int(find_bks_time));
    exit(signum);
}

int main(int argc, char **argv)
{
    best_s.cost = double(INFINITY);
    best_s_cost = -1.0;
    find_best_time = find_bks_time = 0;
    find_best_run = find_bks_run = 0;
    find_best_gen = find_bks_gen = 0;
    find_better = false;
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    ArgumentParser parser;
    // required arguments
    parser.addArgument("--problem", 1, false);

    // optional
    parser.addArgument("--pruning");
    parser.addArgument("--output", 1);
    parser.addArgument("--time", 1);
    parser.addArgument("--runs", 1);
    parser.addArgument("--g_1", 1);
    parser.addArgument("--pop_size", 1);
    parser.addArgument("--init", 1);
    parser.addArgument("--k_init", 1);
    parser.addArgument("--cross_repair", 1);
    parser.addArgument("--k_crossover", 1);
    parser.addArgument("--parent_selection", 1);
    parser.addArgument("--replacement", 1);
    parser.addArgument("--ls_prob", 1);
    parser.addArgument("--O_1_eval");
    parser.addArgument("--two_opt");
    parser.addArgument("--two_opt_star");
    parser.addArgument("--or_opt", 1);
    parser.addArgument("--two_exchange", 1);
    parser.addArgument("--elo", 1);
    parser.addArgument("--random_removal");
    parser.addArgument("--related_removal");
    parser.addArgument("--alpha", 1);
    parser.addArgument("--removal_lower", 1);
    parser.addArgument("--removal_upper", 1);
    parser.addArgument("--regret_insertion");
    parser.addArgument("--greedy_insertion");
    parser.addArgument("--rd_removal_insertion");
    parser.addArgument("--bks", 1);

    parser.addArgument("--random_seed", 1);

    // parse the command-line arguments - throws if invalid format
    parser.parse(argc, argv);
    Data data(parser);
    search_framework(data, best_s);

    // std::cin.get();
    return 0;
}