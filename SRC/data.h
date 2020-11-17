#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <random>
#include <map>
#include "config.h"
#include "argparse.h"
#include "util.h"
#include "move.h"

struct Point
{
    short id; // DC_ID: depot, otherwise: customers
    double pickup; // piuk_up demand
    double delivery; // delivery demand
    double s_time;   //service time
    double start; // start of time window
    double end; // end of time window
};

struct Vehicle
{
    short type;
    double capacity;
    short max_num;
    double unit_cost; // unit cost for transportation
    double d_cost; // dispatching cost
};
class Data
{
public:
    // problem definitions
    std::string problem_name;
    std::vector<Point> node;
    int customer_num; // shouldn't be larger than MAX_POINT
    std::vector<std::vector<double>> dist;
    std::vector<std::vector<double>> time;
    std::vector<std::vector<double>> rm;
    std::vector<std::vector<int>> rm_argrank;
    std::vector<std::vector<bool>> pm;

    Vehicle vehicle;
    double max_dist = 0; // max value in dist
    double min_dist = double(INFINITY); // min value in dist
    double all_pickup; // all pick-up demands
    double all_delivery; // all delivery demands
    double start_time; // the earliest time of servicing
    double end_time;   // the latest time of returning to depot

    int DC; // ID of depot

    // parameters
    bool pruning = DEFAULT_PRUNING; // whether do pruning
    bool O_1_evl = DEFAULT_O_1_EVAL; // whether do O(1) evaluation
    bool no_crossover = DEFAULT_NO_CROSSOVER; // whether do crossover
    bool if_output = DEFAULT_IF_OUTPUT; // whether output to file
    std::string output = " "; // output file path
    int tmax = NO_LIMIT; // max running time (s), -1 means no limit
    int g_1 = G_1; // terminate MA when no improving in g_1 gens
    int runs = RUNS; // restarts MA for runs times
    int p_size = P_SIZE; // population size
    int k_init = K; //the number of generated solutions for producing a solution during initialization
    int k_crossover = K; //the number of generated solutions for producing a solution during crossover
    int ksize = 0;
    int seed = DEFAULT_SEED; // random seed to initialize rng
    double bks = -1.0;       // best known solution
    std::mt19937 rng; // random number generator
    std::string init = DEFAULT_INIT; // initialization method
    std::string cross_repair = DEFAULT_CROSSOVER; // insertion used in crossover
    std::tuple<double, double> lambda_gamma = std::make_tuple(0.0,0.0); // params of RCRS method
    std::vector<std::tuple<double, double>> latin;
    std::string n_insert = ""; //indicate which type-1 insertion should be used, RCRS or TD

    std::string selection = DEFAULT_SEL;           //parent selection method
    std::string replacement = DEFAULT_REPLACEMENT; //replacement strategy

    double ls_prob = DEFAULT_LS_PROB; //local search probability
    bool skip_finding_lo = DEFAULT_SKIP_FINDING_LO; // if skip finding_local_optima
    std::map<std::string, std::vector<Move>> mem;
    bool two_opt = DEFAULT_2_OPT; // 2-opt
    std::vector<Move> mem_2opt;
    bool two_opt_star = DEFAULT_2_OPT_STAR; //2-opt*
    std::vector<Move> mem_2optstar;
    bool or_opt = DEFAULT_OR_OPT; // or-opt
    std::vector<Move> mem_oropt_single;
    std::vector<Move> mem_oropt_double;
    bool two_exchange = DEFAULT_2_EX; // 2-exchange
    std::vector<Move> mem_2ex;

    int or_opt_len = DEFAULT_OR_OPT_LEN; //max length of seqs relocated by oropt
    int exchange_len = DEFAUTL_EX_LEN;   //max length of seqs exchanged

    int escape_local_optima = DEFAULT_ELO;            // number of times of escaping local optima
    double destroy_ratio_l = DEFAULT_DESTROY_RATIO_L; //customers to be delete in recombination
    double destroy_ratio_u = DEFAULT_DESTROY_RATIO_U;
    bool random_removal = DEFAULT_RD_REMOVAL;
    bool related_removal = DEFAULT_RT_REMOVAL;
    bool greedy_insertion = DEFAULT_GD_INSERTION;
    bool regret_insertion = DEFAULT_RG_INSERTION;
    double alpha = DEFAULT_ALPHA;
    double r = 0.0;
    bool rd_removal_insertion = DEFAULT_RD_R_I;

    std::vector<std::string> small_opts;
    std::vector<std::string> destroy_opts;
    std::vector<std::string> repair_opts;
    Data(ArgumentParser &parser); // read problem files, set parameters
    void pre_processing();
    void clear_mem();
    Move& get_mem(std::string &opt, const int &r1, const int &r2);
};