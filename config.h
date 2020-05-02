#pragma once
#include "limits.h"

// The following definitions might need to be changed
// for different problems
const int MAX_POINT = 1250;
const int MAX_NODE_IN_ROUTE = 100;
const int DEFAULT_SEED = 42;
const int V_NUM_RELAX = 3;

const int INF = INT_MAX;
const int N = 8192;      //buffer size for raading problem
const int NO_LIMIT = -1; // default time limit
const int G_1 = 500; //default max generations when no improving
const int RUNS = 10; //default max restart times
const int P_SIZE = 64; //default population size
const double PRECISION = 0.001; // difference smaller than this is considered the same
const int OUTPUT_PER_GENS = 2; // output per generations
const bool DEFAULT_IF_OUTPUT = false;

/* pre-processing */
const bool DEFAULT_PRUNING = false;

/* local search */
const double DEFAULT_LS_PROB = 1.0; //default local search probability
const bool DEFAULT_O_1_EVAL = false;
const bool DEFAULT_2_OPT_STAR = true;
const bool DEFAULT_2_OPT = false;
const bool DEFAULT_OR_OPT = false;
const bool DEFAULT_2_EX = false;
const int DEFAULT_OR_OPT_LEN = 3;
const int DEFAUTL_EX_LEN = 2;

/* large neighborhood opts */
const int DEFAULT_ELO = 1;
const double DEFAULT_DESTROY_RATIO_L = 0.2;
const double DEFAULT_DESTROY_RATIO_U = 0.4;
const bool DEFAULT_RD_REMOVAL = false;
const bool DEFAULT_RT_REMOVAL = false;
const bool DEFAULT_GD_INSERTION = false;
const bool DEFAULT_RG_INSERTION = false;
const bool DEFAULT_RD_R_I = false;

/* insertion type-1: insertion without considering existing routes,
each time building a new route with an arbitrary selected customer.
K is the number of the selected initial customers. The final solution
is the best one among the K solutions */
const std::string RCRS = "rcrs"; // RCRS insertion heuristic
const std::string TD = "td";  // travel-distance based insertion
const std::string REGRET = "regret";

const std::string DEFAULT_INIT = RCRS;
const std::string DEFAULT_CROSSOVER = RCRS;

const int K = -1; // the number of generated solutions for producing a solution

/* Parent selection */
const std::string CIRCLE = "circle"; // circle selection, every two individuals as parents
const std::string TOURNAMENT = "tournament"; // tournament selection
const std::string RDSELECTION = "rdslection"; // uniformly random selection
const std::string DEFAULT_SEL = CIRCLE;

/* Replacement */
const std::string ONE_ON_ONE = "one_on_one";
const std::string ELITISM_1 = "elitism";
const std::string DEFAULT_REPLACEMENT = ONE_ON_ONE;

/* removal and insertion */
const double DEFAULT_ALPHA = 1.0;   // relatedness para
const double PENALTY_FACTOR = 10.0; // penalty factor for tw constraint

// The flags for a sequence status
const int INFEASIBLE = -1;