## Introduction
VRPenstein is a parameterized meta-heuristic for the vehicle routing problems (VRP). Currently, the configuration space of VRPenstein includes two different types of meta-heuristics:  
1. Memetic Algorithm (Population-based search with local search)
2. Iterated Local Search (Single point search)

Now the supported VRP variants include
1. vehicle routing problem (VRP)
2. capacitated vehicle routing problem (CVRP)
3. capacitated vehicle routing problem with time windows (VRPTW)
4. capacitated vehicle routing problem with pick-up and time windows (VRPPTW)
5. simaultanes Delivery and Pick-up Vehicle Routing
Problem with Time Windows (VRPSDPTW)

## Install
```
git clone git@github.com:senshineL/VRPenstein.git
mkdir Bin
cd SRC
g++ -std=c++11 -o ../bin/VRPenstein -O3 MemeticAlgorithm.cpp eval.cpp operator.cpp search_framework.cpp solution.cpp util.cpp data.cpp
```
## Usage
```
VRPenstein --problem PROBLEM [--pruning] [--output OUTPUT] [--time TIME] [--runs RUNS] [--g_1 G_1] [--pop_size POP_SIZE] [--init INIT] [--k_init K_INIT] [--cross_repair CROSS_REPAIR] [--k_crossover K_CROSSOVER] [--parent_selection PARENT_SELECTION] [--replacement REPLACEMENT] [--ls_prob LS_PROB] [--O_1_eval] [--two_opt] [--two_opt_star] [--or_opt OR_OPT] [--two_exchange TWO_EXCHANGE] [--elo ELO] [--random_removal] [--related_removal] [--alpha ALPHA] [--removal_lower REMOVAL_LOWER] [--removal_upper REMOVAL_UPPER] [--regret_insertion] [--greedy_insertion] [--rd_removal_insertion] [--bks BKS] [--random_seed RANDOM_SEED]

```
<!-- ### Parameters

### Input Format -->

### LICENSE
This software is released under MIT license.