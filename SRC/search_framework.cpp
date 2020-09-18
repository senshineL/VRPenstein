#include "search_framework.h"
extern clock_t find_best_time;
extern clock_t find_bks_time;
extern int find_best_run;
extern int find_best_gen;
extern int find_bks_run;
extern int find_bks_gen;
extern bool find_better;

void update_best_solution(Solution &s, Solution &best_s, clock_t used, int run, int gen, Data &data)
{
    if (s.cost - best_s.cost < -PRECISION)
    {
        best_s = s;
        printf("Best solution update: %.4f\n", best_s.cost);
        find_best_time = used;
        find_best_run = run;
        find_best_gen = gen;
        if (!find_better && (std::abs(best_s.cost - data.bks) < PRECISION ||
                             (best_s.cost - data.bks < -PRECISION)))
        {
            find_better = true;
            find_bks_time = used;
            find_bks_run = run;
            find_bks_gen = gen;
        }
    }
}

void initialization(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data)
{
    int len = int(pop.size());
    for (int i = 0; i < len; i++)
    {
        pop[i].clear(data);
    }
    printf("Initialization, using %s method\n", data.init.c_str());
    if (data.init == RCRS)
    {
        data.n_insert = RCRS;
        data.ksize = data.k_init;
        for (int i = 0; i < len; i++)
        {
            data.lambda_gamma = data.latin[i];
            // printf("lambda, gamma: %f, %f\n", get<0>(data.lambda_gamma), get<1>(data.lambda_gamma));
            new_route_insertion(pop[i], data);
        }
    }
    else if (data.init == RCRS_RANDOM)
    {
        data.n_insert = RCRS;
        data.ksize = data.k_init;
        for (int i = 0; i < len; i++)
        {
            data.lambda_gamma = std::make_tuple(rand(0, 1, data.rng), rand(0, 1, data.rng));
            printf("lambda, gamma: %f, %f\n", get<0>(data.lambda_gamma), get<1>(data.lambda_gamma));
            new_route_insertion(pop[i], data);
        }
    }
    else if (data.init == TD)
    {
        data.ksize = data.k_init;
        data.n_insert = TD;
        for (int i = 0; i < len; i++) {new_route_insertion(pop[i], data);}
    }
    else
    {
        /* more insertion heuristic */
    }

    for (int i = 0; i < len; i++)
    {
        pop_fit[i] = pop[i].cost;
        printf("Solution %d, cost %.4f\n", i, pop_fit[i]);
    }
    argsort(pop_fit, pop_argrank, len);
    printf("Initialization done.\n");
}

void tournament(vector<int> &indice, vector<double> pop_fit, int boundray, Data &data)
{
    int index_index_1 = randint(0, boundray, data.rng);
    // swap two values
    int tmp = indice[index_index_1];
    indice[index_index_1] = indice[boundray];
    indice[boundray] = tmp;
    int index_index_2 = randint(0, boundray-1, data.rng);
    tmp = indice[index_index_2];
    indice[index_index_2] = indice[boundray-1];
    indice[boundray-1] = tmp;
    // select one value from indice[boundray-1] and indice[boundray]
    int selected;
    if (abs(pop_fit[indice[boundray]] - pop_fit[indice[boundray-1]]) < PRECISION)
        selected = randint(boundray-1, boundray, data.rng);
    else if (pop_fit[indice[boundray]] < pop_fit[indice[boundray-1]])
        selected = boundray;
    else
        selected = boundray - 1;
    tmp = indice[selected];
    indice[selected] = indice[boundray];
    indice[boundray] = tmp;
}

void select_parents(vector<Solution> &pop, vector<double> pop_fit, vector<tuple<int, int>> &p_indice, Data &data)
{
    int len = data.p_size;
    vector<int> indice(len);
    iota(indice.begin(), indice.end(), 0);
    shuffle(indice.begin(), indice.end(), data.rng);
    if (data.selection == CIRCLE)
    {
        for (int i = 0; i < len - 1; i++)
        {
            get<0>(p_indice[i]) = indice[i];
            get<1>(p_indice[i]) = indice[i+1];
        }
        get<0>(p_indice[len-1]) = indice[len-1];
        get<1>(p_indice[len-1]) = indice[0];
    }
    else if (data.selection == TOURNAMENT)
    {
        for (int i = 0; i < len; i++)
        {
            tournament(indice, pop_fit, len-1, data);
            tournament(indice, pop_fit, len-2, data);
            get<0>(p_indice[i]) = indice[len-1];
            get<1>(p_indice[i]) = indice[len-2];
        }
    }
    else if (data.selection == RDSELECTION)
    {
        for (int i = 0; i < len; i++)
        {
            int index_index_1 = randint(0, len-1, data.rng);
            int tmp = indice[index_index_1];
            indice[index_index_1] = indice[len-1];
            indice[len-1] = tmp;
            int index_index_2 = randint(0, len-2, data.rng);
            get<0>(p_indice[i]) = indice[len-1];
            get<1>(p_indice[i]) = indice[index_index_2];
        }
    }
    else
    {
        /* other selection mechanims */
    }
}

void update_candidate_routes(Route &r, std::unordered_set<int> &inserted, Solution &s, vector<int> &candidate_r, Data &data)
{
    for (auto &node : r.node_list) {inserted.insert(node);}
    int i = 0;
    int len = int(candidate_r.size());
    while (i < len)
    {
        Route &r = s.get(candidate_r[i]);
        bool flag = true;
        for (auto &node : r.node_list)
        {
            if (node == data.DC) continue;
            if (inserted.count(node) == 1)
            {
                flag = false;
                break;
            }
        }
        if (!flag)
        {
            candidate_r.erase(candidate_r.begin() + i);
            len--;
        }
        else
            i++;
    }
}

void crossover(Solution &s1, Solution &s2, Solution &ch, Data &data)
{
    if (data.no_croosover)
    {
        ch = s1;
        return;
    }

    vector<int> candidate_r_1(s1.len());
    std::iota(candidate_r_1.begin(), candidate_r_1.end(), 0);
    vector<int> candidate_r_2(s2.len());
    std::iota(candidate_r_2.begin(), candidate_r_2.end(), 0);
    int count = 0;
    std::unordered_set<int> inserted;
    inserted.reserve(data.customer_num + 1);

    while (true)
    {
        if (int(candidate_r_1.size()) == 0) break;
        int selected = randint(0, int(candidate_r_1.size())-1, data.rng);
        Route &r_1 = s1.get(candidate_r_1[selected]);
        ch.append(r_1);
        update_candidate_routes(r_1, inserted, s2, candidate_r_2, data);
        if (int(candidate_r_2.size()) == 0) break;
        selected = randint(0, int(candidate_r_2.size())-1, data.rng);
        Route &r_2 = s2.get(candidate_r_2[selected]);
        ch.append(r_2);
        update_candidate_routes(r_2, inserted, s1, candidate_r_1, data);
    }
    // call insertion
    if (data.cross_repair == RCRS)
    {
        data.n_insert = RCRS;
        data.ksize = data.k_crossover;
        // using random lambda and gamma
        data.lambda_gamma = std::make_tuple(rand(0, 1, data.rng), rand(0, 1, data.rng));
        // printf("lambda, gamma: %f, %f\n", get<0>(data.lambda_gamma), get<1>(data.lambda_gamma));
        new_route_insertion(ch, data);
    }
    else if (data.cross_repair == TD)
    {
        data.ksize = data.k_crossover;
        data.n_insert = TD;
        new_route_insertion(ch, data);
    }
    else if (data.cross_repair == REGRET)
        regret_insertion(ch, data);

    ch.cal_cost(data);
}

void crossover(vector<Solution> &pop, Data &data, vector<tuple<int,int>> &p_indice, vector<Solution> &child)
{
    cout << "Do crossover." << endl;
    int count = 0;
    for (auto &index_t : p_indice)
    {
        if (randint(0, 1, data.rng) == 0)
            crossover(pop[get<0>(index_t)], pop[get<1>(index_t)], child[count], data);
        else
            crossover(pop[get<1>(index_t)], pop[get<0>(index_t)], child[count], data);
        count++;
        cout << "Child " << count << ". Parent Indice: (" << get<0>(index_t) <<\
                "," << get<1>(index_t) << "). Cost: " << child[count-1].cost << endl;
    }
}

void output(vector<Solution> &pop, vector<double> pop_fit, vector<int> pop_argrank,\
            Data &data, bool output_complete=false)
{
    int len = int(pop.size());
    double best_cost = pop_fit[pop_argrank[0]];
    double worst_cost = pop_fit[pop_argrank[len-1]];
    double avg_cost = mean(pop_fit, 0, len);
    printf("Avg %.4f, Best %.4f, Worst %.4f\n", avg_cost, best_cost, worst_cost);
    if (output_complete) {pop[pop_argrank[0]].output(data);}
}

void local_search(vector<Solution> &pop, vector<double> &pop_fit, vector<int> &pop_argrank, Data &data)
{
    // printf("Do local search\n");
    int len = int(pop.size());
    for (int i = 0; i < len; i++)
    {
        if (rand(0, 1, data.rng) < data.ls_prob)
        {
            cout << "Individual " << i+1 << ". Before Cost " << pop[i].cost << ".";
            do_local_search(pop[i], data);
            pop_fit[i] = pop[i].cost;
            cout << " After Cost " << pop_fit[i] << endl;
        }
    }
    argsort(pop_fit, pop_argrank, len);
}

void replacement(vector<Solution> &pop, vector<tuple<int, int>> &p_indice, vector<Solution> &child, vector<double> &pop_fit, vector<int> &pop_argrank, vector<double> &child_fit, vector<int> &child_argrank, Data &data)
{
    int len = int(child.size());
    if (data.replacement == ONE_ON_ONE)
    {
        for (int i = 0; i < len; i++)
        {
            auto &indice_t = p_indice[i];
            auto p_1_indice = std::get<0>(indice_t);
            if (child[i].cost - pop[p_1_indice].cost < -PRECISION)
            {
                pop[p_1_indice] = child[i];
                pop_fit[p_1_indice] = pop[p_1_indice].cost;
            }
        }
    }
    else if (data.replacement == ELITISM_1)
    {
        pop[len-1] = pop[pop_argrank[0]];
        pop_fit[len-1] = pop_fit[pop_argrank[0]];
        for (int i = 0; i < len-1; i++)
        {
            pop[i] = child[child_argrank[i]];
            pop_fit[i] = pop[i].cost;
        }
    }
    else
    {
        /* more replacement */
    }
    for(int i = 0; i < len; i++)
    {
        child[i].clear(data);
    }
}

void search_framework(Data &data, Solution &best_s)
{
    vector<Solution> pop(data.p_size);
    vector<Solution> child(data.p_size);
    for (int i = 0; i < data.p_size; i++)
    {
        pop[i].reserve(data);
        child[i].reserve(data);
    }
    // fitness
    vector<double> pop_fit(data.p_size);
    vector<double> child_fit(data.p_size);
    // argsort result of fitness array
    vector<int> pop_argrank(data.p_size); 
    vector<int> child_argrank(data.p_size); 

    // parent index in pop
    vector<tuple<int, int>> p_indice(data.p_size);

    clock_t stime = clock();
    clock_t used = 0;

    /* main body */
    bool time_exhausted = false;
    int run = 1;
    for (; run <= data.runs; run++)
    {
        printf("---------------------------------Run %d---------------------------\n", run);
        int no_improve = 0;
        int gen = 0;
        initialization(pop, pop_fit, pop_argrank, data);
        used = (clock() - stime) / CLOCKS_PER_SEC;
        printf("already consumed %d sec\n", used);

        local_search(pop, pop_fit, pop_argrank, data);
        used = (clock() - stime) / CLOCKS_PER_SEC;
        printf("already consumed %d sec\n", used);

        printf("After local search\n");
        output(pop, pop_fit, pop_argrank, data);
        double cost_in_this_run = pop_fit[pop_argrank[0]];
        while (!termination(no_improve, data))
        {
            gen++;
            // printf("---------------------------------Gen %d---------------------------\n", gen);
            no_improve++;
            // select parents
            select_parents(pop, pop_fit, p_indice, data);
            // crossover
            crossover(pop, data, p_indice, child);
            used = (clock() - stime) / CLOCKS_PER_SEC;
            printf("already consumed %d sec\n", used);
            // do local search for children
            local_search(child, child_fit, child_argrank, data);
            // replacement
            replacement(pop, p_indice, child, pop_fit, pop_argrank, child_fit, child_argrank, data);
            // update best
            argsort(pop_fit, pop_argrank, data.p_size);
            update_best_solution(pop[pop_argrank[0]], best_s, used, run, gen, data);
            if (pop_fit[pop_argrank[0]] - cost_in_this_run < -PRECISION)
            {
                // std::cout << pop_fit[pop_argrank[0]] << " " << cost_in_this_run << std::endl;
                no_improve = 0;
                cost_in_this_run = pop_fit[pop_argrank[0]];
            }

            used = (clock() - stime) / CLOCKS_PER_SEC;
            if (gen % OUTPUT_PER_GENS == 0)
            {
                printf("Gen: %d. ", gen);
                output(pop, pop_fit, pop_argrank, data);
                printf("Gen %d done, no improvement for %d gens, already consumed %d sec\n", gen, no_improve, used);
            }
            // printf("-----------------------------------------------------------------\n");

            if (data.tmax != NO_LIMIT && used > clock_t(data.tmax))
            {
                time_exhausted = true;
                break;
            }
        }
        printf("Run %d finishes\n", run);
        output(pop, pop_fit, pop_argrank, data);

        data.rng.seed(data.seed + run);
        if (time_exhausted) break;
    }
    // output best solution
    printf("------------Summary-----------\n");
    printf("Total %d runs, total consumed %d sec\n", run, int(used));
    best_s.output(data);
    printf("In run %d, gen %d, find this solution, at time %d.\n", find_best_run, find_best_gen, int(find_best_time));
    printf("Time to surpass BKS: %d.\n", int(find_bks_time));
    best_s.check(data);
}