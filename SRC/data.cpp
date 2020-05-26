#include "data.h"

Data::Data(ArgumentParser &parser)
{
    // read problem file
    const char* pro_file = parser.retrieve<std::string>("problem").c_str();
    char Buffer[N];
    std::ifstream fp;
    fp.rdbuf()->pubsetbuf(Buffer, N);
    fp.open(pro_file);

    std::string line;
    double all_pickup = 0.0;
    double all_delivery = 0.0;
    double all_dist = 0.0;
    double all_time = 0.0;
    bool flag = true;
    while (true)
    {
        if (flag)
        {
            if (!std::getline(fp, line)) break;
        }
        else flag = true;
        trim(line);
        if (line.size() == 0) continue;
        std::vector<std::string> results = split(line, ':');
        trim(results[0]);
        if (results.size() > 1) trim(results[1]);
        if (results[0] == "NAME")
        {
            printf("%s\n", line.c_str());
            this->problem_name = results[1];
        }
        else if (results[0] == "TYPE")
        {
            printf("%s\n", line.c_str());
        }
        else if (results[0] == "DIMENSION")
        {
            printf("%s\n", line.c_str());
            this->customer_num = stoi(results[1]) - 1;
            std::vector<double> tmp_v_1(this->customer_num+1, 0.0);
            std::vector<bool> tmp_v_2(this->customer_num+1, 0.0);
            std::vector<int> tmp_v_3(this->customer_num+1, 0);
            for (int i = 0; i <= this->customer_num; i++)
            {
                this->node.push_back({0, 0.0, 0.0, 0.0, 0.0, 0.0});
                this->dist.push_back(tmp_v_1);
                this->time.push_back(tmp_v_1);
                this->rm.push_back(tmp_v_1);
                this->rm_argrank.push_back(tmp_v_3);
                this->pm.push_back(tmp_v_2);
            }
        }
        else if (results[0] == "VEHICLES")
        {
            printf("%s\n", line.c_str());
            this->vehicle.max_num = stoi(results[1]) + V_NUM_RELAX;
        }
        else if (results[0] == "DISPATCHINGCOST")
        {
            printf("%s\n", line.c_str());
            this->vehicle.d_cost = stod(results[1]);
        }
        else if (results[0] == "UNITCOST")
        {
            printf("%s\n", line.c_str());
            this->vehicle.unit_cost = stod(results[1]);
        }
        else if (results[0] == "CAPACITY")
        {
            printf("%s\n", line.c_str());
            this->vehicle.capacity = stod(results[1]);
        }
        else if (results[0] == "EDGE_WEIGHT_TYPE")
        {
            printf("%s\n", line.c_str());
            if (results[1] != "EXPLICIT")
            {
                printf("Expect edge weight type: EXPLICIT, while accept type: %s\n", results[1].c_str());
                exit(-1);
            }
        }
        else if (results[0] == "NODE_SECTION")
        {
            while (getline(fp, line))
            {
                trim(line);
                if (line.size() == 0) continue;
                std::vector<std::string> r = split(line, ',');
                if (r.size() > 1)
                {
                    trim(r[0]);
                    int i = stoi(r[0]);
                    trim(r[1]);
                    this->node[i].delivery = stod(r[1]);
                    all_delivery += this->node[i].delivery;
                    trim(r[2]);
                    this->node[i].pickup = stod(r[2]);
                    all_pickup += this->node[i].pickup;
                    trim(r[3]);
                    this->node[i].start = stod(r[3]);
                    trim(r[4]);
                    this->node[i].end = stod(r[4]);
                    trim(r[5]);
                    this->node[i].s_time = stod(r[5]);
                }
                else
                {
                    flag = false;
                    break;
                }

            }
        }
        else if (results[0] == "DISTANCETIME_SECTION")
        {
            while (getline(fp, line))
            {
                trim(line);
                if (line.size() == 0)
                    continue;
                std::vector<std::string> r = split(line, ',');
                if (r.size() > 1)
                {
                    trim(r[0]);
                    int i = stoi(r[0]);
                    trim(r[1]);
                    int j = stoi(r[1]);
                    trim(r[2]);
                    double d = stod(r[2]);
                    trim(r[3]);
                    double t = stod(r[3]);
                    this->dist[i][j] = d;
                    all_dist += d;
                    this->time[i][j] = t;
                    all_time += t;
                    if (d < this->min_dist) this->min_dist = d;
                    if (d > this->max_dist) this->max_dist = d;
                }
                else
                {
                    flag = false;
                    break;
                }
            }
        }
        else if (results[0] == "DEPOT_SECTION")
        {
            getline(fp, line);
            trim(line);
            this->DC = stoi(line);
        }
    }
    fp.close();

    this->start_time = this->node[this->DC].start;
    this->end_time = this->node[this->DC].end;
    this->all_delivery = all_delivery;
    this->all_pickup = all_pickup;

    // print summary information
    printf("Avg pick-up/dilvery demand: %.4f,%.4f\n", this->all_pickup/this->customer_num, this->all_delivery/this->customer_num);
    printf("Starting/end time of DC: %.4f,%.4f\n", this->start_time, this->end_time);

    std::cout << '\n';
    if (parser.exists("random_seed"))
        this->seed = std::stoi(parser.retrieve<std::string>("random_seed"));
    this->rng.seed(this->seed);
    printf("Initial random seed: %d\n", this->seed);

    // set parameters
    if (parser.exists("pruning"))
    {
        printf("Pruning: on\n");
        this->pruning = true;
    }
    else printf("Pruning: off\n");

    if (parser.exists("output"))
    {
        this->if_output = true;
        this->output = parser.retrieve<std::string>("output");
        std::cout << "Write best solution to " << this->output << '\n';
    }

    if (parser.exists("time"))
        this->tmax = std::stoi(parser.retrieve<std::string>("time"));
    printf("Time limit: %d seconds\n", this->tmax);

    if (parser.exists("runs"))
        this->runs = std::stoi(parser.retrieve<std::string>("runs"));
    printf("Runs: %d\n", this->runs);

    if (parser.exists("g_1"))
        this->g_1 = std::stoi(parser.retrieve<std::string>("g_1"));
    printf("g_1: %d\n", this->g_1);

    if (parser.exists("pop_size"))
        this->p_size = std::stoi(parser.retrieve<std::string>("pop_size"));
    printf("Population size: %d\n", this->p_size);
    if (!chk_p_square(this->p_size))
    {
        printf("Expect popsize to be perfect squrare number\n");
        exit(-1);
    }
    int sr = int(sqrt(double(this->p_size)));
    if (sr == 1)
        this->latin.push_back(std::make_tuple(0.5, 0.5));
    else
    {
        double step = 1.0 / (sr - 1);
        for (int i = 0; i < sr; i++)
        {
            for (int j = 0; j < sr; j++)
            {
                double lambda = std::min(1.0, step * i);
                double gamma = std::min(1.0, step * j);
                this->latin.push_back(std::make_tuple(lambda, gamma));
            }
        }
        std::shuffle(this->latin.begin(), this->latin.end(), this->rng);
    }

    if (parser.exists("init"))
        this->init = parser.retrieve<std::string>("init");
    printf("Insertion for initialization: %s\n", this->init.c_str());
    if (parser.exists("k_init"))
        this->k_init = std::stoi(parser.retrieve<std::string>("k_init"));
    if (this->k_init == K)
        this->k_init = this->customer_num;
    printf("k_init: %d\n", this->k_init);

    if (parser.exists("cross_repair"))
        this->cross_repair = parser.retrieve<std::string>("cross_repair");
    printf("Insertion for crossover: %s\n", this->cross_repair.c_str());
    
    if (parser.exists("k_crossover"))
        this->k_crossover = std::stoi(parser.retrieve<std::string>("k_crossover"));
    if (this->k_crossover == K)
        this->k_crossover = this->customer_num;
    printf("k_crossover: %d\n", this->k_crossover);

    if (parser.exists("parent_selection"))
        this->selection = parser.retrieve<std::string>("parent_selection");
    printf("Parent selection: %s\n", this->selection.c_str());

    if (parser.exists("replacement"))
        this->replacement = parser.retrieve<std::string>("replacement");
    printf("Replacement strategy: %s\n", this->replacement.c_str());

    if (parser.exists("ls_prob"))
        this->ls_prob = std::stod(parser.retrieve<std::string>("ls_prob"));
    printf("Local search probability: %.2f\n", this->ls_prob);

    if (parser.exists("O_1_eval"))
    {
        printf("O(1) evaluation: on\n");
        this->O_1_evl = true;
    }
    else
        printf("O(1) evaluation: off");

    if (parser.exists("two_opt"))
    {
        printf("2-opt: on\n");
        this->two_opt = true;
        small_opts.push_back("2opt");
        std::vector<Move> tmp_mem(this->vehicle.max_num);
        mem.insert(std::pair<std::string, std::vector<Move>>("2opt", tmp_mem));
    }
    else
        printf("2-opt: off\n");

    if (parser.exists("two_opt_star"))
    {
        printf("2-opt*: on\n");
        this->two_opt_star = true;
        small_opts.push_back("2opt*");
        std::vector<Move> tmp_mem(this->vehicle.max_num * this->vehicle.max_num);
        mem.insert(std::pair<std::string, std::vector<Move>>("2opt*", tmp_mem));
    }
    else
        printf("2-opt*: off\n");

    if (parser.exists("or_opt"))
    {
        printf("or-opt: on\n");
        this->or_opt = true;
        this->or_opt_len = std::stoi(parser.retrieve<std::string>("or_opt"));
        small_opts.push_back("oropt_single");
        small_opts.push_back("oropt_double");
        std::vector<Move> tmp_mem_0(this->vehicle.max_num);
        std::vector<Move> tmp_mem_1(this->vehicle.max_num * this->vehicle.max_num);
        mem.insert(std::pair<std::string, std::vector<Move>>("oropt_single", tmp_mem_0));
        mem.insert(std::pair<std::string, std::vector<Move>>("oropt_double", tmp_mem_1));
    }
    else
        printf("or-opt: off\n");

    if (parser.exists("two_exchange"))
    {
        printf("2-exchange: on\n");
        this->two_exchange = true;
        this->exchange_len = std::stoi(parser.retrieve<std::string>("two_exchange"));
        small_opts.push_back("2exchange");
        std::vector<Move> tmp_mem(this->vehicle.max_num * this->vehicle.max_num);
        mem.insert(std::pair<std::string, std::vector<Move>>("2exchange", tmp_mem));
    }
    else
        printf("2-exchange: off\n");

    if (parser.exists("elo"))
        this->escape_local_optima = std::stoi(parser.retrieve<std::string>("elo"));
    printf("escape local optima number: %d\n", this->escape_local_optima);

    if (parser.exists("random_removal"))
    {
        printf("random_removal: on\n");
        this->random_removal = true;
        destroy_opts.push_back("random_removal");
    }
    else
        printf("random_removal: off\n");

    if (parser.exists("related_removal"))
    {
        printf("related_removal: on\n");
        this->related_removal = true;
        if (parser.exists("alpha"))
            this->alpha = std::stod(parser.retrieve<std::string>("alpha"));
        this->r = this->alpha * (all_dist / all_time);
        destroy_opts.push_back("related_removal");
        printf("alpha: %f, relateness norm factor: %f\n", this->alpha, this->r);
    }
    else
        printf("related_removal: off\n");

    if (parser.exists("removal_lower"))
    {
        this->destroy_ratio_l = std::stod(parser.retrieve<std::string>("removal_lower"));
    }
    printf("Destroy lower ration: %f\n", this->destroy_ratio_l);
    if (parser.exists("removal_upper"))
    {
        this->destroy_ratio_u = std::stod(parser.retrieve<std::string>("removal_upper"));
    }
    printf("Destroy upper ration: %f\n", this->destroy_ratio_u);

    if (parser.exists("regret_insertion"))
    {
        printf("regret_insertion: on\n");
        this->regret_insertion = true;
        repair_opts.push_back("regret_insertion");
    }
    else
        printf("regret_insertion: off\n");

    if (parser.exists("greedy_insertion"))
    {
        printf("greedy_insertion: on\n");
        this->greedy_insertion = true;
        repair_opts.push_back("greedy_insertion");
    }
    else
        printf("greedy_insertion: off\n");

    if (parser.exists("rd_removal_insertion"))
    {
        printf("Random removal and insertion: on\n");
        this->rd_removal_insertion = true;
    }
    else
        printf("Random removal and insertion: off\n");

    if (parser.exists("bks"))
        this->bks = std::stod(parser.retrieve<std::string>("bks"));

    int c_num = this->customer_num;
    for (int i = 0; i <= c_num; i++)
    {
        for (int j = 0; j <= c_num; j++)
        {this->pm[i][j] = true;}
    }
    this->pre_processing();
}

void Data::pre_processing()
{
    printf("--------------------------------------------\n");
    if (this->related_removal)
    {
        int c_num = this->customer_num;
        int DC = this->DC;
        for (int i = 0; i <= c_num; i++)
        {
            if (i == DC) continue;
            for (int j = 0; j <= c_num; j++)
            {
                if(j == DC || j == i)
                    this->rm[i][j] = double(INFINITY);
                else
                {
                    auto &node_i = this->node[i];
                    auto &node_j = this->node[j];
                    double tmp_1 = this->r * std::max(node_j.start - node_i.s_time - this->time[i][j] - node_i.end, 0.0);
                    double tmp_2 = this->r * PENALTY_FACTOR * std::max(node_i.start + node_i.s_time + this->time[i][j] - node_j.end, 0.0);
                    double tmp_3 = this->dist[i][j];
                    this->rm[i][j] = tmp_3 + tmp_1 + tmp_2;
                }
            }
            argsort(this->rm[i], this->rm_argrank[i], c_num+1);
        }
    }
    if (this->pruning)
    {
        printf("Do Pruning\n");
        int c_num = this->customer_num;
        int DC = this->DC;
        int count_tw = 0;
        int count_c = 0;
        for (int i = 0; i <= c_num; i++)
        {
            if (i == DC) continue;
            for (int j = 0; j <= c_num; j++)
            {
                if (j == DC || j == i) continue;

                double a_i = this->node[i].start;
                double s_i = this->node[i].s_time;
                double d_i = this->node[i].delivery;
                double p_i = this->node[i].pickup;

                double b_j = this->node[j].end;
                double d_j = this->node[j].delivery;
                double p_j = this->node[j].pickup;
                double time_ij = this->time[i][j];

                if (a_i + s_i + time_ij > b_j)
                {
                    this->pm[i][j] = false;
                    count_tw++;
                }
                if (d_i + d_j > this->vehicle.capacity || p_i + p_j > this->vehicle.capacity)
                {
                    this->pm[i][j] = false;
                    count_c++;
                }
            }
        }
        int sum = c_num*(c_num-1);
        printf("Total edges %d, prune by time window %d(%.4f%%),prune by capacity %d(%.4f%%)\n",\
                sum, count_tw, 100.0*double(count_tw)/sum, count_c, 100.0*double(count_c)/sum);
    }
}

void Data::clear_mem()
{
    for (auto &x : this->mem)
    {
        for (auto &move : x.second)
        {
            move.len_1 = 0;
        }
    }
}

Move &Data::get_mem(std::string &opt, const int &r1, const int &r2)
{
    if (opt == "2opt")
        return this->mem[opt][r1];
    else if (opt == "2opt*")
        return this->mem[opt][r1 * this->vehicle.max_num + r2];
    else if (opt == "oropt_single")
        return this->mem[opt][r1];
    else if (opt == "oropt_double")
        return this->mem[opt][r1 * this->vehicle.max_num + r2];
    else if (opt == "2exchange")
        return this->mem[opt][r1 * this->vehicle.max_num + r2];
    else
    {
        std::cout << "Unknown opt name: " << opt << std::endl;
        exit(-1); 
    }
}