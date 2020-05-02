#include "operator.h"
// for preventing memory allocation
Move tmp_move;
// map from string to opt;
std::map<std::string, std::function<void(int, int, Solution&, Data&, Move&)>>
small_opt_map = {
    {"2opt", two_opt},
    {"2opt*", two_opt_star},
    {"oropt_single", or_opt_single},
    {"oropt_double", or_opt_double},
    {"2exchange", two_exchange}};
std::map<std::string, std::function<void(Solution &, Data &)>>
destroy_opt_map = {{"random_removal", random_removal},
                   {"related_removal", related_removal}};
std::map<std::string, std::function<void(Solution &, Data &)>>
repair_opt_map = {{"regret_insertion", regret_insertion},
                  {"greedy_insertion", greedy_insertion}};

void maintain_unrouted(int i, int node, int &index, std::vector<std::tuple<int, int>> &unrouted, double &unrouted_d, double &unrouted_p, Data &data)
{
    unrouted[i] = unrouted[index-1];
    index--;
    unrouted_d -= data.node[node].delivery;
    unrouted_p -= data.node[node].pickup;
}

double cal_tc(std::vector<int> &nl, int inserted_node, int pos, double unrouted_d, double unrouted_p, Data &data)
{
    int ori_len = int(nl.size());
    int new_len = ori_len + 1;
    double capacity = data.vehicle.capacity;

    // RDT and RPT for route
    double rdt = 0.0;
    double rpt = 0.0;
    double route_d = 0.0;
    double route_p = 0.0;

    for (int i = 1; i < ori_len; i++)
    {
        route_d += data.node[nl[i]].delivery;
        route_p += data.node[nl[i]].pickup;
    }
    route_d += data.node[inserted_node].delivery;
    route_p += data.node[inserted_node].pickup;

    static std::vector<double> rd(MAX_NODE_IN_ROUTE);
    static std::vector<double> rp(MAX_NODE_IN_ROUTE);
    static std::vector<double> load(MAX_NODE_IN_ROUTE);
    static std::vector<double> cd(MAX_NODE_IN_ROUTE);
    static std::vector<double> cp(MAX_NODE_IN_ROUTE);

    load[0] = route_d;
    cd[0] = 0.0;
    cp[new_len - 1] = 0.0;
    int index = 1;
    bool checked = false;
    int pre_node = nl[0];
    for (int i = 1; i < ori_len; i++)
    {
        int node = nl[i];
        if (i == pos && !checked)
        {
            node = inserted_node;
            i--;
            checked = true;
        }
        load[index] = load[index - 1] - data.node[node].delivery + data.node[node].pickup;
        cd[index] = cd[index - 1] + data.dist[pre_node][node];
        pre_node = node;
        index++;
    }
    checked = false;
    int after_node = nl[ori_len - 1];
    index = new_len-1;
    for (int i = ori_len - 1; i > 0; i--)
    {
        int node = nl[i - 1];
        if (i == pos && !checked)
        {
            node = inserted_node;
            i++;
            checked = true;
        }
        cp[index - 1] = cp[index] + data.dist[node][after_node];
        after_node = node;
        index--;
    }

    rd[0] = capacity - route_d;
    rp[new_len - 2] = capacity - route_p;

    for (int i = 1; i < new_len - 1; i++)
    {
        rd[i] = std::min(rd[i - 1], capacity - load[i]);
        rp[new_len - 2 - i] = std::min(rp[new_len - 1 - i], capacity - load[new_len - 2 - i]);
    }

    double rdt_u = 0.0;
    double rdt_d = 0.0;
    double rpt_u = 0.0;
    double rpt_d = 0.0;
    for (int i = 0; i < new_len - 1; i++)
    {
        rdt_u += rd[i] * cd[i + 1];
        rdt_d += cd[i + 1];
        rpt_u += rp[i] * cp[i];
        rpt_d += cp[i];
    }
    rdt = rdt_u / rdt_d;
    rpt = rpt_u / rpt_d;

    // TC
    double tc = (unrouted_d / data.all_delivery) * (1 - rdt / capacity) + (unrouted_p / data.all_pickup) * (1 - rpt / capacity);
    return tc;
}

double criterion(Route &r, Data &data, int node, int pos, double unrouted_d, double unrouted_p)
{
    std::vector<int> &nl = r.node_list;
    // TD
    int pre = nl[pos-1];
    int suc = nl[pos];
    double td = data.dist[pre][node] + data.dist[node][suc] - data.dist[pre][suc];
    if (data.n_insert == TD) return td;

    // TC
    // std::vector<int> tmp_nl = r.node_list;
    // tmp_nl.insert(tmp_nl.begin() + pos, node);
    double tc = cal_tc(r.node_list, node, pos, unrouted_d, unrouted_p, data);

    // RS
    double rs = data.dist[data.DC][node] + data.dist[node][data.DC];

    // RCRS
    double rcrs = td + std::get<0>(data.lambda_gamma) * tc * (2 * data.max_dist - data.min_dist) -
                  std::get<1>(data.lambda_gamma) * rs;

    return rcrs;
}

bool cal_score(std::vector<bool> &feasible_pos, std::vector<std::tuple<int, int>> &unrouted, std::vector<double> &score, int index, Route &r, double unrouted_d, double unrouted_p, Data &data)
{
    /* calculate insertion criterion for each node in unrouted, return
    false if no feasible insertion exists */
    if (index == 0) return false;
    int r_len = int(r.node_list.size());
    // filter all infeasible positions
    int count = 0;
    for (int i = 0; i < index; i++)
    {
        int node = std::get<0>(unrouted[i]);
        for (int pos = 1; pos < r_len; pos++)
        {
            bool flag = false;
            double cost = -1.0;
            chk_nl_node_pos_O_n(r.node_list, node, pos, data, flag, cost);
            if (!flag) feasible_pos[i*MAX_NODE_IN_ROUTE+pos] = false;
            else 
            {
                feasible_pos[i*MAX_NODE_IN_ROUTE+pos] = true;
                count++;
            }
        }
    }
    if (count == 0) return false;

    // insertion criterion
    for (int i = 0; i < index; i++)
    {
        int node = std::get<0>(unrouted[i]);
        double best_score = double(INFINITY);
        int best_pos = -1;
        for (int pos = 1; pos < r_len; pos++)
        {
            if (!feasible_pos[i*MAX_NODE_IN_ROUTE+pos]) continue;
            double utility = criterion(r, data, node, pos, unrouted_d,\
                                       unrouted_p);
            if (utility - best_score < -PRECISION)
            {
                best_score = utility;
                best_pos = pos;
            }
        }
        std::get<1>(unrouted[i]) = best_pos;
        score[i] = best_score;
    }
    return true;
}

void find_unrouted(Solution &s, std::vector<int> &record)
{
    int len = s.len();
    for (int i = 0; i < len; i++)
    {
        Route &r = s.get(i);
        for (auto node : r.node_list)
        {
            record[node] = 1;
        }
    }
}

void update_nodes_pm_cost(Solution &s, std::vector<std::vector<int>> &nodes_pm_pos, std::vector<std::vector<double>> &nodes_pm_cost, std::vector<int> &unrouted_nodes, Data &data)
{
    for (int i = 0; i < int(unrouted_nodes.size()); i++)
    {
        auto &single_node_pm_pos = nodes_pm_pos[i];
        auto &single_node_pm_cost = nodes_pm_cost[i];
        int node = unrouted_nodes[i];

        // build a new route with node {DC, node, DC}
        std::vector<int> tmp_nl = make_tmp_nl(data);
        bool flag = false;
        double cost = -1.0;
        chk_nl_node_pos_O_n(tmp_nl, node, 1, data, flag, cost);
        if (!flag)
        {
            printf("Error: Detect not feasible 1-customer route: ");
            for (auto &node : tmp_nl)
            {
                std::cout << node;
            }
            exit(-1);
        }
        single_node_pm_pos[0] = 1;
        single_node_pm_cost[0] = cost;

        // insert into existing routes, one by one
        for (int r_index = 0; r_index < s.len(); r_index++)
        {
            Route &r = s.get(r_index);
            double ori_cost = r.cal_cost(data);
            double best_incur_cost = double(INFINITY);
            int best_pos = -1;
            for (int pos = 1; pos < int(r.node_list.size()); pos++)
            {
                bool flag = false;
                double cost = -1.0;
                chk_nl_node_pos_O_n(r.node_list, node, pos, data, flag, cost);
                if (flag)
                {
                    double incur_cost = cost - ori_cost;
                    if (incur_cost - best_incur_cost < -PRECISION)
                    {
                        best_incur_cost = incur_cost;
                        best_pos = pos;
                    }
                }
            }
            single_node_pm_pos[r_index + 1] = best_pos;
            single_node_pm_cost[r_index + 1] = best_incur_cost;
        }
    }
}

void update_single_node_pm_cost(Solution &s, std::vector<std::vector<int>> &nodes_pm_pos, std::vector<std::vector<double>> &nodes_pm_cost, std::vector<int> &unrouted_nodes, int changed_r_index, std::vector<bool> &inserted, Data &data)
{
    Route &r = s.get(changed_r_index);
    double ori_cost = r.cal_cost(data);

    for (int i = 0; i < int(unrouted_nodes.size()); i++)
    {
        if (inserted[i])
            continue;
        auto &single_node_pm_pos = nodes_pm_pos[i];
        auto &single_node_pm_cost = nodes_pm_cost[i];
        int node = unrouted_nodes[i];
        double best_incur_cost = double(INFINITY);
        int best_pos = -1;

        for (int pos = 1; pos < int(r.node_list.size()); pos++)
        {
            bool flag = false;
            double cost = -1.0;
            chk_nl_node_pos_O_n(r.node_list, node, pos, data, flag, cost);
            if (flag)
            {
                double incur_cost = cost - ori_cost;
                if (incur_cost - best_incur_cost < -PRECISION)
                {
                    best_incur_cost = incur_cost;
                    best_pos = pos;
                }
            }
        }
        single_node_pm_pos[changed_r_index + 1] = best_pos;
        single_node_pm_cost[changed_r_index + 1] = best_incur_cost;
    }
}

void greedy_insertion(Solution &s, Data &data)
{
    int num_cus = data.customer_num;
    // find all unrouted nodes
    std::vector<int> record(num_cus + 1, 0);
    find_unrouted(s, record);
    std::vector<int> unrouted_nodes;
    unrouted_nodes.reserve(data.customer_num);

    for (int i = 0; i < num_cus + 1; i++)
    {
        if (i == data.DC)
            continue;
        if (record[i] == 0)
            unrouted_nodes.push_back(i);
    }
    int unroute_len = int(unrouted_nodes.size());
    std::vector<bool> inserted(unroute_len, false);
    // <pos, best incur_cost in the route>
    std::vector<int> single_node_pm_pos(data.vehicle.max_num + 1);
    std::vector<double> single_node_pm_cost(data.vehicle.max_num + 1);
    std::vector<std::vector<int>> nodes_pm_pos;
    std::vector<std::vector<double>> nodes_pm_cost;
    for (int i = 0; i < unroute_len; i++)
    {
        nodes_pm_pos.push_back(single_node_pm_pos);
        nodes_pm_cost.push_back(single_node_pm_cost);
    }
    update_nodes_pm_cost(s, nodes_pm_pos, nodes_pm_cost, unrouted_nodes, data);

    while (unroute_len > 0)
    {
        // find the one with the min insertion cost
        double min_cost = double(INFINITY);
        int best_node_index = -1;
        int best_route_index = -2;
        for (int i = 0; i < int(unrouted_nodes.size()); i++)
        {
            if (inserted[i]) continue;
            auto &single_node_pm_pos = nodes_pm_pos[i];
            auto &single_node_pm_cost = nodes_pm_cost[i];
            // find the best and the second best insertion cost in single_node_pm
            double best_incur_cost = single_node_pm_cost[0];
            int best_r = -1;
            for (int r_index = 0; r_index < s.len(); r_index++)
            {
                if (single_node_pm_cost[r_index + 1] - best_incur_cost < -PRECISION)
                {
                    best_incur_cost = single_node_pm_cost[r_index + 1];
                    best_r = r_index;
                }
            }
            // second_incur_cost = INFINITY, no feasible insertion into s
            // except building a new route. In this case set regret = 0, giving
            // it the minimum priority for insertion
            if (best_incur_cost - min_cost < -PRECISION)
            {
                min_cost = best_incur_cost;
                best_node_index = i;
                best_route_index = best_r;
            }
        }
        if (min_cost == double(INFINITY))
        {
            std::cout << "Error: Min insertion cost -INFINITY, this should not happen\n";
            exit(-1);
        }
        // insert
        auto &single_node_pm_pos = nodes_pm_pos[best_node_index];
        auto &single_node_pm_cost = nodes_pm_cost[best_node_index];
        int node = unrouted_nodes[best_node_index];

        if (best_route_index == -1)
        {
            // build a new route
            Route r(data);
            r.node_list.insert(r.node_list.begin() + 1, node);
            r.update(data);
            s.append(r);
        }
        else
        {
            Route &r = s.get(best_route_index);
            r.node_list.insert(r.node_list.begin() + single_node_pm_pos[best_route_index + 1], node);
            r.update(data);
        }
        // flag inserted node in unrouted
        inserted[best_node_index] = true;
        unroute_len--;

        // update regret score in nodes_pm
        // find the changed route
        int changed_r_index = best_route_index;
        if (changed_r_index == -1)
            changed_r_index = s.len() - 1;
        update_single_node_pm_cost(s, nodes_pm_pos, nodes_pm_cost, unrouted_nodes, changed_r_index, inserted, data);
    }
    s.cal_cost(data);
}

void regret_insertion(Solution &s, Data &data)
{
    int num_cus = data.customer_num;
    // find all unrouted nodes
    std::vector<int> record(num_cus + 1, 0);
    find_unrouted(s, record);
    std::vector<int> unrouted_nodes;
    unrouted_nodes.reserve(data.customer_num);

    for (int i = 0; i < num_cus + 1; i++)
    {
        if (i == data.DC) continue;
        if (record[i] == 0)
            unrouted_nodes.push_back(i);
    }
    int unroute_len = int(unrouted_nodes.size());
    std::vector<bool> inserted(unroute_len, false);
    // <pos, best incur_cost in the route>
    std::vector<int> single_node_pm_pos(data.vehicle.max_num+1);
    std::vector<double> single_node_pm_cost(data.vehicle.max_num+1);

    std::vector<std::vector<int>> nodes_pm_pos;
    std::vector<std::vector<double>> nodes_pm_cost;
    for (int i = 0; i < unroute_len; i++)
    {
        nodes_pm_pos.push_back(single_node_pm_pos);
        nodes_pm_cost.push_back(single_node_pm_cost);
    }
    update_nodes_pm_cost(s, nodes_pm_pos, nodes_pm_cost, unrouted_nodes, data);

    while (unroute_len > 0)
    {
        // find the one with the max regret value
        double max_regret = -double(INFINITY);
        int best_node_index = -1;
        int best_route_index = -2;
        for (int i = 0; i < int(unrouted_nodes.size()); i++) 
        {
            if (inserted[i])
                continue;
            auto &single_node_pm_pos = nodes_pm_pos[i];
            auto &single_node_pm_cost = nodes_pm_cost[i];
            // find the best and the second best insertion cost in single_node_pm
            double best_incur_cost = single_node_pm_cost[0];
            int best_r = -1;
            double second_incur_cost = double(INFINITY);
            for (int r_index = 0; r_index < s.len(); r_index++)
            {
                if (single_node_pm_cost[r_index+1] - best_incur_cost < -PRECISION)
                {
                    second_incur_cost = best_incur_cost;
                    best_incur_cost = single_node_pm_cost[r_index+1];
                    best_r = r_index;
                }
                else if (single_node_pm_cost[r_index+1] - second_incur_cost < -PRECISION)
                {
                    second_incur_cost = single_node_pm_cost[r_index+1];
                }
            }
            // second_incur_cost = INFINITY, no feasible insertion into s
            // except building a new route. In this case set regret = 0, giving
            // it the minimum priority for insertion
            double regret = second_incur_cost - best_incur_cost;
            if (second_incur_cost == double(INFINITY)) regret = 0.0;
            if (regret - max_regret > PRECISION)
            {
                max_regret = regret;
                best_node_index = i;
                best_route_index = best_r;
            }
        }
        if (max_regret == -double(INFINITY))
        {
            std::cout << "Error: Max regret -INFINITY, this should not happen\n";
            exit(-1);
        }
        // all nodes have one feasible insertion: building new route
        // find the one with minimum best_incur_cost
        if (max_regret == 0.0)
        {
            max_regret = double(INFINITY);
            for (int i = 0; i < int(unrouted_nodes.size()); i++)
            {
                if (inserted[i])
                    continue;
                auto &single_node_pm_pos = nodes_pm_pos[i];
                auto &single_node_pm_cost = nodes_pm_cost[i];
                if (single_node_pm_cost[0] - max_regret < -PRECISION)
                {
                    max_regret = single_node_pm_cost[0];
                    best_node_index = i;
                    best_route_index = -1;
                }
            }
        }
        // insert
        auto &single_node_pm_pos = nodes_pm_pos[best_node_index];
        auto &single_node_pm_cost = nodes_pm_cost[best_node_index];
        int node = unrouted_nodes[best_node_index];

        if (best_route_index == -1)
        {
            // build a new route
            Route r(data);
            r.node_list.insert(r.node_list.begin()+1, node);
            r.update(data);
            s.append(r);
        }
        else
        {
            Route &r = s.get(best_route_index);
            r.node_list.insert(r.node_list.begin() + single_node_pm_pos[best_route_index+1], node);
            r.update(data);
        }
        // flag inserted node in unrouted
        inserted[best_node_index] = true;
        unroute_len--;

        // update regret score in nodes_pm
        // find the changed route
        int changed_r_index = best_route_index;
        if (changed_r_index == -1) changed_r_index = s.len() - 1;
        update_single_node_pm_cost(s, nodes_pm_pos, nodes_pm_cost, unrouted_nodes, changed_r_index, inserted, data);
    }
    s.cal_cost(data);
}

void new_route_insertion(Solution &s, Data &data, int initial_node)
{
    std::vector<double> score(MAX_POINT);
    std::vector<int> score_argrank(MAX_POINT);
    std::vector<int> ties(MAX_POINT);
    std::vector<bool> feasible_pos(MAX_NODE_IN_ROUTE*MAX_POINT, false);

    double unrouted_d = data.all_delivery;
    double unrouted_p = data.all_pickup;

    int num_cus = data.customer_num;
    // find all unrouted customers
    // index points to the last position
    std::vector<int> record(num_cus + 1, 0);
    find_unrouted(s, record);

    // first dim: node, second dim: position
    std::vector<std::tuple<int, int>> unrouted(data.customer_num);
    int index = 0;
    for (int i = 0; i < num_cus + 1; i++)
    {
        if (i != data.DC && record[i] == 0)
        {
            std::get<0>(unrouted[index]) = i;
            index++;
        }
        else if (i != data.DC && record[i] == 1)
        {
            unrouted_d -= data.node[i].delivery;
            unrouted_p -= data.node[i].pickup;
        }
    }

    while (index > 0)
    {
        Route r(data);
        int selected = -1;
        int first_node = -1;
        if (index == data.customer_num) // the first time into this loop
        {
            first_node = initial_node;
            for (int i = 0; i < index; i++)
            {
                if (std::get<0>(unrouted[i]) == first_node)
                {
                    selected = i;
                    break;
                }
            }
        }
        else
        {
            selected = randint(0, index-1, data.rng);
            first_node = std::get<0>(unrouted[selected]);
        }
        maintain_unrouted(selected, first_node, index, unrouted, unrouted_d, unrouted_p, data);
        r.node_list.insert(r.node_list.begin() + 1, first_node);

        while (cal_score(feasible_pos, unrouted, score, index, r,
                         unrouted_d, unrouted_p, data))
        {
            argsort(score, score_argrank, index);
            double best_score = score[score_argrank[0]];
            ties[0] = score_argrank[0];
            int selected;
            int i = 1;
            for (; i < index; i++)
            {
                if (std::abs(best_score - score[score_argrank[i]]) < -PRECISION)
                    ties[i] = score_argrank[i];
                else
                    break;
            }
            if (i > 1) selected = ties[randint(0, i - 1, data.rng)];
            else selected = ties[0];
            // insert
            int node = std::get<0>(unrouted[selected]);
            int pos = std::get<1>(unrouted[selected]);

            r.node_list.insert(r.node_list.begin() + pos, node);
            maintain_unrouted(selected, node, index, unrouted, unrouted_d, unrouted_p, data);
        }
        r.update(data);
        s.append(r);
    }
    s.cal_cost(data);
}

void new_route_insertion(Solution &s, Data &data)
{
    int num_cus = data.customer_num;
    // find all unrouted customers
    // index points to the last position
    std::vector<int> record(num_cus + 1, 0);
    find_unrouted(s, record);

    // first dim: node, second dim: position
    std::vector<std::tuple<int, int>> unrouted(data.customer_num);
    int index = 0;
    for (int i = 0; i <= num_cus; i++)
    {
        if (i != data.DC && record[i] == 0)
        {
            std::get<0>(unrouted[index]) = i;
            index++;
        }
    }
    if (index == 0) return;
    if (data.ksize == 1)
    {
        int selected = randint(0, index - 1, data.rng);
        int node = std::get<0>(unrouted[selected]);
        new_route_insertion(s, data, node);
    }
    else
    {
        double best_cost = double(INFINITY);
        Solution best_s(data);
        for (int i = 0; i < std::min(data.ksize, index); i++)
        {
            int selected = randint(0, index-1-i, data.rng);
            int node = std::get<0>(unrouted[selected]);
            Solution tmp_s(data);
            tmp_s = s;
            new_route_insertion(tmp_s, data, node);
            if (tmp_s.cost - best_cost < -PRECISION)
            {
                best_s = tmp_s;
                best_cost = tmp_s.cost;
            }
            unrouted[selected] = unrouted[index-1-i];
            // printf("Solution %d, Cost %.4f\n", i, tmp_s.cost);
        }
        s = best_s;
    }
}

void two_opt(int r1, int r2, Solution &s, Data &data, Move &m)
{
    // inverse a 2-sequence in a route
    m.delta_cost = double(INFINITY);

    Route &r = s.get(r1);
    auto &n_l = r.node_list;
    int len = int(n_l.size());
    if (len < 4) return;
    for (int start = 1; start <= len - 3; start++)
    {
        if (data.pruning &&
            (!data.pm[n_l[start-1]][n_l[start+1]] ||
                !data.pm[n_l[start+1]][n_l[start]] ||
                !data.pm[n_l[start]][n_l[start+2]]))
            continue;
        if (r.gat(start+1, start).num_cus == INFEASIBLE)
            continue;
        tmp_move.r_indice[0] = r1;
        tmp_move.r_indice[1] = -2;
        tmp_move.len_1 = 3;
        tmp_move.seqList_1[0] = {r1, 0, start-1};
        tmp_move.seqList_1[1] = {r1, start+1, start};
        tmp_move.seqList_1[2] = {r1, start+2, len-1};
        tmp_move.len_2 = 0;
        if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
        {
            m = tmp_move;
        }
    }
}

void two_opt_star(int r1, int r2, Solution &s, Data &data, Move &m)
{
    m.delta_cost = double(INFINITY);

    Route &r_1 = s.get(r1);
    auto &n_l_1 = r_1.node_list;
    int len_1 = int(n_l_1.size());

    Route &r_2 = s.get(r2);
    auto &n_l_2 = r_2.node_list;
    int len_2 = int(n_l_2.size());
    for (int pos_1 = 1; pos_1 <= len_1 - 1; pos_1++)
    {
        for (int pos_2 = 1; pos_2 <= len_2 - 1; pos_2++)
        {
            if ((pos_1 == 1 && pos_2 == 1) || (pos_1 == len_1-1 && pos_2 == len_2-1))
                continue;
            if (data.pruning &&
                (!data.pm[n_l_1[pos_1-1]] [n_l_2[pos_2]] ||\
                    !data.pm[n_l_2[pos_2-1]][n_l_1[pos_1]]))
                    continue;
            tmp_move.r_indice[0] = r1;
            tmp_move.r_indice[1] = r2;
            tmp_move.len_1 = 2;
            tmp_move.seqList_1[0] = {r1, 0, pos_1-1};
            tmp_move.seqList_1[1] = {r2, pos_2, len_2-1};
            tmp_move.len_2 = 2;
            tmp_move.seqList_2[0] = {r2, 0, pos_2-1};
            tmp_move.seqList_2[1] = {r1, pos_1, len_1-1};
            if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
            {
                m = tmp_move;
            }
        }
    }
}

void or_opt_single(int r1, int r2, Solution &s, Data &data, Move &m)
{
    m.delta_cost = double(INFINITY);
    Route &r = s.get(r1);
    auto &n_l = r.node_list;
    int len = int(n_l.size());
    for (int start = 1; start <= len - 2; start++)
    {
        for (int seq_len = 1; seq_len <= data.or_opt_len; seq_len++)
        {
            int end = start + seq_len - 1;
            if (end >= len - 1) continue;
            if (data.pruning &&
                (!data.pm[n_l[start-1]][n_l[end+1]]))
                continue;
            // relocate to the same route
            for (int pos = 1; pos <= start - 1; pos++)
            {
                if (data.pruning &&
                    (!data.pm[n_l[pos-1]][n_l[start]] ||\
                        !data.pm[n_l[end]][n_l[pos]]))
                        continue;
                tmp_move.r_indice[0] = r1;
                tmp_move.r_indice[1] = -2;
                tmp_move.len_1 = 4;
                tmp_move.seqList_1[0] = {r1, 0, pos-1};
                tmp_move.seqList_1[1] = {r1, start, end};
                tmp_move.seqList_1[2] = {r1, pos, start-1};
                tmp_move.seqList_1[3] = {r1, end+1, len-1};
                tmp_move.len_2 = 0;
                if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
                {
                    m = tmp_move;
                }
            }
            for (int pos = end + 2; pos <= len - 1; pos++)
            {
                if (data.pruning &&
                    (!data.pm[n_l[pos-1]][n_l[start]] ||
                        !data.pm[n_l[end]][n_l[pos]]))
                    continue;
                tmp_move.r_indice[0] = r1;
                tmp_move.r_indice[1] = -2;
                tmp_move.len_1 = 4;
                tmp_move.seqList_1[0] = {r1, 0, start-1};
                tmp_move.seqList_1[1] = {r1, end+1, pos-1};
                tmp_move.seqList_1[2] = {r1, start, end};
                tmp_move.seqList_1[3] = {r1, pos, len-1};
                tmp_move.len_2 = 0;
                if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
                {
                    m = tmp_move;
                }
            }
            // relocate to a new route
            tmp_move.r_indice[0] = r1;
            tmp_move.r_indice[1] = -1;
            tmp_move.len_1 = 2;
            tmp_move.seqList_1[0] = {r1, 0, start - 1};
            tmp_move.seqList_1[1] = {r1, end + 1, len - 1};
            tmp_move.len_2 = 3;
            tmp_move.seqList_2[0] = {-1, data.DC, data.DC};
            tmp_move.seqList_2[1] = {r1, start, end};
            tmp_move.seqList_2[2] = {-1, data.DC, data.DC};
            if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
            {
                m = tmp_move;
            }
        }
    }
}

void or_opt_double(int r_index_1, int r_index_2, Solution &s, Data &data, Move &m)
{
    m.delta_cost = double(INFINITY);
    // relocate a sequence
    for (int i = 0; i < 2; i++)
    {
        int r1, r2;
        if (i == 0)
        {
            r1 = r_index_1;
            r2 = r_index_2;
        }
        else if (i == 1)
        {
            r1 = r_index_2;
            r2 = r_index_1;
        }
        Route &r = s.get(r1);
        auto &n_l = r.node_list;
        int len = int(n_l.size());
        for (int start = 1; start <= len - 2; start++)
        {
            for (int seq_len = 1; seq_len <= data.or_opt_len; seq_len++)
            {
                int end = start + seq_len - 1;
                if (end >= len - 1)
                    continue;
                if (data.pruning &&
                    (!data.pm[n_l[start - 1]][n_l[end + 1]]))
                    continue;
                // relocate to other route

                if (r1 == r2)
                    continue;
                Route &r_2 = s.get(r2);
                auto &n_l_2 = r_2.node_list;
                int len_2 = int(n_l_2.size());
                for (int pos = 1; pos <= len_2 - 1; pos++)
                {
                    if (data.pruning &&
                        (!data.pm[n_l_2[pos - 1]][n_l[start]] ||
                         !data.pm[n_l[end]][n_l_2[pos]]))
                        continue;
                    tmp_move.r_indice[0] = r1;
                    tmp_move.r_indice[1] = r2;
                    tmp_move.len_1 = 2;
                    tmp_move.seqList_1[0] = {r1, 0, start - 1};
                    tmp_move.seqList_1[1] = {r1, end + 1, len - 1};
                    tmp_move.len_2 = 3;
                    tmp_move.seqList_2[0] = {r2, 0, pos - 1};
                    tmp_move.seqList_2[1] = {r1, start, end};
                    tmp_move.seqList_2[2] = {r2, pos, len_2 - 1};
                    if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
                    {
                        m = tmp_move;
                    }
                }
            }
        }
    }
}

void two_exchange(int r1, int r2, Solution &s, Data &data, Move &m)
{
    m.delta_cost = double(INFINITY);
    // exchange two sequences with seqs
    Route &r_1 = s.get(r1);
    auto &n_l_1 = r_1.node_list;
    int len_1 = int(n_l_1.size());

    Route &r_2 = s.get(r2);
    auto &n_l_2 = r_2.node_list;
    int len_2 = int(n_l_2.size());
    for (int start_1 = 1; start_1 <= len_1 - 2; start_1++)
    {
        for (int seq_len_1 = 1; seq_len_1 <= data.exchange_len; seq_len_1++)
        {
            int end_1 = start_1 + seq_len_1 - 1;
            if (end_1 >= len_1 - 1) continue;
            for (int start_2 = 1; start_2 <= len_2 - 2; start_2++)
            {
                for (int seq_len_2 = 1; seq_len_2 <= data.exchange_len; seq_len_2++)
                {
                    int end_2 = start_2 + seq_len_2 - 1;
                    if (end_2 >= len_2 - 1) continue;
                    if (data.pruning &&
                        (!data.pm[n_l_1[start_1-1]][n_l_2[start_2]] ||\
                            !data.pm[n_l_2[end_2]][n_l_1[end_1+1]] ||\
                            !data.pm[n_l_2[start_2-1]][n_l_1[start_1]] ||\
                            !data.pm[n_l_1[end_1]][n_l_2[end_2+1]]))
                        continue;
                    tmp_move.r_indice[0] = r1;
                    tmp_move.r_indice[1] = r2;
                    tmp_move.len_1 = 3;
                    tmp_move.seqList_1[0] = {r1, 0, start_1-1};
                    tmp_move.seqList_1[1] = {r2, start_2, end_2};
                    tmp_move.seqList_1[2] = {r1, end_1+1, len_1-1};
                    tmp_move.len_2 = 3;
                    tmp_move.seqList_2[0] = {r2, 0, start_2-1};
                    tmp_move.seqList_2[1] = {r1, start_1, end_1};
                    tmp_move.seqList_2[2] = {r2, end_2+1, len_2-1};
                    if (eval_move(s, tmp_move, data) && tmp_move.delta_cost < m.delta_cost)
                    {
                        m = tmp_move;
                    }
                }
            }
        }
    }
}

void removal_from_s(Solution &s, std::vector<int>& flag)
{
    int s_len = s.len();
    for (int r_index = 0; r_index < s_len; r_index++)
    {
        Route &r = s.get(r_index);
        auto &n_l = r.node_list;
        int len = int(n_l.size());
        for (int i = 0; i < len; i++)
        {
            if (flag[n_l[i]] == 1)
            {
                n_l.erase(n_l.begin() + i);
                i--;
                len--;
            }
        }
    }
}

void related_removal(Solution &s, Data &data)
{
    int selected = randint(0, data.customer_num, data.rng);
    while(selected == data.DC)
        selected = randint(0, data.customer_num, data.rng);
    std::vector<int> flag(data.customer_num+1, 0);
    std::vector<int> selected_cus;
    selected_cus.reserve(data.customer_num);
    flag[selected] = 1;
    selected_cus.push_back(selected);
    int total_remove = round(data.customer_num * \
                             rand(data.destroy_ratio_l, data.destroy_ratio_u, data.rng));
    int already_remove = 1;
    while (already_remove < total_remove)
    {
        int ref_cus = selected_cus[randint(0, int(selected_cus.size())-1, data.rng)];
        auto &argrank = data.rm_argrank[ref_cus];
        std::vector<int> best_two;
        best_two.reserve(2);
        for (int i = 0; i < data.customer_num - 1; i++)
        {
            if (flag[argrank[i]] == 1) continue;
            best_two.push_back(argrank[i]);
            if (int(best_two.size())== 2) break;
        }
        if (int(best_two.size()) != 2)
        {
            printf("Cound not find not 2 inserted customers in related removal\n");
            exit(-1);
        }
        // roulette selection
        int selected = -1;
        double prob = data.rm[ref_cus][best_two[1]] / (data.rm[ref_cus][best_two[0]] + data.rm[ref_cus][best_two[1]]);
        if (rand(0, 1, data.rng) < prob)
            selected = best_two[0];
        else
            selected = best_two[1];
        flag[selected] = 1;
        selected_cus.push_back(selected);
        already_remove ++;
    }
    removal_from_s(s, flag);
    s.update(data);
}

void random_removal(Solution &s, Data &data)
{
    // randomly delete customers from s
    std::vector<int> customers(data.customer_num);
    std::vector<int> indice(data.customer_num+1, 0);
    int count = 0;
    for(int i = 0; i < data.customer_num + 1; i++)
    {
        if (i == data.DC) continue;
        customers[count] = i;
        count++;
    }
    shuffle(customers.begin(), customers.end(), data.rng);
    int boundray = int(round(double(data.customer_num) *\
                       rand(data.destroy_ratio_l, data.destroy_ratio_u, data.rng)));
    for (int i = 0; i < boundray + 1; i++) {indice[customers[i]] = 1;}
    removal_from_s(s, indice);
    s.update(data);
}

std::vector<int> apply_move(Solution &s, Move &m, Data data)
{
    std::vector<int> r_indice;
    r_indice.push_back(m.r_indice[0]);
    if (m.r_indice[1] != -2)
        r_indice.push_back(m.r_indice[1]);

    // handle the first route
    if (r_indice[0] == -2 || r_indice[0] == -1)
    {
        printf("Error: detect -1 or -2 in r_indice[0] in move\n");
        exit(-1);
    }
    Route &r = s.get(r_indice[0]);
    std::vector<int> target_n_l;
    target_n_l.reserve(MAX_NODE_IN_ROUTE);

    for (int i = 0; i < m.len_1; i++)
    {
        auto &seq = m.seqList_1[i];
        auto &source_n_l = s.get(seq.r_index).node_list;
        if (seq.start_point <= seq.end_point)
        {
            for (int index = seq.start_point; index <= seq.end_point; index++)
                {target_n_l.push_back(source_n_l[index]);}
        }
        else
        {
            for (int index = seq.start_point; index >= seq.end_point; index--)
            {
                target_n_l.push_back(source_n_l[index]);
            }
        }
    }

    // handle the second route
    if (int(r_indice.size()) == 2)
    {
        std::vector<int> target_n_l_2;
        target_n_l_2.reserve(MAX_NODE_IN_ROUTE);

        for (int i = 0; i < m.len_2; i++)
        {
            auto &seq = m.seqList_2[i];
            if (seq.r_index == -1)
            {
                target_n_l_2.push_back(data.DC);
                continue;
            }
            auto &source_n_l = s.get(seq.r_index).node_list;
            if (seq.start_point <= seq.end_point)
            {
                for (int index = seq.start_point; index <= seq.end_point; index++)
                {
                    target_n_l_2.push_back(source_n_l[index]);
                }
            }
            else
            {
                for (int index = seq.start_point; index >= seq.end_point; index--)
                {
                    target_n_l_2.push_back(source_n_l[index]);
                }
            }
        }
        if (r_indice[1] == -1)
        {
            Route r(data);
            r.node_list = target_n_l_2;
            r.update(data);
            s.append(r);
            r_indice[1] = s.len() - 1;
        }
        else
        {
            Route &r = s.get(r_indice[1]);
            r.node_list = target_n_l_2;
            r.update(data);
        }
    }
    // set node list only at the end
    r.node_list = target_n_l;
    r.update(data);

    s.local_update(r_indice);
    return r_indice;
}

void output_move(Move &m)
{
    std::cout << "r_indice: " << m.r_indice[0] << m.r_indice[1] << std::endl;
    std::cout << "len_1: " << m.len_1 << std::endl;
    for (int i = 0; i < m.len_1; i++)
    {
        std::cout << m.seqList_1[i].r_index << m.seqList_1[i].start_point << m.seqList_1[i].end_point << std::endl;
    }
    std::cout << "len_2: " << m.len_2 << std::endl;
    for (int i = 0; i < m.len_2; i++)
    {
        std::cout << m.seqList_2[i].r_index << m.seqList_2[i].start_point << m.seqList_2[i].end_point << std::endl;
    }
}

void snippet(int r1, int r2, std::string &opt, Solution &s, Data &data, Move &target)
{
    auto &m = data.get_mem(opt, r1, r2);
    small_opt_map[opt](r1, r2, s, data, m);
    if (m.delta_cost - target.delta_cost < -PRECISION)
        target = m;
}

void find_local_optima(Solution &s, Data &data)
{
    // record the best solution in the neighborhood of each small opt
    std::vector<Move> move_list(int(data.small_opts.size()));

    // find the best move for all sub-neighbors of each opt
    int len = int(s.len());
    for (int i = 0; i < int(move_list.size()); i++)
    {
        move_list[i].delta_cost = double(INFINITY);
        auto &opt = data.small_opts[i];
        if (opt == "2opt" || opt == "oropt_single")
        {
            for (int r = 0; r < len; r++)
            {
                snippet(r, -1, opt, s, data, move_list[i]);
            }
        }
        else if (opt == "2opt*" || opt == "2exchange" || opt == "oropt_double")
        {
            for (int r1 = 0; r1 < len; r1++)
            {
                for (int r2 = r1 + 1; r2 < len; r2++)
                {
                    snippet(r1, r2, opt, s, data, move_list[i]);
                }
            }
        }
        else
        {
            std::cout << "Unknown opt: " << opt;
            exit(-1);
        }
    }

    // double acc_delta_cost = 0;
    std::vector<int> tour_id_array;
    while (true)
    {
        int best_index = -1;
        double min_delta_cost = double(INFINITY);
        for (int i = 0; i < int(move_list.size()); i++)
        {
            if (move_list[i].delta_cost - min_delta_cost < -PRECISION)
            {
                best_index = i;
                min_delta_cost = move_list[i].delta_cost;
            }
        }
        if (min_delta_cost < -PRECISION)
        {
            // apply move
            // acc_delta_cost += min_delta_cost;
            tour_id_array = apply_move(s, move_list[best_index], data);
            // printf("Acc_delta_cost %f.\nBest move index %d\n", acc_delta_cost, best_index);
            // if (!s.check(data)) output_move(move_list[best_index]);

            // update move_list
            int len = s.len();
            for (int i = 0; i < int(move_list.size()); i++)
            {
                move_list[i].delta_cost = double(INFINITY);
                auto &opt = data.small_opts[i];
                if (opt == "2opt" || opt == "oropt_single")
                {
                    for (auto &r : tour_id_array)
                    {
                        if (r >= len) continue;
                        snippet(r, -1, opt, s, data, move_list[i]);
                    }
                    for (int r = 0; r < len; r++)
                    {
                        if (data.get_mem(opt, r, -1).delta_cost - move_list[i].delta_cost < -PRECISION)
                            move_list[i] = data.get_mem(opt, r, -1);
                    }
                }
                else if (opt == "2opt*" || opt == "2exchange" || opt == "oropt_double")
                {
                    for (auto &r : tour_id_array)
                    {
                        if (r >= len) continue;
                        for (int r1 = 0; r1 < r; r1++)
                        {
                            snippet(r1, r, opt, s, data, move_list[i]);
                        }
                        for (int r1 = r+1; r1 < len; r1++)
                        {
                            snippet(r, r1, opt, s, data, move_list[i]);
                        }
                    }
                    for (int r1 = 0; r1 < len; r1++)
                    {
                        for (int r2 = r1 + 1; r2 < len; r2++)
                        {
                            if (data.get_mem(opt, r1, r2).delta_cost - move_list[i].delta_cost < -PRECISION)
                                move_list[i] = data.get_mem(opt, r1, r2);
                        }
                    }
                }
                else
                {
                    std::cout << "Unknown opt: " << opt;
                    exit(-1);
                }
            }
        }
        else break;
    }
}

void do_local_search(Solution &s, Data &data)
{
    if (int(data.small_opts.size() == 0))
    {
        printf("No small stepsize operator used, directly return.\n");
        return;
    }
    static int tmp_solution_num = int(data.destroy_opts.size()) * int(data.repair_opts.size());
    if (data.rd_removal_insertion)
        tmp_solution_num = 1;
    static std::vector<Solution> s_vector(tmp_solution_num);
    find_local_optima(s, data);
    s.cal_cost(data);

    int no_improve = 0;
    while (no_improve < data.escape_local_optima)
    {
        for (int i = 0; i < tmp_solution_num; i++)
            s_vector[i] = s;
        perturb(s_vector, data);
        int best_index = -1;
        double best_cost = double(INFINITY);
        for (int i = 0; i < tmp_solution_num; i++)
        {
            find_local_optima(s_vector[i], data);
            s_vector[i].cal_cost(data);
            if (s_vector[i].cost - best_cost < -PRECISION)
            {
                best_index = i;
                best_cost = s_vector[i].cost;
            }
        }
        if (s_vector[best_index].cost - s.cost < -PRECISION)
        {
            s = s_vector[best_index];
            no_improve = 0;
        }
        else
        {
            no_improve++;
            // if (no_improve == data.escape_local_optima)
            //     printf("Have Tried %d peturbations, return.\n", no_improve);
        }
    }
}

void perturb(std::vector<Solution> &s_vector, Data &data)
{
    int count = 0;
    if (data.rd_removal_insertion)
    {
        int i = randint(0, int(data.destroy_opts.size())-1, data.rng);
        int j = randint(0, int(data.repair_opts.size())-1, data.rng);
        destroy_opt_map[data.destroy_opts[i]](s_vector[0], data);
        repair_opt_map[data.repair_opts[j]](s_vector[0], data);
        return;
    }
    for (int i = 0; i < int(data.destroy_opts.size()); i++)
    {
        for (int j = 0; j < int(data.repair_opts.size()); j++)
        {
            destroy_opt_map[data.destroy_opts[i]](s_vector[count], data);
            repair_opt_map[data.repair_opts[j]](s_vector[count], data);
            // s_vector[count].check(data);
            count++;
        }
    }
}