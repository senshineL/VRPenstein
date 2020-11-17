# include "eval.h"

extern long call_count_move_eval;
extern double mean_duration_move_eval;
extern double mean_route_len;

using namespace std::chrono;

void chk_nl_node_pos_O_n(std::vector<int> &nl, int inserted_node, int pos, Data &data, bool &flag, double &cost)
{
    int len = int(nl.size());
    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl)
    {
        load += data.node[node].delivery;
    }
    load += data.node[inserted_node].delivery;

    if (load > capacity)
    {
        flag = false;
        return;
    }

    int pre_node = nl[0];
    bool checked = false;
    for (int i = 1; i < len; i++)
    {
        int node = nl[i];
        if (i == pos && !checked)
        {
            node = inserted_node;
            i--;
            checked = true;
        }

        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity)
        {
            flag = false;
            return;
        }
        time += data.time[pre_node][node];
        if (time > data.node[node].end)
        {
            flag = false;
            return;
        }
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        distance += data.dist[pre_node][node];
        pre_node = node;
    }

    flag = true;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;
}

void chk_route_O_n(Route &r, Data &data, bool &flag, double &cost)
{
    // auto start = high_resolution_clock::now();

    /* time complexity O(n) */
    std::vector<int> &nl = r.node_list;
    int len = int(nl.size());

    // start and end at DC
    if (nl[0] != data.DC || nl[len-1] != data.DC) {flag = false; return;}
    if (len == 2)
    {
        flag = true;
        cost = 0.0;
        return;
    }

    double capacity = data.vehicle.capacity;
    double distance = 0.0;
    double time = data.start_time;
    double load = 0.0;
    for (auto node : nl) {load += data.node[node].delivery;}
    if (load > capacity) {flag = false; return;}

    int pre_node = nl[0];
    for (int i = 1; i < len; i++)
    {
        int node = nl[i];
        load = load - data.node[node].delivery + data.node[node].pickup;
        if (load > capacity) {flag = false; return;}
        time += data.time[pre_node][node];
        if (time > data.node[node].end) {flag = false; return;}
        time = std::max(time, data.node[node].start) + data.node[node].s_time;
        distance += data.dist[pre_node][node];
        pre_node = node;
    }

    flag = true;
    cost = data.vehicle.d_cost + distance * data.vehicle.unit_cost;

    // only count those feasible routes
    // if (BENCHMARKING_O_1_EVAL)
    // {
    //     auto duration = long(duration_cast<nanoseconds>(high_resolution_clock::now() - start).count());
    //     call_count_move_eval += 1;
    //     // std::cout << call_count_move_eval << std::endl;
    //     mean_duration_move_eval += 1.0 / double(call_count_move_eval) * (double(duration) - mean_duration_move_eval);
    //     // mean_duration_move_eval += 0.001 * (duration - mean_duration_move_eval);
    //     mean_route_len += 1.0 / double(call_count_move_eval) * (double(len) - mean_route_len);
    //     // mean_route_len += 0.001 * (double(len) - mean_route_len);
    //     if (call_count_move_eval >= 500000)
    //     {
    //         printf("Number of move eval calls: %d, avg. time: %f nanosecs, avg. len: %f.\n", call_count_move_eval, mean_duration_move_eval, mean_route_len);
    //         exit(0);
    //     }
    // }
}

bool eval_route(Solution &s, Seq *seqList, int seqListLen, Attr &tmp_attr, Data &data)
{
    const Attr &attr_1 = seqList[0].r_index == -1 ? attr_for_one_node(data, seqList[0].start_point) : s.get(seqList[0].r_index).gat(seqList[0].start_point, seqList[0].end_point);

    const Attr &attr_2 = seqList[1].r_index == -1 ? attr_for_one_node(data, seqList[1].start_point) : s.get(seqList[1].r_index).gat(seqList[1].start_point, seqList[1].end_point);

    if ((!check_tw(attr_1, attr_2, data)) || (!check_capacity(attr_1, attr_2, data)))
        return false;
    connect(attr_1, attr_2, tmp_attr, data.dist[attr_1.e][attr_2.s], data.time[attr_1.e][attr_2.s]);

    for (int i = 2; i < seqListLen; i++)
    {
        const Attr &attr = seqList[i].r_index == -1 ? attr_for_one_node(data, seqList[i].start_point) : s.get(seqList[i].r_index).gat(seqList[i].start_point, seqList[i].end_point);

        if ((!check_tw(tmp_attr, attr, data)) || (!check_capacity(tmp_attr, attr, data)))
            return false;
        connect(tmp_attr, attr, data.dist[tmp_attr.e][attr.s], data.time[tmp_attr.e][attr.s]);
    }
    return true;
}


bool eval_move(Solution &s, Move &m, Data &data)
{
    std::vector<int> r_indice;
    r_indice.push_back(m.r_indice[0]);
    if (m.r_indice[1] != -2)
        r_indice.push_back(m.r_indice[1]);
    double ori_cost = s.get(r_indice[0]).cal_cost(data);

    if (!data.O_1_evl)
    {
        // eval the first route
        std::vector<int> target_n_l;
        target_n_l.reserve(MAX_NODE_IN_ROUTE);

        for (int i = 0; i < m.len_1; i++)
        {
            auto &seq = m.seqList_1[i];
            auto &source_n_l = s.get(seq.r_index).node_list;
            for (int index = seq.start_point; index <= seq.end_point; index++)
            {
                target_n_l.push_back(source_n_l[index]);
            }
        }
        Route r(data);
        r.node_list = target_n_l;
        bool flag = false;
        double new_cost = 0.0;
        chk_route_O_n(r, data, flag, new_cost);
        if (!flag) return false;

        // eval the second route
        if (int(r_indice.size()) == 2)
        {
            std::vector<int> target_n_l;
            target_n_l.reserve(MAX_NODE_IN_ROUTE);

            for (int i = 0; i < m.len_2; i++)
            {
                auto &seq = m.seqList_2[i];
                if (seq.r_index == -1)
                {
                    target_n_l.push_back(data.DC);
                    continue;
                }
                auto &source_n_l = s.get(seq.r_index).node_list;
                for (int index = seq.start_point; index <= seq.end_point; index++)
                {
                    target_n_l.push_back(source_n_l[index]);
                }
            }
            Route r(data);
            r.node_list = target_n_l;
            if (r_indice[1] != -1)
                ori_cost += s.get(r_indice[1]).cal_cost(data);
            bool flag = false;
            double cost = 0.0;
            chk_route_O_n(r, data, flag, cost);
            if (!flag) return false;
            new_cost += cost;
        }
        m.delta_cost = new_cost - ori_cost;
        return true;
    }

    /* eval the connection of the seqeunces in m */
    // auto start = high_resolution_clock::now();

    Attr tmp_attr_1;
    if (!eval_route(s, m.seqList_1, m.len_1, tmp_attr_1, data))
        return false;
    double new_cost = 0.0;
    if (tmp_attr_1.num_cus != 0)
        new_cost += data.vehicle.d_cost + tmp_attr_1.dist * data.vehicle.unit_cost;
    if (int(r_indice.size()) == 2)
    {
        Attr tmp_attr_2;
        if (!eval_route(s, m.seqList_2, m.len_2, tmp_attr_2, data))
            return false;
        if (r_indice[1] != -1)
            ori_cost += s.get(r_indice[1]).cal_cost(data);
        if (tmp_attr_2.num_cus != 0)
            new_cost += data.vehicle.d_cost + tmp_attr_2.dist * data.vehicle.unit_cost;
    }
    m.delta_cost = new_cost - ori_cost;

    // if (BENCHMARKING_O_1_EVAL)
    // {
    //     auto duration = duration_cast<nanoseconds>(high_resolution_clock::now() - start).count();
    //     auto new_call_count_move_eval = call_count_move_eval + int(r_indice.size());
    //     // std::cout << call_count_move_eval << std::endl;
    //     mean_duration_move_eval = 1.0 / double(new_call_count_move_eval) * (double(call_count_move_eval) * mean_duration_move_eval + double(duration));
    //     call_count_move_eval = new_call_count_move_eval;
    //     // mean_duration_move_eval += 0.001 * (double(duration) - mean_duration_move_eval);
    //     if (call_count_move_eval >= 500000)
    //     {
    //         printf("Number of move eval calls: %d, average time: %f nanosecs\n", call_count_move_eval, mean_duration_move_eval);
    //         exit(0);
    //     }
    // }
    return true;
}
