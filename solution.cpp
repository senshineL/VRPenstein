#include "solution.h"

void attr_for_one_node(Data &data, int node, Attr &a)
{
    a.s = node;
    a.e = node;
    a.dist = 0;

    if (node == data.DC)
    {
        // depot
        a.num_cus = 0;
        a.T_D = 0;
        a.T_E = data.start_time;
        a.T_L = data.end_time;

        a.C_E = 0;
        a.C_L = 0;
        a.C_H = 0;
    }
    else
    {
        a.num_cus = 1;
        a.T_D = data.node[node].s_time;
        a.T_E = data.node[node].start;
        a.T_L = data.node[node].end;

        a.C_E = data.node[node].delivery;
        a.C_L = data.node[node].pickup;
        a.C_H = std::max(a.C_E, a.C_L);
    }
}

Attr attr_for_one_node(Data &data, int node)
{
    Attr a;
    attr_for_one_node(data, node, a);
    return a;
}

Attr connect(const Attr &tmp_a, const Attr &tmp_b, double dist_ij, double t_ij)
{
    Attr a_b;
    connect(tmp_a, tmp_b, a_b, dist_ij, t_ij);
    return a_b;
}

void connect(const Attr &tmp_a, const Attr &tmp_b, Attr &merged_attr, double dist_ij, double t_ij)
{
    merged_attr.num_cus = tmp_a.num_cus + tmp_b.num_cus;
    merged_attr.dist = tmp_a.dist + dist_ij + tmp_b.dist;

    // time window
    double delta = tmp_a.T_D + t_ij;
    double delta_wt = std::max(tmp_b.T_E - delta - tmp_a.T_L, 0.0);
    merged_attr.T_D = tmp_a.T_D + tmp_b.T_D + t_ij + delta_wt;
    merged_attr.T_E = std::max(tmp_b.T_E - delta, tmp_a.T_E) - delta_wt;
    merged_attr.T_L = std::min(tmp_b.T_L - delta, tmp_a.T_L);

    // capacity
    merged_attr.C_E = tmp_a.C_E + tmp_b.C_E;
    merged_attr.C_H = std::max(tmp_a.C_H + tmp_b.C_E, tmp_a.C_L + tmp_b.C_H);
    merged_attr.C_L = tmp_a.C_L + tmp_b.C_L;

    // s and e
    merged_attr.s = tmp_a.s;
    merged_attr.e = tmp_b.e;
}

void connect(Attr &merged_attr, const Attr &tmp_b, double dist_ij, double t_ij)
{
    merged_attr.num_cus = merged_attr.num_cus + tmp_b.num_cus;
    merged_attr.dist = merged_attr.dist + dist_ij + tmp_b.dist;

    // time window
    double delta = merged_attr.T_D + t_ij;
    double delta_wt = std::max(tmp_b.T_E - delta - merged_attr.T_L, 0.0);
    merged_attr.T_D = merged_attr.T_D + tmp_b.T_D + t_ij + delta_wt;
    merged_attr.T_E = std::max(tmp_b.T_E - delta, merged_attr.T_E) - delta_wt;
    merged_attr.T_L = std::min(tmp_b.T_L - delta, merged_attr.T_L);

    // capacity
    merged_attr.C_E = merged_attr.C_E + tmp_b.C_E;
    merged_attr.C_H = std::max(merged_attr.C_H + tmp_b.C_E, merged_attr.C_L + tmp_b.C_H);
    merged_attr.C_L = merged_attr.C_L + tmp_b.C_L;

    // s and e
    merged_attr.e = tmp_b.e;
}

std::vector<int> make_tmp_nl(Data &data)
{
    std::vector<int> nl(2, data.DC);
    return nl;
}

bool equal_attr(const Attr &a, const Attr &b)
{
    if (a.num_cus == b.num_cus && a.dist == b.dist && a.s == b.s && a.e == b.e && a.T_D == b.T_D && a.T_E == b.T_E && a.T_L == b.T_L && a.C_E == b.C_E && a.C_H == b.C_H && a.C_L == b.C_L)
        return true;
    return false;
}