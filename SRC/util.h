#pragma once
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cctype>
#include <locale>
#include <random>

// split string with a delimiter
std::vector<std::string> split(const std::string &s, char delimiter);

// trim from start (in place)
static inline void ltrim(std::string &s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    })
        .base(),
        s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s)
{
    ltrim(s);
    rtrim(s);
}

// calculate mean values of an array
static inline double mean(double *a, int start_index, int len)
{
    double sum = 0.0;
    for (int i = start_index; i < start_index+len; i++)
    {
        sum += a[i];
    }
    return sum / double(len);
}

// calculate mean values of an array
static inline double mean(std::vector<double> a, int start_index, int len)
{
    double sum = 0.0;
    for (int i = start_index; i < start_index + len; i++)
    {
        sum += a[i];
    }
    return sum / double(len);
}

static inline double mean(int *a, int start_index, int len)
{
    double sum = 0.0;
    for (int i = start_index; i < start_index + len; i++)
    {
        sum += a[i];
    }
    return sum / double(len);
}

// check a number is perfect square
static inline bool chk_p_square(double x)
{
    double sr = sqrt(x);
    return ((sr - floor(sr)) == 0);
}

// argsort, a is the result list, b is unchanged
static inline void argsort(double b[], int a[], int len)
{
    for (int i = 0; i < len; i++)
    {
        a[i] = i;
    }
    std::sort(a, a + len, [&b](int x, int y) { return b[x] < b[y]; });
}

static inline void argsort(std::vector<double> &b, std::vector<int> &a, int len)
{
    for (int i = 0; i < len; i++)
    {
        a[i] = i;
    }
    std::sort(a.begin(), a.begin() + len, [&b](int x, int y) { return b[x] < b[y]; });
}

static inline int randint(int from, int end, std::mt19937 &rng)
{
    std::uniform_int_distribution<int> distr(from, end);
    return distr(rng);
}

static inline double rand(double from, double end, std::mt19937 &rng)
{
    std::uniform_real_distribution<double> distr(from, end);
    return distr(rng);
}