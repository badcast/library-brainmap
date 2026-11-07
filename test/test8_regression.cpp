// Regression tests: the cost of every optimal pathfinder must agree on random maps,
// so refactoring of the search code can be checked against the Dijkstra reference.

#include <iostream>
#include <cstdint>
#include <cstdlib>
#include <type_traits>
#include <utility>
#include <vector>

#include <brain_map.hpp>

using namespace brain;

namespace
{
// deterministic generator, independent of std::rand
std::uint32_t g_state = 12345u;
int next_random()
{
    g_state = g_state * 1664525u + 1013904223u;
    return static_cast<int>((g_state >> 8) & 0x7fffff);
}

int path_cost(const std::vector<the_site> &path, int orthogonal, int diagonal)
{
    int cost = 0;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        const int dx = std::abs(path[i].x - path[i - 1].x);
        const int dy = std::abs(path[i].y - path[i - 1].y);
        if (dx > 1 || dy > 1 || (dx == 0 && dy == 0))
            return -1; // not a chain of neighbour cells
        cost += (dx && dy) ? diagonal : orthogonal;
    }
    return cost;
}

int failures = 0;
void check(bool ok, const char *what, int seed, int query)
{
    if (ok)
        return;
    std::cerr << "FAIL: " << what << " (map " << seed << ", query " << query << ")" << std::endl;
    ++failures;
}

int run_cost(
    brain_map &map,
    pathfinder_algo algo,
    search_mode mode,
    const the_site &from,
    const the_site &to,
    int orthogonal,
    int diagonal)
{
    brain_map::result_site result;
    if (!map.find(algo, result, from, to, mode))
        return -1;
    return path_cost(result.connections, orthogonal, diagonal);
}
} // namespace

int main()
{
    const int width = 40, height = 40;
    unsigned long long signature = 0;

    for (int seed = 0; seed < 12; ++seed)
    {
        g_state = 1000u + seed;
        brain_map map {static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height)};
        map.set_identity(matrix_identity::DiagonalMethod);
        map.set_heuristic(heuristic_method::Manhattan);
        for (int x = 0; x < width; ++x)
            for (int y = 0; y < height; ++y)
                map.set_lock(the_site {x, y}, next_random() % 100 < 20 + seed * 2);

        hpa_graph hpa;
        hpa.build(map, 8, false);

        for (int q = 0; q < 40; ++q)
        {
            the_site from {next_random() % width, next_random() % height};
            the_site to {next_random() % width, next_random() % height};
            if (map.has_lock(from) || map.has_lock(to))
                continue;

            const int ref = run_cost(map, pathfinder_algo::Dijkstra, search_mode::UniDirectional, from, to, 1, 2);
            signature = signature * 31 + static_cast<unsigned long long>(ref);

            const int astar = run_cost(map, pathfinder_algo::AStar, search_mode::UniDirectional, from, to, 1, 2);
            const int astar_bi = run_cost(map, pathfinder_algo::AStar, search_mode::BiDirectional, from, to, 1, 2);
            const int dijkstra_bi
                = run_cost(map, pathfinder_algo::Dijkstra, search_mode::BiDirectional, from, to, 1, 2);
            const int jps
                = run_cost(map, pathfinder_algo::JumpPointSearch, search_mode::UniDirectional, from, to, 1, 2);
            // IDA* re-searches many times: check every fourth query only
            const int ida = (q % 4 == 0)
                                ? run_cost(map, pathfinder_algo::IDAStar, search_mode::UniDirectional, from, to, 1, 2)
                                : ref;

            check(astar == ref, "A* != Dijkstra", seed, q);
            check(astar_bi == ref, "bidirectional A* != Dijkstra", seed, q);
            check(dijkstra_bi == ref, "bidirectional Dijkstra != Dijkstra", seed, q);
            check(jps == ref, "JPS != Dijkstra", seed, q);
            check(ida == ref, "IDA* != Dijkstra", seed, q);

            std::vector<the_site> hpa_path;
            weight_t hpa_cost = 0;
            const bool hpa_ok = hpa.find(from, to, hpa_path, &hpa_cost);
            check(hpa_ok == (ref >= 0), "HPA reachability differs", seed, q);
            if (hpa_ok && ref >= 0)
            {
                check(hpa_cost == ref, "HPA cost != Dijkstra", seed, q);
                check(path_cost(hpa_path, 1, 2) == ref, "HPA path is broken", seed, q);
            }

            flow_field field;
            field.build(map, to);
            check(field.reachable(from) == (ref >= 0), "flow field reachability differs", seed, q);
            if (ref >= 0)
                check(field.cost(from) == ref, "flow field cost != Dijkstra", seed, q);
        }
    }

    // a sparse HPA graph may fall back to a map search, which must not touch the neurons of the map
    {
        g_state = 4242u;
        brain_map map {width, height};
        for (int x = 0; x < width; ++x)
            for (int y = 0; y < height; ++y)
                map.set_lock(the_site {x, y}, next_random() % 100 < 35);
        hpa_graph sparse;
        sparse.build(map, 6, true);
        map.clear(false);
        int fallbacks_checked = 0;
        for (int q = 0; q < 80; ++q)
        {
            the_site from {next_random() % width, next_random() % height};
            the_site to {next_random() % width, next_random() % height};
            if (map.has_lock(from) || map.has_lock(to))
                continue;
            std::vector<the_site> path;
            weight_t cost = 0;
            const bool ok = sparse.find(from, to, path, &cost);
            bool touched = false;
            for (the_neuron *n = map.front(); n <= map.back(); ++n)
                if (n->flags != 0 || n->parent != nullptr)
                    touched = true;
            check(!touched, "sparse HPA modified the neurons of the map", 300, q);
            const int ref = run_cost(map, pathfinder_algo::Dijkstra, search_mode::UniDirectional, from, to, 1, 2);
            map.clear(false);
            check(ok == (ref >= 0), "sparse HPA reachability differs", 300, q);
            if (ok)
            {
                check(path_cost(path, 1, 2) == cost, "sparse HPA cost does not match its path", 300, q);
                check(cost >= ref, "sparse HPA is cheaper than Dijkstra", 300, q);
                ++fallbacks_checked;
            }
        }
        check(fallbacks_checked > 0, "no sparse HPA query succeeded", 300, 0);
    }

    // BFS is optimal for equal step costs
    {
        g_state = 777u;
        brain_map map {width, height};
        map.set_identity(matrix_identity::PlusMethod);
        for (int x = 0; x < width; ++x)
            for (int y = 0; y < height; ++y)
                map.set_lock(the_site {x, y}, next_random() % 100 < 25);
        for (int q = 0; q < 60; ++q)
        {
            the_site from {next_random() % width, next_random() % height};
            the_site to {next_random() % width, next_random() % height};
            if (map.has_lock(from) || map.has_lock(to))
                continue;
            const int ref = run_cost(map, pathfinder_algo::Dijkstra, search_mode::UniDirectional, from, to, 1, 1);
            check(
                run_cost(map, pathfinder_algo::BreadthFirstSearch, search_mode::UniDirectional, from, to, 1, 1) == ref,
                "BFS != Dijkstra",
                100,
                q);
            check(
                run_cost(map, pathfinder_algo::BreadthFirstSearch, search_mode::BiDirectional, from, to, 1, 1) == ref,
                "bidirectional BFS != Dijkstra",
                100,
                q);
        }
    }

    // the map owns raw memory: it must not be copied, but may be moved
    static_assert(!std::is_copy_constructible<brain_map>::value, "brain_map must not be copyable");
    static_assert(!std::is_copy_assignable<brain_map>::value, "brain_map must not be copy assignable");
    {
        brain_map a {10, 10};
        a.set_lock(the_site {3, 3}, true);
        a.set_custom_weights(200, 300); // does not fit int8_t
        brain_map b {std::move(a)};
        check(b.has_lock(the_site {3, 3}), "lock lost by move", 200, 0);
        brain_map c {2, 2};
        c = std::move(b);
        check(c.get_width() == 10 && c.has_lock(the_site {3, 3}), "move assignment lost data", 200, 2);

        brain_map::result_site res;
        c.set_lock(the_site {3, 3}, false);
        check(c.find(res, the_site {0, 0}, the_site {0, 3}), "path on moved map", 200, 3);
        check(path_cost(res.connections, 200, 300) == 600, "custom weights >127 give a wrong cost", 200, 4);
    }

    // the search state is reset by a generation stamp: the results must not change over many searches,
    // also when the 16 bit stamp wraps around
    {
        brain_map map {12, 12};
        for (int x = 0; x < 12; ++x)
            for (int y = 0; y < 12; ++y)
                map.set_lock(the_site {x, y}, (x * 7 + y * 3) % 5 == 0 && !(x == 0 && y == 0));
        map.set_lock(the_site {11, 11}, false);
        brain_map::result_site first;
        const bool found = map.find(first, the_site {0, 0}, the_site {11, 11});
        check(found, "stamp test: no path", 400, 0);
        const int first_cost = path_cost(first.connections, 1, 2);
        for (int i = 1; i <= 70000; ++i)
        {
            brain_map::result_site again;
            const search_mode mode = (i % 3 == 0) ? search_mode::BiDirectional : search_mode::UniDirectional;
            const bool ok = map.find(pathfinder_algo::AStar, again, the_site {0, 0}, the_site {11, 11}, mode);
            if (!ok || path_cost(again.connections, 1, 2) != first_cost)
            {
                check(false, "result changed over repeated searches", 400, i);
                break;
            }
        }
    }

    // maze generators must survive tiny maps
    for (int algo = 0; algo <= static_cast<int>(maze_algorithm::CellularAutomata); ++algo)
        for (std::uint32_t side = 1; side <= 4; ++side)
        {
            brain_map small {side, side};
            small.create_maze_ex(static_cast<maze_algorithm>(algo));
        }
    {
        brain_map one {1, 1}, two {2, 2};
        one.create_maze();
        two.create_maze();
    }

    std::cout << "signature " << signature << std::endl;
    if (failures)
    {
        std::cerr << failures << " regression check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "regression: OK" << std::endl;
    return 0;
}
