#include "brain_map.hpp"
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace brain;

static int failures = 0;
#define CHECK(cond, ...)                                  \
    do                                                    \
    {                                                     \
        if (!(cond))                                      \
        {                                                 \
            std::printf("FAIL line %d: ", __LINE__);      \
            std::printf(__VA_ARGS__);                     \
            std::printf("\n");                            \
            ++failures;                                   \
        }                                                 \
    } while (0)

static weight_t path_cost(const brain_map::result_site &r)
{
    weight_t cost = 0;
    for (size_t i = 1; i < r.connections.size(); ++i)
    {
        int dx = std::abs(r.connections[i].x - r.connections[i - 1].x);
        int dy = std::abs(r.connections[i].y - r.connections[i - 1].y);
        cost += (dx && dy) ? 2 : 1;
    }
    return cost;
}

static bool valid_path(brain_map &m, const brain_map::result_site &r, the_site a, the_site b)
{
    if (r.connections.empty() || r.connections.front() != a || r.connections.back() != b)
        return false;
    for (size_t i = 0; i < r.connections.size(); ++i)
    {
        if (m.has_lock(r.connections[i]))
            return false;
        if (i && (std::abs(r.connections[i].x - r.connections[i - 1].x) > 1
                  || std::abs(r.connections[i].y - r.connections[i - 1].y) > 1))
            return false;
    }
    return true;
}

// Regression: bidirectional A*/Dijkstra must return the same cost as the unidirectional search
static void fuzz_bidirectional()
{
    std::mt19937 rng(7);
    const matrix_identity ids[] {matrix_identity::PlusMethod, matrix_identity::DiagonalMethod};
    const heuristic_method heuristics[] {
        heuristic_method::Manhattan, heuristic_method::Chebyshev, heuristic_method::Octile, heuristic_method::Hypot};
    for (int it = 0; it < 3000; ++it)
    {
        int w = 5 + rng() % 30, h = 5 + rng() % 30;
        brain_map m(w, h);
        int density = rng() % 45;
        for (int y = 0; y < h; ++y)
            for (int x = 0; x < w; ++x)
                if (static_cast<int>(rng() % 100) < density)
                    m.set_lock(m.get(the_site {x, y}), true);
        the_site a {static_cast<int>(rng() % w), static_cast<int>(rng() % h)};
        the_site b {static_cast<int>(rng() % w), static_cast<int>(rng() % h)};
        m.set_lock(m.get(a), false);
        m.set_lock(m.get(b), false);
        m.set_identity(ids[rng() % 2]);
        m.set_heuristic(heuristics[rng() % 4]);

        for (auto algo : {pathfinder_algo::AStar, pathfinder_algo::Dijkstra})
        {
            brain_map::result_site uni, bi;
            bool ok_uni = m.find(algo, uni, a, b, search_mode::UniDirectional);
            bool ok_bi = m.find(algo, bi, a, b, search_mode::BiDirectional);
            CHECK(ok_uni == ok_bi, "iteration %d: found %d vs %d", it, ok_uni, ok_bi);
            if (ok_uni && ok_bi)
            {
                CHECK(valid_path(m, bi, a, b), "iteration %d: invalid bidirectional path", it);
                CHECK(path_cost(uni) == path_cost(bi), "iteration %d: cost %d vs %d", it, path_cost(uni), path_cost(bi));
            }
            if (failures > 10)
                return;
        }
    }
}

int main()
{
    fuzz_bidirectional();

    const pathfinder_algo algos[] {
        pathfinder_algo::AStar,
        pathfinder_algo::Dijkstra,
        pathfinder_algo::BreadthFirstSearch,
        pathfinder_algo::DepthFirstSearch};
    const search_mode modes[] {search_mode::UniDirectional, search_mode::BiDirectional};
    const the_site start {0, 0}, goal {19, 0};

    for (auto mode : modes)
    {
        weight_t dijkstra_cost = -1, astar_cost = -1;
        for (auto algo : algos)
        {
            brain_map m(20, 20);
            for (int y = 0; y < 19; ++y)
                m.set_lock(m.get(the_site {10, y}), true);

            brain_map::result_site r;
            bool ok = m.find(algo, r, start, goal, mode);
            CHECK(ok, "algo %d mode %d not found", (int) algo, (int) mode);
            CHECK(valid_path(m, r, start, goal), "algo %d mode %d invalid path", (int) algo, (int) mode);
            if (algo == pathfinder_algo::AStar)
                astar_cost = path_cost(r);
            if (algo == pathfinder_algo::Dijkstra)
                dijkstra_cost = path_cost(r);

            // unreachable goal
            brain_map m2(20, 20);
            for (int y = 0; y < 20; ++y)
                m2.set_lock(m2.get(the_site {10, y}), true);
            brain_map::result_site r2;
            CHECK(!m2.find(algo, r2, start, goal, mode), "algo %d reached the wall", (int) algo);
            CHECK(r2.status == navigation_status::Closed || r2.status == navigation_status::Locked, "status");

            // locked goal
            brain_map m3(5, 5);
            m3.set_lock(m3.get(the_site {4, 4}), true);
            brain_map::result_site r3;
            CHECK(!m3.find(algo, r3, the_site {0, 0}, the_site {4, 4}, mode), "locked goal");
            CHECK(r3.status == navigation_status::Locked, "locked status");

            // start == goal
            brain_map m4(5, 5);
            brain_map::result_site r4;
            CHECK(m4.find(algo, r4, the_site {2, 2}, the_site {2, 2}, mode), "start==goal");
            CHECK(r4.connections.size() == 1, "start==goal size %zu", r4.connections.size());
        }
        CHECK(astar_cost == dijkstra_cost, "A* optimal: %d vs %d (mode %d)", astar_cost, dijkstra_cost, (int) mode);
    }

    // default pathfinder setter
    brain_map m(10, 10);
    m.set_pathfinder(pathfinder_algo::DepthFirstSearch);
    CHECK(m.get_pathfinder() == pathfinder_algo::DepthFirstSearch, "set_pathfinder");

    std::printf(failures ? "FAILED (%d)\n" : "OK\n", failures);
    return failures ? 1 : 0;
}
