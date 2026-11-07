#include "test_header.h"

int main()
{
    brain_map test_map {51, 51};
    test_map.set_identity(matrix_identity::PlusMethod);
    test_map.set_heuristic(heuristic_method::Manhattan);

    // Test 1: Empty grid path
    the_site start {5, 5};
    the_site goal {45, 45};

    brain_map::result_site res_uni;
    brain_map::result_site res_bi;

    bool ok_uni = test_map.find(res_uni, start, goal, search_mode::UniDirectional);
    test_map.clear(false);

    bool ok_bi = test_map.find(res_bi, start, goal, search_mode::BiDirectional);
    test_map.clear(false);

    if(!ok_uni || !ok_bi)
    {
        cerr << "Test 1 failed: Path not found on empty grid." << endl;
        return 1;
    }

    if(res_uni.connections.size() != res_bi.connections.size())
    {
        cerr << "Test 1 failed: Path sizes differ on empty grid (Uni: "
             << res_uni.connections.size() << ", Bi: " << res_bi.connections.size() << ")" << endl;
        return 1;
    }

    // Verify continuity of the bidirectional path
    for(size_t i = 1; i < res_bi.connections.size(); ++i)
    {
        int dx = std::abs(res_bi.connections[i].x - res_bi.connections[i - 1].x);
        int dy = std::abs(res_bi.connections[i].y - res_bi.connections[i - 1].y);
        if(dx + dy != 1)
        {
            cerr << "Test 1 failed: Non-adjacent step in BiDirectional path at index " << i << endl;
            return 1;
        }
    }

    // Test 2: Identical start and goal
    brain_map::result_site res_same;
    bool ok_same = test_map.find(res_same, start, start, search_mode::BiDirectional);
    test_map.clear(false);
    if(!ok_same || res_same.connections.size() != 1 || res_same.status != navigation_status::Opened)
    {
        cerr << "Test 2 failed: Identical start and goal." << endl;
        return 1;
    }

    // Test 3: Locked start or goal
    test_map.set_lock(start, true);
    brain_map::result_site res_locked;
    bool ok_locked = test_map.find(res_locked, start, goal, search_mode::BiDirectional);
    test_map.clear(false);
    if(ok_locked || res_locked.status != navigation_status::Locked)
    {
        cerr << "Test 3 failed: Locked node not detected." << endl;
        return 1;
    }
    test_map.set_lock(start, false);

    // Test 4: Completely blocked path
    for(int y = 0; y < 51; ++y)
        test_map.set_lock({25, y}, true);

    brain_map::result_site res_blocked;
    bool ok_blocked = test_map.find(res_blocked, start, goal, search_mode::BiDirectional);
    test_map.clear(false);
    if(ok_blocked || res_blocked.status != navigation_status::Closed)
    {
        cerr << "Test 4 failed: Blocked path did not return Closed." << endl;
        return 1;
    }

    // Clear wall
    for(int y = 0; y < 51; ++y)
        test_map.set_lock({25, y}, false);

    // Test 5: Verify on generated mazes
    std::srand(42);
    maze_algorithm algorithms[] = {
        maze_algorithm::RecursiveBacktracer,
        maze_algorithm::RandomizedPrim,
        maze_algorithm::RandomizedKruskal,
        maze_algorithm::Ellers,
        maze_algorithm::HuntAndKill,
        maze_algorithm::BinaryTree,
        maze_algorithm::Sidewinder,
        maze_algorithm::Wilsons
    };

    for(auto alg : algorithms)
    {
        test_map.create_maze_ex(alg, {0, 0}, {50, 50});

        the_site s {0, 0};
        the_site g {50, 50};

        // Find nearest open cells
        while(test_map.contains(s) && test_map.has_lock(s))
            s.x = (s.x + 1) % 51;
        while(test_map.contains(g) && test_map.has_lock(g))
            g.x = (g.x > 0 ? g.x - 1 : 50);

        brain_map::result_site r_uni;
        brain_map::result_site r_bi;

        bool f_uni = test_map.find(r_uni, s, g, search_mode::UniDirectional);
        test_map.clear(false);

        bool f_bi = test_map.find(r_bi, s, g, search_mode::BiDirectional);
        test_map.clear(false);

        if(f_uni != f_bi)
        {
            cerr << "Test 5 failed: Result mismatch for algorithm " << static_cast<int>(alg)
                 << " (Uni: " << f_uni << ", Bi: " << f_bi << ")" << endl;
            return 1;
        }

        if(f_bi)
        {
            // Verify path continuity
            for(size_t i = 1; i < r_bi.connections.size(); ++i)
            {
                int dx = std::abs(r_bi.connections[i].x - r_bi.connections[i - 1].x);
                int dy = std::abs(r_bi.connections[i].y - r_bi.connections[i - 1].y);
                if(dx + dy != 1)
                {
                    cerr << "Test 5 failed: Discontinuity in BiDirectional maze path at " << i << endl;
                    return 1;
                }
            }
        }
    }

    // Test 6: Verify all heuristics with BiDirectional search (ensuring both frontiers expand)
    heuristic_method heuristics[] = {
        heuristic_method::Manhattan,
        heuristic_method::Hypot,
        heuristic_method::Pythagorean,
        heuristic_method::Chebyshev,
        heuristic_method::Octile
    };

    test_map.clear(true);
    for(auto heur : heuristics)
    {
        test_map.set_heuristic(heur);
        brain_map::result_site r_heur;
        bool ok_heur = test_map.find(r_heur, start, goal, search_mode::BiDirectional);
        if(!ok_heur || r_heur.status != navigation_status::Opened || r_heur.connections.empty())
        {
            cerr << "Test 6 failed: Heuristic " << static_cast<int>(heur) << " failed in BiDirectional search." << endl;
            return 1;
        }

        // Verify both frontiers expanded
        size_t fwd_nodes = 0, bwd_nodes = 0;
        for(size_t i = 0; i < test_map.size(); ++i)
        {
            the_neuron *n = test_map.front() + i;
            if(n->flags & (1 | 2)) fwd_nodes++;
            if(n->flags & (8 | 16)) bwd_nodes++;
        }
        test_map.clear(false);

        if(fwd_nodes == 0 || bwd_nodes == 0)
        {
            cerr << "Test 6 failed: Zero-expansion for heuristic " << static_cast<int>(heur)
                 << " (FWD: " << fwd_nodes << ", BWD: " << bwd_nodes << ")" << endl;
            return 1;
        }

        if(heur == heuristic_method::Hypot)
        {
            if(fwd_nodes < 10 || bwd_nodes < 10)
            {
                cerr << "Test 6 failed: Hypot did not expand both frontiers symmetrically (FWD: "
                     << fwd_nodes << ", BWD: " << bwd_nodes << ")" << endl;
                return 1;
            }
        }
    }

    cout << "All BiDirectional A* tests passed successfully!" << endl;
    return 0;
}
