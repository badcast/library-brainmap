#include "test_header.h"

int main()
{
    brain_map test_map {51, 51};
    test_map.set_identity(MatrixIdentity::PlusMethod);
    test_map.set_heuristic(HeuristicMethod::Manhattan);

    // Test 1: Empty grid path
    the_site start {5, 5};
    the_site goal {45, 45};

    brain_map::result_site res_uni;
    brain_map::result_site res_bi;

    bool ok_uni = test_map.find(res_uni, start, goal, SearchMode::UniDirectional);
    test_map.clear(false);

    bool ok_bi = test_map.find(res_bi, start, goal, SearchMode::BiDirectional);
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
    bool ok_same = test_map.find(res_same, start, start, SearchMode::BiDirectional);
    test_map.clear(false);
    if(!ok_same || res_same.connections.size() != 1 || res_same.status != NavigationStatus::Opened)
    {
        cerr << "Test 2 failed: Identical start and goal." << endl;
        return 1;
    }

    // Test 3: Locked start or goal
    test_map.set_lock(start, true);
    brain_map::result_site res_locked;
    bool ok_locked = test_map.find(res_locked, start, goal, SearchMode::BiDirectional);
    test_map.clear(false);
    if(ok_locked || res_locked.status != NavigationStatus::Locked)
    {
        cerr << "Test 3 failed: Locked node not detected." << endl;
        return 1;
    }
    test_map.set_lock(start, false);

    // Test 4: Completely blocked path
    for(int y = 0; y < 51; ++y)
        test_map.set_lock({25, y}, true);

    brain_map::result_site res_blocked;
    bool ok_blocked = test_map.find(res_blocked, start, goal, SearchMode::BiDirectional);
    test_map.clear(false);
    if(ok_blocked || res_blocked.status != NavigationStatus::Closed)
    {
        cerr << "Test 4 failed: Blocked path did not return Closed." << endl;
        return 1;
    }

    // Clear wall
    for(int y = 0; y < 51; ++y)
        test_map.set_lock({25, y}, false);

    // Test 5: Verify on generated mazes
    std::srand(42);
    MazeAlgorithm algorithms[] = {
        MazeAlgorithm::RecursiveBacktracer,
        MazeAlgorithm::RandomizedPrim,
        MazeAlgorithm::RandomizedKruskal,
        MazeAlgorithm::Ellers,
        MazeAlgorithm::HuntAndKill,
        MazeAlgorithm::BinaryTree,
        MazeAlgorithm::Sidewinder,
        MazeAlgorithm::Wilsons
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

        bool f_uni = test_map.find(r_uni, s, g, SearchMode::UniDirectional);
        test_map.clear(false);

        bool f_bi = test_map.find(r_bi, s, g, SearchMode::BiDirectional);
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

    cout << "All BiDirectional A* tests passed successfully!" << endl;
    return 0;
}
