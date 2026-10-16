#include "test_header.h"

struct AlgorithmEntry
{
    MazeAlgorithm algorithm;
    const char *name;
    bool guaranteedConnected;
};

static the_neuron *find_nearest_free(brain_map &m, const the_site &site)
{
    the_neuron *candidate = m.get(site);
    if(candidate != nullptr && !m.has_lock(candidate))
        return candidate;

    auto neighbours = m.get_neighbours<brain_map::list_neuron>(MatrixIdentity::DiagonalMethod, candidate);
    for(auto *nb : neighbours)
    {
        if(!m.has_lock(nb))
            return nb;
    }
    return candidate;
}

int main()
{
    const AlgorithmEntry algorithms[] = {
        {MazeAlgorithm::RecursiveBacktracer, "RecursiveBacktracer", true},
        {MazeAlgorithm::RecursiveDivision, "RecursiveDivision", true},
        {MazeAlgorithm::SelectiveStormChaos, "SelectiveStormChaos", false},
        {MazeAlgorithm::ForcedChaos, "ForcedChaos", false},
        {MazeAlgorithm::RandomizedPrim, "RandomizedPrim", true},
        {MazeAlgorithm::RandomizedKruskal, "RandomizedKruskal", true},
        {MazeAlgorithm::Ellers, "Ellers", true},
        {MazeAlgorithm::HuntAndKill, "HuntAndKill", true},
        {MazeAlgorithm::BinaryTree, "BinaryTree", true},
        {MazeAlgorithm::Sidewinder, "Sidewinder", true},
        {MazeAlgorithm::AldousBroder, "AldousBroder", true},
        {MazeAlgorithm::Wilsons, "Wilsons", true},
        {MazeAlgorithm::CellularAutomata, "CellularAutomata", true},
    };

    std::srand(19992023);
    brain_map test_map {41, 41};
    test_map.set_identity(MatrixIdentity::PlusMethod);
    test_map.set_heuristic(HeuristicMethod::Manhattan);

    the_site start_site {0, 0};
    the_site end_site {40, 40};

    for(const auto &entry : algorithms)
    {
        ++iterate;
        test_map.create_maze_ex(entry.algorithm, start_site, end_site);

        std::size_t locked_count = 0;
        std::size_t open_count = 0;
        for(auto *n = test_map.front(), *ne = test_map.back(); n <= ne; ++n)
        {
            if(test_map.has_lock(n))
                ++locked_count;
            else
                ++open_count;
        }

        brain_map::result_site nav_result;
        the_neuron *from_n = find_nearest_free(test_map, start_site);
        the_neuron *to_n = find_nearest_free(test_map, end_site);
        bool path_found = test_map.find(nav_result, from_n, to_n);
        test_map.clear(false);

        cout << "Algorithm: " << entry.name
             << " | Open: " << open_count
             << " | Locked: " << locked_count
             << " | Path size: " << nav_result.connections.size() << endl;

        if(open_count == 0 || locked_count == 0)
        {
            cerr << "Error: " << entry.name << " produced degenerate map (open=" << open_count << ", locked=" << locked_count << ")" << endl;
            ++errors;
        }

        if(entry.guaranteedConnected && !path_found)
        {
            cerr << "Error: " << entry.name << " failed to produce a connected path from start to end!" << endl;
            ++errors;
        }
    }

    if(errors != 0)
    {
        cerr << "Maze algorithms test failed with " << errors << " error(s)." << endl;
        return 1;
    }

    cout << "All " << iterate << " maze algorithms verified successfully." << endl;
    return 0;
}
