#include "test_header.h"

// set_custom_*: own step costs, own heuristic and the corner cutting rule

namespace
{
    weight_t octile10(weight_t dx, weight_t dy)
    {
        dx = std::abs(dx);
        dy = std::abs(dy);
        return 10 * std::max(dx, dy) + 4 * std::min(dx, dy);
    }
} // namespace

int main()
{
    brain_map map {10, 10};
    map.set_identity(MatrixIdentity::DiagonalMethod);

    // Custom heuristic
    if(!map.set_custom_heuristic(octile10) || map.get_heuristic() != HeuristicMethod::Custom)
    {
        cerr << "custom heuristic was not accepted" << endl;
        return 1;
    }
    if(map.set_custom_heuristic(nullptr))
    {
        cerr << "null heuristic must be refused" << endl;
        return 1;
    }

    // Custom costs survive a change of the identity
    map.set_custom_weights(10, 14);
    for(MatrixIdentity id : {MatrixIdentity::DiagonalMethod, MatrixIdentity::PlusMethod, MatrixIdentity::CrossMethod})
    {
        map.set_identity(id);
        for(int s = 0; s < map.identity.length; ++s)
        {
            const bool diagonal = map.identity.horizontal[s] != 0 && map.identity.vertical[s] != 0;
            if(map.identity.g_weight[s] != (diagonal ? 14 : 10))
            {
                cerr << "wrong custom cost at direction " << s << endl;
                return 1;
            }
        }
        if(map.get_identity() != id)
        {
            cerr << "identity not recognized with custom costs" << endl;
            return 1;
        }
    }

    // Corner cutting: two locked cells touching at the corner
    map.set_identity(MatrixIdentity::DiagonalMethod);
    map.set_lock(the_site {1, 0}, true);
    map.set_lock(the_site {0, 1}, true);
    brain_map::result_site res;

    map.set_custom_corner_cutting(false);
    if(map.find(res, the_site {0, 0}, the_site {1, 1}))
    {
        cerr << "corner cut happened although it is forbidden" << endl;
        return 1;
    }
    map.clear(false);
    res = {};
    map.set_custom_corner_cutting(true);
    if(!map.find(res, the_site {0, 0}, the_site {1, 1}) || res.connections.size() != 2)
    {
        cerr << "corner cut was not allowed" << endl;
        return 1;
    }

    // Back to the built-in costs
    map.set_custom_weights(0, 0);
    map.set_identity(MatrixIdentity::PlusMethod);
    if(map.identity.g_weight[0] != 1)
    {
        cerr << "built-in costs were not restored" << endl;
        return 1;
    }
    cout << "custom settings: OK" << endl;
    return 0;
}
