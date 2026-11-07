// Flow Field (Dijkstra map) part-file
#ifndef _BRAIN_FLOW_FIELD_HPP_
#define _BRAIN_FLOW_FIELD_HPP_

#include "brain_map.hpp"

#ifndef BRAIN_TEMPLATE
#error "main header not included"
#endif

/**
 * @brief Flow Field (a.k.a. Dijkstra map) toward one goal.
 *
 * One Dijkstra run from the goal fills the whole map: every reachable cell gets the cost to the goal and the
 * next cell on a cheapest path. After build() any number of agents can read their way in O(1) per step
 * or extract a whole path, no search is needed. Use it when many units move to the same target.
 *
 * The field uses the identity (neighbour matrix), the step costs, the locks and the corner cutting
 * rule of the map at the moment of build(). Call build() again after the map changes.
 * The field stores its own data, so the map can be searched while the field is alive.
 */
BRAIN_TEMPLATE
class basic_flow_field
{
public:
    using map_type = basic_brain_map<ISite, INeuron>;

    /// cost of the cells the goal can not be reached from
    static constexpr weight_t unreachable = immune_system::unreachable_cost;

    /**
     * @brief build the field
     * @param map the map to read, it is not modified
     * @param goal the target of the field
     * @return false if the goal is outside of the map or locked
     */
    bool build(map_type &map, const ISite &goal)
    {
        _map = &map;
        _width = static_cast<int>(map.get_width());
        _height = static_cast<int>(map.get_height());
        const std::size_t total = map.size();
        _cost.assign(total, unreachable);
        _next.assign(total, -1);
        _built = false;

        if (!map.contains(goal) || map.has_lock(goal))
            return false;
        _goal = goal;

        immune_system::cost_queue open;

        const int goal_index = index_of(goal);
        _cost[goal_index] = 0;
        open.push({0, goal_index});

        while (!open.empty())
        {
            immune_system::cost_entry e = open.top();
            open.pop();
            if (e.cost != _cost[e.id])
                continue; // stale entry

            ISite site {e.id % _width, e.id / _width};
            for (int s = 0; s < map.identity.length; ++s)
            {
                ISite nb {site.x + map.identity.horizontal[s], site.y + map.identity.vertical[s]};
                if (!map.contains(nb) || map.has_lock(nb) || !map._find_can_step(site, s))
                    continue;

                // the step costs are symmetric, so walking back from the goal gives the cost toward it
                weight_t cost = e.cost + map.identity.g_weight[s];
                int nb_index = index_of(nb);
                if (cost < _cost[nb_index])
                {
                    _cost[nb_index] = cost;
                    _next[nb_index] = e.id;
                    open.push({cost, nb_index});
                }
            }
        }

        _built = true;
        return true;
    }

    /// @return true once build() succeeded
    bool is_built() const
    {
        return _built;
    }

    /// @return the goal of the field
    ISite goal() const
    {
        return _goal;
    }

    /// @return true if the cell is inside the map and the goal can be reached from it
    bool reachable(const ISite &site) const
    {
        return cost(site) < unreachable;
    }

    /// @return the cost of the cheapest path from the cell to the goal, or unreachable
    weight_t cost(const ISite &site) const
    {
        if (!_built || !inside(site))
            return unreachable;
        return _cost[index_of(site)];
    }

    /**
     * @brief the next cell of the cheapest path toward the goal
     * @param site the current cell
     * @param out the next cell (equals site at the goal)
     * @return false if the goal can not be reached from the cell
     */
    bool next(const ISite &site, ISite &out) const
    {
        if (!reachable(site))
            return false;
        int n = _next[index_of(site)];
        out = n < 0 ? site : ISite {n % _width, n / _width};
        return true;
    }

    /**
     * @brief the unit step (dx, dy) toward the goal, (0, 0) at the goal or if there is no path
     */
    ISite direction(const ISite &site) const
    {
        ISite n;
        if (!next(site, n))
            return ISite {0, 0};
        return ISite {n.x - site.x, n.y - site.y};
    }

    /**
     * @brief the whole path from a cell to the goal (both ends included)
     * @param from the start cell
     * @param out receives the cells, cleared before
     * @return false if the goal can not be reached from the start
     */
    template <typename list_type>
    bool path(const ISite &from, list_type &out) const
    {
        out.clear();
        if (!reachable(from))
            return false;
        ISite current = from, following;
        out.push_back(current);
        while (next(current, following) && !(following.x == current.x && following.y == current.y))
        {
            current = following;
            out.push_back(current);
        }
        return true;
    }

    /// @return the cost of the most distant reachable cell (useful to scale a heat map)
    weight_t max_cost() const
    {
        weight_t result = 0;
        for (weight_t c : _cost)
            if (c < unreachable && c > result)
                result = c;
        return result;
    }

private:
    bool inside(const ISite &site) const
    {
        return site.x >= 0 && site.y >= 0 && site.x < _width && site.y < _height;
    }
    int index_of(const ISite &site) const
    {
        return site.y * _width + site.x;
    }

    map_type *_map = nullptr;
    ISite _goal {0, 0};
    int _width = 0, _height = 0;
    bool _built = false;
    std::vector<weight_t> _cost;
    std::vector<int> _next;
};

#endif
