// HPA* (hierarchical path finding) part-file
#ifndef _BRAIN_HPA_HPP_
#define _BRAIN_HPA_HPP_

#include "brain_map.hpp"

#ifndef BRAIN_TEMPLATE
#error "main header not included"
#endif

/**
 * @brief Hierarchical path finding (HPA*).
 *
 * The map is cut into square clusters. Every free cell that has a legal step into another cluster is an
 * "entrance" node of the abstract graph; the nodes of one cluster are linked by their exact in-cluster
 * costs, the nodes of neighbour clusters by the border step. A query inserts the start and the goal into
 * the graph, searches the small abstract graph and refines every abstract edge into real cells.
 *
 * Because every border step is represented, the abstract graph is exact: the returned cost equals the cost
 * of a Dijkstra search (the path itself may differ between equal-cost alternatives).
 * The price is the memory and the build time of the graph, so the class pays off on big maps with many queries.
 *
 * The graph stores its own data. It is built for the walls, the identity, the step costs and the corner cutting
 * rule of the map at the moment of build(); call build() again after the map changes.
 */
BRAIN_TEMPLATE
class basic_hpa_graph
{
public:
    using map_type = basic_brain_map<ISite, INeuron>;

    /**
     * @brief build the abstract graph
     * @param map the map to read, it is not modified
     * @param cluster_size the side of a square cluster (at least 2), bigger clusters mean a smaller abstract graph
     *        but slower queries
     * @param sparse false: every border step is represented, the graph is exact but big.
     *        true: only the first, the middle and the last step of every free stretch of a border are kept. The graph
     *        is several times smaller and much faster to build, the paths may be slightly longer than optimal, and when
     *        the sparse graph finds no route find() falls back to a plain A* search of the map, so reachability stays
     * exact.
     */
    bool build(map_type &map, int cluster_size = 8, bool sparse = false)
    {
        _built = false;
        if (cluster_size < 2)
            return false;
        _map = &map;
        _cs = cluster_size;
        _sparse = sparse;
        _w = static_cast<int>(map.get_width());
        _h = static_cast<int>(map.get_height());
        _sites.clear();
        _adj.clear();
        _node_of.assign(static_cast<std::size_t>(_w) * _h, -1);

        // 1. the border steps that become edges: (cell, direction) pairs
        struct crossing
        {
            ISite a;
            int s;
        };
        std::vector<crossing> kept;
        if (!sparse)
        {
            for (int y = 0; y < _h; ++y)
                for (int x = 0; x < _w; ++x)
                {
                    ISite a {x, y};
                    if (map.has_lock(a))
                        continue;
                    for (int s = 0; s < map.identity.length; ++s)
                        if (cross_step(a, s))
                            kept.push_back({a, s});
                }
        }
        else
        {
            // group the crossings by the ordered pair of clusters; the scan order walks along every border
            struct keyed_crossing
            {
                std::size_t ca;
                std::size_t cb;
                crossing c;
            };
            std::vector<keyed_crossing> all_crossings;
            for (int y = 0; y < _h; ++y)
                for (int x = 0; x < _w; ++x)
                {
                    ISite a {x, y};
                    if (map.has_lock(a))
                        continue;
                    for (int s = 0; s < map.identity.length; ++s)
                        if (cross_step(a, s))
                        {
                            ISite b {x + map.identity.horizontal[s], y + map.identity.vertical[s]};
                            all_crossings.push_back({cluster_index(a), cluster_index(b), {a, s}});
                        }
                }
            std::stable_sort(
                all_crossings.begin(), all_crossings.end(), [](const keyed_crossing &lhs, const keyed_crossing &rhs) {
                    if (lhs.ca != rhs.ca)
                        return lhs.ca < rhs.ca;
                    return lhs.cb < rhs.cb;
                });

            std::size_t g_begin = 0;
            while (g_begin < all_crossings.size())
            {
                std::size_t g_end = g_begin + 1;
                while (g_end < all_crossings.size() && all_crossings[g_end].ca == all_crossings[g_begin].ca
                       && all_crossings[g_end].cb == all_crossings[g_begin].cb)
                    ++g_end;

                std::size_t begin = g_begin;
                while (begin < g_end)
                {
                    // a run is a stretch of neighbouring border cells
                    std::size_t end = begin + 1;
                    while (end < g_end && std::abs(all_crossings[end].c.a.x - all_crossings[end - 1].c.a.x) <= 1
                           && std::abs(all_crossings[end].c.a.y - all_crossings[end - 1].c.a.y) <= 1)
                        ++end;
                    kept.push_back(all_crossings[begin].c);
                    if (end - begin > 2)
                        kept.push_back(all_crossings[begin + (end - begin) / 2].c);
                    if (end - begin > 1)
                        kept.push_back(all_crossings[end - 1].c);
                    begin = end;
                }
                g_begin = g_end;
            }
        }

        // 2. nodes and the inter-cluster edges (the step costs are symmetric: add both directions)
        auto node_for = [&](const ISite &cell) {
            int &id = _node_of[index_of(cell)];
            if (id < 0)
            {
                id = static_cast<int>(_sites.size());
                _sites.push_back(cell);
                _adj.emplace_back();
            }
            return id;
        };
        for (const crossing &c : kept)
        {
            ISite b {c.a.x + map.identity.horizontal[c.s], c.a.y + map.identity.vertical[c.s]};
            int na = node_for(c.a), nb = node_for(b);
            _adj[na].push_back({nb, map.identity.g_weight[c.s]});
            if (sparse)
                _adj[nb].push_back({na, map.identity.g_weight[c.s]});
        }

        // 3. intra-cluster edges: exact costs between the nodes of one cluster
        std::vector<std::vector<int>> by_cluster(static_cast<std::size_t>(clusters_x()) * clusters_y());
        for (std::size_t n = 0; n < _sites.size(); ++n)
            by_cluster[cluster_index(_sites[n])].push_back(static_cast<int>(n));
        std::vector<weight_t> dist;
        std::vector<int> parent;
        for (auto &group : by_cluster)
            for (int from : group)
            {
                local_dijkstra(_sites[from], dist, parent);
                rect r = cluster_rect(_sites[from]);
                for (int to : group)
                {
                    if (to == from)
                        continue;
                    weight_t c = dist[(_sites[to].y - r.y0) * r.width() + (_sites[to].x - r.x0)];
                    if (c < unreachable)
                        _adj[from].push_back({to, c});
                }
            }

        // lower bound of the cost of one manhattan unit: every step moves |dx| + |dy| units for its cost
        _unit_cost = unreachable;
        for (int s = 0; s < map.identity.length; ++s)
        {
            const int span = std::abs(map.identity.horizontal[s]) + std::abs(map.identity.vertical[s]);
            _unit_cost = std::min<weight_t>(_unit_cost, map.identity.g_weight[s] / span);
        }
        if (_unit_cost >= unreachable || map.identity.length == 0)
            _unit_cost = 0;

        // buffers for the queries
        const std::size_t slots = _sites.size() + 2;
        _query = 0;
        _best.assign(slots, unreachable);
        _goal_cost.assign(slots, unreachable);
        _prev.assign(slots, -1);
        _best_stamp.assign(slots, 0);
        _goal_stamp.assign(slots, 0);

        _built = true;
        return true;
    }

    /// @return true once build() succeeded
    bool is_built() const
    {
        return _built;
    }

    /// @return the number of abstract nodes (entrance cells)
    std::size_t node_count() const
    {
        return _sites.size();
    }

    /// @return the cluster side the graph was built with
    int cluster_size() const
    {
        return _cs;
    }

    /**
     * @brief find a path
     * @param from the start
     * @param to the goal
     * @param out receives the cells from start to goal (both included), cleared before
     * @param cost_out optional, receives the cost of the path
     * @return false if the goal can not be reached, or a cell is locked or outside of the map
     */
    template <typename list_type>
    bool find(const ISite &from, const ISite &to, list_type &out, weight_t *cost_out = nullptr)
    {
        out.clear();
        if (!_built || !inside(from) || !inside(to) || _map->has_lock(from) || _map->has_lock(to))
            return false;
        if (from.x == to.x && from.y == to.y)
        {
            out.push_back(from);
            if (cost_out)
                *cost_out = 0;
            return true;
        }

        const int n_nodes = static_cast<int>(_sites.size());
        const int S = n_nodes, G = n_nodes + 1;
        std::vector<weight_t> dist;
        std::vector<int> parent;

        // The query state lives in reusable buffers: a query id marks the entries that belong to this query, so the
        // work of a query depends on the clusters of the start and of the goal, not on the size of the whole graph.
        ++_query;
        std::vector<edge> from_start; // temporary edges S -> entrance nodes of the start cluster (and S -> G)
        auto seen = [&](int node) {
            if (_best_stamp[node] != _query)
            {
                _best_stamp[node] = _query;
                _best[node] = unreachable;
                _prev[node] = -1;
            }
        };

        // temporary edges of the start and of the goal
        auto attach = [&](const ISite &cell, bool outgoing) {
            local_dijkstra(cell, dist, parent);
            rect r = cluster_rect(cell);
            for (int y = r.y0; y <= r.y1; ++y)
                for (int x = r.x0; x <= r.x1; ++x)
                {
                    weight_t c = dist[(y - r.y0) * r.width() + (x - r.x0)];
                    if (c >= unreachable)
                        continue;
                    int node = _node_of[index_of({x, y})];
                    if (node >= 0)
                    {
                        if (outgoing)
                            from_start.push_back({node, c});
                        else
                        {
                            _goal_stamp[node] = _query;
                            _goal_cost[node] = c;
                        }
                    }
                    if (x == to.x && y == to.y && outgoing && same_cluster(from, to))
                        from_start.push_back({G, c});
                }
        };
        attach(from, true);
        attach(to, false);

        // A* over the abstract graph. The heuristic is a lower bound of the real cost (a step moves at most
        // two cells in the manhattan metric), so the cost stays exact.
        auto heuristic = [&](int node) -> weight_t {
            if (node == G)
                return 0;
            const ISite &site = node == S ? from : _sites[node];
            return _unit_cost * (std::abs(site.x - to.x) + std::abs(site.y - to.y));
        };
        immune_system::cost_queue open;
        seen(S);
        _best[S] = 0;
        open.push({heuristic(S), S});
        while (!open.empty())
        {
            immune_system::cost_entry e = open.top();
            open.pop();
            if (e.cost != _best[e.id] + heuristic(e.id))
                continue; // stale entry
            if (e.id == G)
                break;
            const weight_t g = _best[e.id];
            auto relax = [&](const edge &ed) {
                weight_t c = g + ed.cost;
                seen(ed.to);
                if (c < _best[ed.to])
                {
                    _best[ed.to] = c;
                    _prev[ed.to] = e.id;
                    open.push({c + heuristic(ed.to), ed.to});
                }
            };
            if (e.id == S)
                for (const edge &ed : from_start)
                    relax(ed);
            else
            {
                for (const edge &ed : _adj[e.id])
                    relax(ed);
                if (_goal_stamp[e.id] == _query)
                    relax({G, _goal_cost[e.id]});
            }
        }
        seen(G);
        if (_best[G] >= unreachable)
            return _sparse ? fallback(from, to, out, cost_out) : false;

        // refine the abstract path into cells
        std::vector<int> chain;
        for (int n = G; n != -1; n = _prev[n])
            chain.push_back(n);
        std::reverse(chain.begin(), chain.end());
        auto site_of = [&](int n) {
            return n == S ? from : (n == G ? to : _sites[n]);
        };

        out.push_back(from);
        for (std::size_t i = 1; i < chain.size(); ++i)
        {
            ISite a = site_of(chain[i - 1]), b = site_of(chain[i]);
            if (a.x == b.x && a.y == b.y)
                continue;
            if (!same_cluster(a, b))
            {
                out.push_back(b); // border step
                continue;
            }
            // in-cluster hop: walk the exact local path
            local_dijkstra(a, dist, parent);
            rect r = cluster_rect(a);
            std::vector<ISite> hop;
            for (int cell = (b.y - r.y0) * r.width() + (b.x - r.x0); cell != -1; cell = parent[cell])
                hop.push_back({r.x0 + cell % r.width(), r.y0 + cell / r.width()});
            for (std::size_t k = hop.size() - 1; k-- > 0;)
                out.push_back(hop[k]);
        }
        if (cost_out)
            *cost_out = _best[G];
        return true;
    }

private:
    // sparse graphs may miss a route: search the whole map with Dijkstra. The neurons of the map are not touched.
    template <typename list_type>
    bool fallback(const ISite &from, const ISite &to, list_type &out, weight_t *cost_out)
    {
        const std::size_t total = static_cast<std::size_t>(_w) * _h;
        std::vector<weight_t> dist(total, unreachable);
        std::vector<int> parent(total, -1);

        immune_system::cost_queue open;

        const int start = static_cast<int>(index_of(from)), goal = static_cast<int>(index_of(to));
        dist[start] = 0;
        open.push({0, start});
        while (!open.empty())
        {
            immune_system::cost_entry e = open.top();
            open.pop();
            if (e.cost != dist[e.id])
                continue;
            if (e.id == goal)
                break;
            ISite cur {e.id % _w, e.id / _w};
            for (int s = 0; s < _map->identity.length; ++s)
            {
                ISite nb {cur.x + _map->identity.horizontal[s], cur.y + _map->identity.vertical[s]};
                if (!inside(nb) || _map->has_lock(nb) || !_map->_find_can_step(cur, s))
                    continue;
                const weight_t c = e.cost + _map->identity.g_weight[s];
                const int nb_cell = static_cast<int>(index_of(nb));
                if (c < dist[nb_cell])
                {
                    dist[nb_cell] = c;
                    parent[nb_cell] = e.id;
                    open.push({c, nb_cell});
                }
            }
        }
        if (dist[goal] >= unreachable)
            return false;

        std::vector<ISite> cells;
        for (int cell = goal; cell != -1; cell = parent[cell])
            cells.push_back(ISite {cell % _w, cell / _w});
        for (auto it = cells.rbegin(); it != cells.rend(); ++it)
            out.push_back(*it);
        if (cost_out)
            *cost_out = dist[goal];
        return true;
    }

    static constexpr weight_t unreachable = immune_system::unreachable_cost;

    struct edge
    {
        int to;
        weight_t cost;
    };
    struct rect
    {
        int x0, y0, x1, y1;
        int width() const
        {
            return x1 - x0 + 1;
        }
        int height() const
        {
            return y1 - y0 + 1;
        }
    };

    bool inside(const ISite &s) const
    {
        return s.x >= 0 && s.y >= 0 && s.x < _w && s.y < _h;
    }
    std::size_t index_of(const ISite &s) const
    {
        return static_cast<std::size_t>(s.y) * _w + s.x;
    }
    int clusters_x() const
    {
        return (_w + _cs - 1) / _cs;
    }
    int clusters_y() const
    {
        return (_h + _cs - 1) / _cs;
    }
    std::size_t cluster_index(const ISite &s) const
    {
        return static_cast<std::size_t>(s.y / _cs) * clusters_x() + s.x / _cs;
    }
    bool same_cluster(const ISite &a, const ISite &b) const
    {
        return cluster_index(a) == cluster_index(b);
    }
    rect cluster_rect(const ISite &s) const
    {
        rect r;
        r.x0 = (s.x / _cs) * _cs;
        r.y0 = (s.y / _cs) * _cs;
        r.x1 = std::min(_w - 1, r.x0 + _cs - 1);
        r.y1 = std::min(_h - 1, r.y0 + _cs - 1);
        return r;
    }

    // a legal step from a (free) cell into a free cell of another cluster
    bool cross_step(const ISite &a, int s) const
    {
        ISite b {a.x + _map->identity.horizontal[s], a.y + _map->identity.vertical[s]};
        return inside(b) && !_map->has_lock(b) && !same_cluster(a, b) && _map->_find_can_step(a, s);
    }

    // Dijkstra from src limited to the cluster of src; dist/parent are indexed by the local cell (row-major)
    void local_dijkstra(const ISite &src, std::vector<weight_t> &dist, std::vector<int> &parent) const
    {
        rect r = cluster_rect(src);
        const int rw = r.width();
        dist.assign(static_cast<std::size_t>(rw) * r.height(), unreachable);
        parent.assign(dist.size(), -1);

        immune_system::cost_queue open;

        int start = (src.y - r.y0) * rw + (src.x - r.x0);
        dist[start] = 0;
        open.push({0, start});
        while (!open.empty())
        {
            immune_system::cost_entry e = open.top();
            open.pop();
            if (e.cost != dist[e.id])
                continue;
            ISite cur {r.x0 + e.id % rw, r.y0 + e.id / rw};
            for (int s = 0; s < _map->identity.length; ++s)
            {
                ISite nb {cur.x + _map->identity.horizontal[s], cur.y + _map->identity.vertical[s]};
                if (nb.x < r.x0 || nb.x > r.x1 || nb.y < r.y0 || nb.y > r.y1 || _map->has_lock(nb)
                    || !_map->_find_can_step(cur, s))
                    continue;
                weight_t c = e.cost + _map->identity.g_weight[s];
                int nb_cell = (nb.y - r.y0) * rw + (nb.x - r.x0);
                if (c < dist[nb_cell])
                {
                    dist[nb_cell] = c;
                    parent[nb_cell] = e.id;
                    open.push({c, nb_cell});
                }
            }
        }
    }

    map_type *_map = nullptr;
    int _cs = 8, _w = 0, _h = 0;
    bool _built = false;
    bool _sparse = false;
    std::vector<ISite> _sites;
    std::vector<int> _node_of;
    std::vector<std::vector<edge>> _adj;

    // scratch buffers of find(), indexed by the abstract node (the last two are the start and the goal)
    std::uint32_t _query = 0;
    weight_t _unit_cost = 0; // lower bound of the cost per manhattan unit, the heuristic of the abstract search
    std::vector<weight_t> _best, _goal_cost;
    std::vector<int> _prev;
    std::vector<std::uint32_t> _best_stamp, _goal_stamp;
};

#endif
