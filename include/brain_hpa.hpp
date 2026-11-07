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
     */
    bool build(map_type &map, int cluster_size = 8)
    {
        _built = false;
        if (cluster_size < 2)
            return false;
        _map = &map;
        _cs = cluster_size;
        _w = static_cast<int>(map.get_width());
        _h = static_cast<int>(map.get_height());
        _sites.clear();
        _adj.clear();
        _node_of.assign(static_cast<std::size_t>(_w) * _h, -1);

        // 1. entrance nodes: free cells with a legal step into another cluster
        for (int y = 0; y < _h; ++y)
            for (int x = 0; x < _w; ++x)
            {
                ISite a {x, y};
                if (map.has_lock(a))
                    continue;
                for (int s = 0; s < map.identity.length; ++s)
                    if (cross_step(a, s))
                    {
                        _node_of[index_of(a)] = static_cast<int>(_sites.size());
                        _sites.push_back(a);
                        break;
                    }
            }
        _adj.assign(_sites.size(), {});

        // 2. inter-cluster edges: the border steps
        for (std::size_t n = 0; n < _sites.size(); ++n)
            for (int s = 0; s < map.identity.length; ++s)
                if (cross_step(_sites[n], s))
                {
                    ISite b {_sites[n].x + map.identity.horizontal[s], _sites[n].y + map.identity.vertical[s]};
                    _adj[n].push_back({_node_of[index_of(b)], map.identity.g_weight[s]});
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

        _built = true;
        return true;
    }

    /// @return true once build() succeeded
    bool is_built() const { return _built; }

    /// @return the number of abstract nodes (entrance cells)
    std::size_t node_count() const { return _sites.size(); }

    /// @return the cluster side the graph was built with
    int cluster_size() const { return _cs; }

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
        std::vector<std::vector<edge>> extra(n_nodes + 2);
        std::vector<weight_t> dist;
        std::vector<int> parent;

        // temporary edges of the start and of the goal
        auto attach = [&](const ISite &cell, int virtual_node, bool outgoing) {
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
                        (outgoing ? extra[virtual_node] : extra[node]).push_back({outgoing ? node : virtual_node, c});
                    if (x == to.x && y == to.y && outgoing && same_cluster(from, to))
                        extra[S].push_back({G, c});
                }
        };
        attach(from, S, true);
        attach(to, G, false);

        // Dijkstra over the abstract graph
        struct entry
        {
            weight_t cost;
            int node;
        };
        auto worse = [](const entry &a, const entry &b) { return a.cost > b.cost; };
        std::priority_queue<entry, std::vector<entry>, decltype(worse)> open(worse);
        std::vector<weight_t> best(n_nodes + 2, unreachable);
        std::vector<int> prev(n_nodes + 2, -1);
        best[S] = 0;
        open.push({0, S});
        while (!open.empty())
        {
            entry e = open.top();
            open.pop();
            if (e.cost != best[e.node])
                continue;
            if (e.node == G)
                break;
            auto relax = [&](const edge &ed) {
                weight_t c = e.cost + ed.cost;
                if (c < best[ed.to])
                {
                    best[ed.to] = c;
                    prev[ed.to] = e.node;
                    open.push({c, ed.to});
                }
            };
            if (e.node < n_nodes)
                for (const edge &ed : _adj[e.node])
                    relax(ed);
            for (const edge &ed : extra[e.node])
                relax(ed);
        }
        if (best[G] >= unreachable)
            return false;

        // refine the abstract path into cells
        std::vector<int> chain;
        for (int n = G; n != -1; n = prev[n])
            chain.push_back(n);
        std::reverse(chain.begin(), chain.end());
        auto site_of = [&](int n) { return n == S ? from : (n == G ? to : _sites[n]); };

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
            *cost_out = best[G];
        return true;
    }

private:
    static constexpr weight_t unreachable = std::numeric_limits<weight_t>::max() / 2;

    struct edge
    {
        int to;
        weight_t cost;
    };
    struct rect
    {
        int x0, y0, x1, y1;
        int width() const { return x1 - x0 + 1; }
        int height() const { return y1 - y0 + 1; }
    };

    bool inside(const ISite &s) const { return s.x >= 0 && s.y >= 0 && s.x < _w && s.y < _h; }
    std::size_t index_of(const ISite &s) const { return static_cast<std::size_t>(s.y) * _w + s.x; }
    int clusters_x() const { return (_w + _cs - 1) / _cs; }
    int clusters_y() const { return (_h + _cs - 1) / _cs; }
    std::size_t cluster_index(const ISite &s) const
    {
        return static_cast<std::size_t>(s.y / _cs) * clusters_x() + s.x / _cs;
    }
    bool same_cluster(const ISite &a, const ISite &b) const { return cluster_index(a) == cluster_index(b); }
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

        struct entry
        {
            weight_t cost;
            int cell;
        };
        auto worse = [](const entry &a, const entry &b) { return a.cost > b.cost; };
        std::priority_queue<entry, std::vector<entry>, decltype(worse)> open(worse);

        int start = (src.y - r.y0) * rw + (src.x - r.x0);
        dist[start] = 0;
        open.push({0, start});
        while (!open.empty())
        {
            entry e = open.top();
            open.pop();
            if (e.cost != dist[e.cell])
                continue;
            ISite cur {r.x0 + e.cell % rw, r.y0 + e.cell / rw};
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
                    parent[nb_cell] = e.cell;
                    open.push({c, nb_cell});
                }
            }
        }
    }

    map_type *_map = nullptr;
    int _cs = 8, _w = 0, _h = 0;
    bool _built = false;
    std::vector<ISite> _sites;
    std::vector<int> _node_of;
    std::vector<std::vector<edge>> _adj;
};

#endif
