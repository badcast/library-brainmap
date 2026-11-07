#include <brain_map.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numeric>
#include <deque>
#include <set>
#include <string>
#include <vector>

using namespace brain;

namespace
{
    struct AlgorithmItem
    {
        maze_algorithm alg;
        const char *label;
    };

    constexpr AlgorithmItem kAlgorithms[] = {
        {maze_algorithm::RecursiveBacktracer, "0: Recursive Backtracer (DFS)"},
        {maze_algorithm::RecursiveDivision, "1: Recursive Division"},
        {maze_algorithm::SelectiveStormChaos, "2: Selective Storm Chaos"},
        {maze_algorithm::ForcedChaos, "3: Forced Chaos"},
        {maze_algorithm::RandomizedPrim, "4: Randomized Prim's"},
        {maze_algorithm::RandomizedKruskal, "5: Randomized Kruskal's"},
        {maze_algorithm::Ellers, "6: Eller's Algorithm"},
        {maze_algorithm::HuntAndKill, "7: Hunt-and-Kill"},
        {maze_algorithm::BinaryTree, "8: Binary Tree"},
        {maze_algorithm::Sidewinder, "9: Sidewinder"},
        {maze_algorithm::AldousBroder, "10: Aldous-Broder"},
        {maze_algorithm::Wilsons, "11: Wilson's (Loop-Erased)"},
        {maze_algorithm::CellularAutomata, "12: Cellular Automata (Cave)"},
    };

    struct PathfinderItem
    {
        pathfinder_algo algo;
        const char *label;
        const char *name;
    };

    constexpr PathfinderItem kPathfinders[] = {
        {pathfinder_algo::AStar, "A* (heuristic)", "A*"},
        {pathfinder_algo::Dijkstra, "Dijkstra", "Dijkstra"},
        {pathfinder_algo::BreadthFirstSearch, "Breadth-First Search", "BFS"},
        {pathfinder_algo::DepthFirstSearch, "Depth-First Search", "DFS"},
        {pathfinder_algo::GreedyBestFirst, "Greedy Best-First", "Greedy"},
        {pathfinder_algo::JumpPointSearch, "Jump Point Search", "JPS"},
        {pathfinder_algo::ThetaStar, "Theta* (any-angle)", "Theta*"},
        {pathfinder_algo::IDAStar, "IDA* (iterative)", "IDA*"},
    };

    struct MazeTemplateInfo
    {
        int level_number = 0;
        std::uint32_t score = 0;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::size_t data_len = 0;
        std::string file_path;
        std::string title;
        std::string difficulty; // "EXTREME", "HARD", "MEDIUM", "EASY"
    };

    struct HeuristicItem
    {
        heuristic_method method;
        const char *label;
        const char *short_label;
    };

    constexpr HeuristicItem kHeuristics[] = {
        {heuristic_method::Pythagorean, "Euclidean (dx^2+dy^2)", "Euclidean"},
        {heuristic_method::Hypot, "Hypot (std::hypot)", "Hypot"},
        {heuristic_method::Manhattan, "Manhattan (|dx|+|dy|)", "Manhattan"},
        {heuristic_method::Chebyshev, "Chebyshev (max(|dx|,|dy|))", "Chebyshev"},
        {heuristic_method::Octile, "Octile (diagonal shortcut)", "Octile"},
    };

    struct IdentityItem
    {
        matrix_identity identity;
        const char *label;
        const char *short_label;
    };

    constexpr IdentityItem kIdentities[] = {
        {matrix_identity::PlusMethod, "Plus (4-dir orthogonal)", "4-Dir"},
        {matrix_identity::DiagonalMethod, "Diagonal (8-dir all neighbours)", "8-Dir"},
        {matrix_identity::CrossMethod, "Cross (4-diag diagonal only)", "4-Diag"},
    };

    struct StepChange
    {
        int cell_index;
        std::uint8_t new_flags;
    };

    struct SearchStep
    {
        the_site active_site {-1, -1};
        bool is_backward = false;
        std::vector<StepChange> changes;
        bool path_found = false;
    };

    struct GuiAppState
    {
        std::unique_ptr<brain_map> map;
        brain_map::result_site path_result;
        std::vector<std::uint8_t> search_flags;

        the_site start_site {0, 0};
        the_site end_site {50, 50};

        int algorithm_idx = 0;
        int pathfinder_idx = 0; // A* default
        int maze_gen_idx = -1;  // -1: random maze algorithm
        int heuristic_idx = 0; // Euclidean default
        int identity_idx = 0;  // 4-Dir default

        int width = 51;
        int height = 51;

        bool bidirectional = true;
        bool show_explored = true;
        bool show_flow = false; // flow field overlay (heat map + arrows toward the goal)
        flow_field field;

        // Mouse controls
        bool is_dragging_end = false;   // LMB: Red marker (Goal / end_site)
        bool is_dragging_start = false; // RMB: Green marker (Start / start_site)
        bool is_dragging_wall = false;  // MMB: Wall
        bool wall_drag_value = true;
        the_site last_mouse_site {-1, -1};

        double gen_time_ms = 0.0;
        double path_time_ms = 0.0;

        // Step-by-step playback system
        std::vector<SearchStep> recorded_steps;
        int current_step = 0;
        int total_steps = 0;
        bool is_playing = false;
        float speed = 60.0f; // steps per second
        float step_accumulator = 0.0f;
        the_site active_expand_site {-1, -1};

        // Maze templates
        std::vector<MazeTemplateInfo> templates;
        int current_template_level = -1;
        char filter_buf[64] = "";
        int difficulty_filter = 0; // 0: All, 1: Extreme (1500+), 2: Hard (1200+), 3: Normal/Medium
        int sort_mode = 0;         // 0: Score desc, 1: Level ID asc
    };

    the_site find_nearest_open_site(brain_map &m, the_site target)
    {
        int w = static_cast<int>(m.get_width());
        int h = static_cast<int>(m.get_height());
        target.x = std::clamp(target.x, 0, std::max(0, w - 1));
        target.y = std::clamp(target.y, 0, std::max(0, h - 1));

        if(m.contains(target) && !m.has_lock(target))
            return target;

        for(int radius = 1; radius < std::max(w, h); ++radius)
        {
            for(int dy = -radius; dy <= radius; ++dy)
            {
                for(int dx = -radius; dx <= radius; ++dx)
                {
                    if(std::abs(dx) != radius && std::abs(dy) != radius)
                        continue;
                    the_site candidate {target.x + dx, target.y + dy};
                    if(m.contains(candidate) && !m.has_lock(candidate))
                        return candidate;
                }
            }
        }
        return target;
    }

    void apply_step_display(GuiAppState &state)
    {
        if(state.current_step < 0)
            state.current_step = 0;
        if(state.current_step > state.total_steps)
            state.current_step = state.total_steps;

        std::fill(state.search_flags.begin(), state.search_flags.end(), 0);
        state.active_expand_site = {-1, -1};

        for(int i = 0; i < state.current_step; ++i)
        {
            const auto &st = state.recorded_steps[i];
            for(const auto &ch : st.changes)
            {
                if(ch.cell_index >= 0 && ch.cell_index < static_cast<int>(state.search_flags.size()))
                {
                    state.search_flags[ch.cell_index] = ch.new_flags;
                }
            }
            if(i == state.current_step - 1)
            {
                state.active_expand_site = st.active_site;
            }
        }
    }

    // BFS / DFS have no bidirectional variant (the library falls back to one direction)
    bool effective_bidirectional(const GuiAppState &state)
    {
        pathfinder_algo a = kPathfinders[state.pathfinder_idx].algo;
        return state.bidirectional && (a == pathfinder_algo::AStar || a == pathfinder_algo::Dijkstra);
    }

    // Single-direction steppers for BFS and DFS (mirror _find_bfs/_find_dfs of the library)
    void record_uninformed_steps(GuiAppState &state, bool depth_first, brain_identity &id_matrix)
    {
        the_neuron *base_neuron = state.map->front();
        the_neuron *firstNeuron = state.map->get(state.start_site);
        the_neuron *lastNeuron = state.map->get(state.end_site);

        auto can_step = [&](const the_site &cur, int s) {
            if(id_matrix.horizontal[s] != 0 && id_matrix.vertical[s] != 0)
            {
                return !(state.map->has_lock({cur.x + id_matrix.horizontal[s], cur.y}) ||
                         state.map->has_lock({cur.x, cur.y + id_matrix.vertical[s]}));
            }
            return true;
        };

        state.map->clear(false);

        SearchStep init_step;
        init_step.active_site = state.start_site;
        init_step.changes.push_back({static_cast<int>(firstNeuron - base_neuron), NEURON_CAPTURE_OPEN_LIST});
        state.recorded_steps.push_back(std::move(init_step));

        std::deque<std::pair<the_neuron *, the_neuron *>> frontier; // (neuron, came from)
        frontier.emplace_back(firstNeuron, nullptr);
        firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;

        while(!frontier.empty())
        {
            std::pair<the_neuron *, the_neuron *> item;
            if(depth_first)
            {
                item = frontier.back();
                frontier.pop_back();
                if(item.first->flags == NEURON_CAPTURE_CLOSED_LIST)
                    continue;
            }
            else
            {
                item = frontier.front();
                frontier.pop_front();
            }

            the_neuron *current = item.first;
            current->parent = item.second;
            current->flags = NEURON_CAPTURE_CLOSED_LIST;

            the_site cur_site = state.map->get_point(current);
            SearchStep step;
            step.active_site = cur_site;
            step.changes.push_back({static_cast<int>(current - base_neuron), NEURON_CAPTURE_CLOSED_LIST});

            if(current == lastNeuron)
            {
                step.path_found = true;
                state.recorded_steps.push_back(std::move(step));
                break;
            }

            for(int k = 0; k < id_matrix.length; ++k)
            {
                // DFS pops from the back, so push in reverse to explore the first direction first
                int s = depth_first ? id_matrix.length - 1 - k : k;
                the_site nb_site {cur_site.x + id_matrix.horizontal[s], cur_site.y + id_matrix.vertical[s]};
                the_neuron *self = state.map->get(nb_site);
                if(!self || self->flags == NEURON_CAPTURE_CLOSED_LIST || state.map->has_lock(nb_site) || !can_step(cur_site, s))
                    continue;
                if(!depth_first && self->flags == NEURON_CAPTURE_OPEN_LIST)
                    continue;

                if(self->flags != NEURON_CAPTURE_OPEN_LIST)
                    step.changes.push_back({static_cast<int>(self - base_neuron), NEURON_CAPTURE_OPEN_LIST});
                self->flags = NEURON_CAPTURE_OPEN_LIST;
                if(depth_first)
                    frontier.emplace_back(self, current);
                else
                {
                    self->parent = current;
                    frontier.emplace_back(self, current);
                }
            }
            state.recorded_steps.push_back(std::move(step));
        }
        state.map->clear(false);
    }

    void record_search_steps(GuiAppState &state)
    {
        state.recorded_steps.clear();
        state.active_expand_site = {-1, -1};

        if(!state.map || state.map->size() == 0)
        {
            state.total_steps = 0;
            state.current_step = 0;
            return;
        }

        the_neuron *firstNeuron = state.map->get(state.start_site);
        the_neuron *lastNeuron = state.map->get(state.end_site);

        if(!firstNeuron || !lastNeuron || state.map->has_lock(firstNeuron) || state.map->has_lock(lastNeuron))
        {
            state.total_steps = 0;
            state.current_step = 0;
            return;
        }

        the_neuron *base_neuron = state.map->front();
        std::size_t total = state.map->size();
        brain_identity id_matrix;
        immune_system::get_matrix(kIdentities[state.identity_idx].identity, id_matrix);

        auto heuristic_fn = immune_system::heuristic_manhtattan;
        switch(kHeuristics[state.heuristic_idx].method)
        {
            case heuristic_method::Pythagorean:
                heuristic_fn = immune_system::heuristic_pythagorean;
                break;
            case heuristic_method::Hypot:
                heuristic_fn = immune_system::heuristic_hypot;
                break;
            case heuristic_method::Manhattan:
                heuristic_fn = immune_system::heuristic_manhtattan;
                break;
            case heuristic_method::Chebyshev:
                heuristic_fn = immune_system::heuristic_chebyshev;
                break;
            case heuristic_method::Octile:
                heuristic_fn = immune_system::heuristic_octile;
                break;
            default:
                break;
        }

        if(state.start_site == state.end_site)
        {
            SearchStep s;
            s.active_site = state.start_site;
            s.path_found = true;
            s.changes.push_back({static_cast<int>(firstNeuron - base_neuron), NEURON_CAPTURE_OPEN_LIST});
            state.recorded_steps.push_back(std::move(s));
            state.total_steps = 1;
            return;
        }

        const pathfinder_algo algo = kPathfinders[state.pathfinder_idx].algo;
        if(algo == pathfinder_algo::Dijkstra)
            heuristic_fn = [](weight_t, weight_t) -> weight_t { return 0; };

        if(algo == pathfinder_algo::BreadthFirstSearch || algo == pathfinder_algo::DepthFirstSearch)
        {
            record_uninformed_steps(state, algo == pathfinder_algo::DepthFirstSearch, id_matrix);
        }
        else if(algo == pathfinder_algo::GreedyBestFirst || algo == pathfinder_algo::JumpPointSearch
                || algo == pathfinder_algo::ThetaStar || algo == pathfinder_algo::IDAStar)
        {
            // replay the expansion order reported by the library
            std::vector<the_site> trace;
            brain_map::result_site scratch;
            state.map->set_trace(&trace);
            state.map->find(algo, scratch, state.start_site, state.end_site, search_mode::UniDirectional);
            state.map->set_trace(nullptr);

            SearchStep init_step;
            init_step.active_site = state.start_site;
            init_step.changes.push_back({static_cast<int>(firstNeuron - base_neuron), NEURON_CAPTURE_OPEN_LIST});
            state.recorded_steps.push_back(std::move(init_step));
            for(const the_site &site : trace)
            {
                SearchStep step;
                step.active_site = site;
                step.changes.push_back({static_cast<int>(state.map->get(site) - base_neuron), NEURON_CAPTURE_CLOSED_LIST});
                step.path_found = (site == state.end_site);
                state.recorded_steps.push_back(std::move(step));
            }
            state.map->clear(false);
        }
        else if(!effective_bidirectional(state))
        {
            state.map->clear(false);

            auto compare_fwd = [](const the_neuron *lhs, const the_neuron *rhs) {
                if(lhs->f != rhs->f)
                    return lhs->f < rhs->f;
                return lhs < rhs;
            };
            std::set<the_neuron *, decltype(compare_fwd)> open_set(compare_fwd);

            firstNeuron->g = 0;
            firstNeuron->h = heuristic_fn(state.start_site.x - state.end_site.x, state.start_site.y - state.end_site.y);
            firstNeuron->f = firstNeuron->h;
            firstNeuron->parent = nullptr;
            firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
            open_set.insert(firstNeuron);

            SearchStep init_step;
            init_step.active_site = state.start_site;
            init_step.changes.push_back({static_cast<int>(firstNeuron - base_neuron), NEURON_CAPTURE_OPEN_LIST});
            state.recorded_steps.push_back(std::move(init_step));

            while(!open_set.empty())
            {
                the_neuron *current = *open_set.begin();
                open_set.erase(open_set.begin());
                current->flags = NEURON_CAPTURE_CLOSED_LIST;

                the_site cur_site = state.map->get_point(current);
                SearchStep step;
                step.active_site = cur_site;
                step.changes.push_back({static_cast<int>(current - base_neuron), NEURON_CAPTURE_CLOSED_LIST});

                if(current == lastNeuron)
                {
                    step.path_found = true;
                    state.recorded_steps.push_back(std::move(step));
                    break;
                }

                for(int s = 0; s < id_matrix.length; ++s)
                {
                    the_site nb_site {cur_site.x + id_matrix.horizontal[s], cur_site.y + id_matrix.vertical[s]};
                    the_neuron *self = state.map->get(nb_site);
                    if(!self || self->flags == NEURON_CAPTURE_CLOSED_LIST || state.map->has_lock(nb_site))
                        continue;

                    if(id_matrix.horizontal[s] != 0 && id_matrix.vertical[s] != 0)
                    {
                        if(state.map->has_lock({cur_site.x + id_matrix.horizontal[s], cur_site.y}) ||
                           state.map->has_lock({cur_site.x, cur_site.y + id_matrix.vertical[s]}))
                        {
                            continue;
                        }
                    }

                    int newG = current->g + id_matrix.g_weight[s];
                    int cell_idx = static_cast<int>(self - base_neuron);

                    if(self->flags == NEURON_CAPTURE_OPEN_LIST)
                    {
                        if(newG < self->g)
                        {
                            open_set.erase(self);
                            self->g = newG;
                            self->f = self->g + self->h;
                            self->parent = current;
                            open_set.insert(self);
                        }
                    }
                    else
                    {
                        self->flags = NEURON_CAPTURE_OPEN_LIST;
                        self->parent = current;
                        self->g = newG;
                        self->h = heuristic_fn(nb_site.x - state.end_site.x, nb_site.y - state.end_site.y);
                        self->f = self->g + self->h;
                        open_set.insert(self);
                        step.changes.push_back({cell_idx, NEURON_CAPTURE_OPEN_LIST});
                    }
                }

                state.recorded_steps.push_back(std::move(step));
            }
            state.map->clear(false);
        }
        else
        {
            // BiDirectional Stepper
            state.map->clear(false);

            struct bwd_info_t
            {
                weight_t g = std::numeric_limits<weight_t>::max() / 2;
                weight_t h = 0;
                weight_t f = 0;
                the_neuron *parent = nullptr;
            };

            std::vector<bwd_info_t> bwd_info(total);

            auto compare_fwd = [](const the_neuron *lhs, const the_neuron *rhs) {
                if(lhs->f != rhs->f)
                    return lhs->f < rhs->f;
                return lhs < rhs;
            };
            std::set<the_neuron *, decltype(compare_fwd)> open_fwd(compare_fwd);

            auto compare_bwd = [&bwd_info, base_neuron](const the_neuron *lhs, const the_neuron *rhs) {
                std::size_t li = static_cast<std::size_t>(lhs - base_neuron);
                std::size_t ri = static_cast<std::size_t>(rhs - base_neuron);
                if(bwd_info[li].f != bwd_info[ri].f)
                    return bwd_info[li].f < bwd_info[ri].f;
                return lhs < rhs;
            };
            std::set<the_neuron *, decltype(compare_bwd)> open_bwd(compare_bwd);

            firstNeuron->g = 0;
            firstNeuron->h = heuristic_fn(state.start_site.x - state.end_site.x, state.start_site.y - state.end_site.y);
            firstNeuron->f = firstNeuron->h;
            firstNeuron->parent = nullptr;
            firstNeuron->flags |= NEURON_CAPTURE_OPEN_LIST;
            open_fwd.insert(firstNeuron);

            std::size_t last_idx = static_cast<std::size_t>(lastNeuron - base_neuron);
            bwd_info[last_idx].g = 0;
            bwd_info[last_idx].h = heuristic_fn(state.end_site.x - state.start_site.x, state.end_site.y - state.start_site.y);
            bwd_info[last_idx].f = bwd_info[last_idx].h;
            bwd_info[last_idx].parent = nullptr;
            lastNeuron->flags |= NEURON_CAPTURE_BWD_OPEN_LIST;
            open_bwd.insert(lastNeuron);

            SearchStep init_step;
            init_step.active_site = state.start_site;
            init_step.changes.push_back({static_cast<int>(firstNeuron - base_neuron), firstNeuron->flags});
            init_step.changes.push_back({static_cast<int>(last_idx), lastNeuron->flags});
            state.recorded_steps.push_back(std::move(init_step));

            weight_t best_path_cost = std::numeric_limits<weight_t>::max() / 2;
            the_neuron *meeting_neuron = nullptr;

            while(!open_fwd.empty() && !open_bwd.empty())
            {
                the_neuron *top_fwd = *open_fwd.begin();
                std::size_t top_fwd_idx = static_cast<std::size_t>(top_fwd - base_neuron);
                the_neuron *top_bwd = *open_bwd.begin();
                std::size_t top_bwd_idx = static_cast<std::size_t>(top_bwd - base_neuron);

                if(meeting_neuron != nullptr)
                {
                    if(std::max(top_fwd->f, bwd_info[top_bwd_idx].f) >= best_path_cost)
                    {
                        SearchStep final_step;
                        final_step.active_site = state.map->get_point(meeting_neuron);
                        final_step.path_found = true;
                        state.recorded_steps.push_back(std::move(final_step));
                        break;
                    }
                }

                bool expand_fwd;
                if(top_fwd->f != bwd_info[top_bwd_idx].f)
                    expand_fwd = (top_fwd->f < bwd_info[top_bwd_idx].f);
                else
                    expand_fwd = (open_fwd.size() <= open_bwd.size());

                if(expand_fwd)
                {
                    the_neuron *current = top_fwd;
                    open_fwd.erase(open_fwd.begin());
                    current->flags &= ~NEURON_CAPTURE_OPEN_LIST;
                    current->flags |= NEURON_CAPTURE_CLOSED_LIST;

                    std::size_t cur_idx = static_cast<std::size_t>(current - base_neuron);
                    the_site cur_site = state.map->get_point(current);

                    SearchStep step;
                    step.active_site = cur_site;
                    step.is_backward = false;
                    step.changes.push_back({static_cast<int>(cur_idx), current->flags});

                    if(current->flags & (NEURON_CAPTURE_BWD_OPEN_LIST | NEURON_CAPTURE_BWD_CLOSED_LIST))
                    {
                        weight_t total_g = current->g + bwd_info[cur_idx].g;
                        if(total_g < best_path_cost)
                        {
                            best_path_cost = total_g;
                            meeting_neuron = current;
                        }
                    }

                    for(int s = 0; s < id_matrix.length; ++s)
                    {
                        the_site nb_site {cur_site.x + id_matrix.horizontal[s], cur_site.y + id_matrix.vertical[s]};
                        the_neuron *self = state.map->get(nb_site);
                        if(!self || (self->flags & NEURON_CAPTURE_CLOSED_LIST) || state.map->has_lock(nb_site))
                            continue;

                        if(id_matrix.horizontal[s] != 0 && id_matrix.vertical[s] != 0)
                        {
                            if(state.map->has_lock({cur_site.x + id_matrix.horizontal[s], cur_site.y}) ||
                               state.map->has_lock({cur_site.x, cur_site.y + id_matrix.vertical[s]}))
                            {
                                continue;
                            }
                        }

                        int newG = current->g + id_matrix.g_weight[s];
                        std::size_t self_idx = static_cast<std::size_t>(self - base_neuron);

                        if(self->flags & NEURON_CAPTURE_OPEN_LIST)
                        {
                            if(newG < self->g)
                            {
                                open_fwd.erase(self);
                                self->g = newG;
                                self->f = self->g + self->h;
                                self->parent = current;
                                open_fwd.insert(self);
                            }
                        }
                        else
                        {
                            self->flags |= NEURON_CAPTURE_OPEN_LIST;
                            self->parent = current;
                            self->g = newG;
                            self->h = heuristic_fn(nb_site.x - state.end_site.x, nb_site.y - state.end_site.y);
                            self->f = self->g + self->h;
                            open_fwd.insert(self);
                            step.changes.push_back({static_cast<int>(self_idx), self->flags});
                        }

                        if(self->flags & (NEURON_CAPTURE_BWD_OPEN_LIST | NEURON_CAPTURE_BWD_CLOSED_LIST))
                        {
                            weight_t total_g = self->g + bwd_info[self_idx].g;
                            if(total_g < best_path_cost)
                            {
                                best_path_cost = total_g;
                                meeting_neuron = self;
                            }
                        }
                    }
                    state.recorded_steps.push_back(std::move(step));
                }
                else
                {
                    the_neuron *current = top_bwd;
                    open_bwd.erase(open_bwd.begin());
                    current->flags &= ~NEURON_CAPTURE_BWD_OPEN_LIST;
                    current->flags |= NEURON_CAPTURE_BWD_CLOSED_LIST;

                    std::size_t cur_idx = static_cast<std::size_t>(current - base_neuron);
                    the_site cur_site = state.map->get_point(current);

                    SearchStep step;
                    step.active_site = cur_site;
                    step.is_backward = true;
                    step.changes.push_back({static_cast<int>(cur_idx), current->flags});

                    if(current->flags & (NEURON_CAPTURE_OPEN_LIST | NEURON_CAPTURE_CLOSED_LIST))
                    {
                        weight_t total_g = current->g + bwd_info[cur_idx].g;
                        if(total_g < best_path_cost)
                        {
                            best_path_cost = total_g;
                            meeting_neuron = current;
                        }
                    }

                    for(int s = 0; s < id_matrix.length; ++s)
                    {
                        the_site nb_site {cur_site.x + id_matrix.horizontal[s], cur_site.y + id_matrix.vertical[s]};
                        the_neuron *self = state.map->get(nb_site);
                        if(!self || (self->flags & NEURON_CAPTURE_BWD_CLOSED_LIST) || state.map->has_lock(nb_site))
                            continue;

                        if(id_matrix.horizontal[s] != 0 && id_matrix.vertical[s] != 0)
                        {
                            if(state.map->has_lock({cur_site.x + id_matrix.horizontal[s], cur_site.y}) ||
                               state.map->has_lock({cur_site.x, cur_site.y + id_matrix.vertical[s]}))
                            {
                                continue;
                            }
                        }

                        std::size_t self_idx = static_cast<std::size_t>(self - base_neuron);
                        int newG = bwd_info[cur_idx].g + id_matrix.g_weight[s];

                        if(self->flags & NEURON_CAPTURE_BWD_OPEN_LIST)
                        {
                            if(newG < bwd_info[self_idx].g)
                            {
                                open_bwd.erase(self);
                                bwd_info[self_idx].g = newG;
                                bwd_info[self_idx].f = bwd_info[self_idx].g + bwd_info[self_idx].h;
                                bwd_info[self_idx].parent = current;
                                open_bwd.insert(self);
                            }
                        }
                        else
                        {
                            self->flags |= NEURON_CAPTURE_BWD_OPEN_LIST;
                            bwd_info[self_idx].parent = current;
                            bwd_info[self_idx].g = newG;
                            bwd_info[self_idx].h = heuristic_fn(nb_site.x - state.start_site.x, nb_site.y - state.start_site.y);
                            bwd_info[self_idx].f = bwd_info[self_idx].g + bwd_info[self_idx].h;
                            open_bwd.insert(self);
                            step.changes.push_back({static_cast<int>(self_idx), self->flags});
                        }

                        if(self->flags & (NEURON_CAPTURE_OPEN_LIST | NEURON_CAPTURE_CLOSED_LIST))
                        {
                            weight_t total_g = self->g + bwd_info[self_idx].g;
                            if(total_g < best_path_cost)
                            {
                                best_path_cost = total_g;
                                meeting_neuron = self;
                            }
                        }
                    }
                    state.recorded_steps.push_back(std::move(step));
                }
            }
            state.map->clear(false);
        }

        state.total_steps = static_cast<int>(state.recorded_steps.size());
    }

    void run_pathfinding(GuiAppState &state)
    {
        if(!state.map)
            return;

        state.map->set_heuristic(kHeuristics[state.heuristic_idx].method);
        state.map->set_identity(kIdentities[state.identity_idx].identity);
        state.map->clear(false);
        state.path_result.connections.clear();
        if(state.show_flow)
            state.field.build(*state.map, state.end_site);

        auto t0 = std::chrono::steady_clock::now();
        state.map->find(
            kPathfinders[state.pathfinder_idx].algo,
            state.path_result,
            state.start_site,
            state.end_site,
            effective_bidirectional(state) ? search_mode::BiDirectional : search_mode::UniDirectional);
        auto t1 = std::chrono::steady_clock::now();
        state.path_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::size_t total = state.map->size();
        state.search_flags.resize(total);
        state.map->clear(false);

        // Precompute animated steps for scrubber and playback
        record_search_steps(state);

        if(state.is_playing)
        {
            state.current_step = 0;
        }
        else
        {
            state.current_step = state.total_steps;
        }
        apply_step_display(state);
    }

    std::string find_mazes_directory()
    {
        const char *candidates[] = {
            "test/mazes",
            "mazes",
            "../test/mazes",
            "../../test/mazes",
            "build/test/mazes"
        };
        for(const char *dir : candidates)
        {
            std::error_code ec;
            if(std::filesystem::is_directory(dir, ec))
                return dir;
        }
        return "";
    }

    std::vector<MazeTemplateInfo> scan_maze_templates()
    {
        std::vector<MazeTemplateInfo> result;
        std::string dir = find_mazes_directory();
        if(dir.empty())
            return result;

        namespace fs = std::filesystem;
        std::error_code ec;
        for(const auto &entry : fs::directory_iterator(dir, ec))
        {
            if(!entry.is_regular_file())
                continue;
            std::string filename = entry.path().filename().string();
            if(filename.rfind("level-", 0) != 0 || entry.path().extension() != ".maze")
                continue;

            int level_num = 0;
            if(std::sscanf(filename.c_str(), "level-%d.maze", &level_num) != 1)
                continue;

            std::ifstream f(entry.path().string(), std::ios::in | std::ios::binary);
            if(!f.is_open())
                continue;

            std::uint32_t score = 0;
            brain_breakfast bf;
            if(f.read(reinterpret_cast<char *>(&score), sizeof(score)).good() &&
               f.read(reinterpret_cast<char *>(&bf.widthSpace), sizeof(bf.widthSpace)).good() &&
               f.read(reinterpret_cast<char *>(&bf.heightSpace), sizeof(bf.heightSpace)).good() &&
               f.read(reinterpret_cast<char *>(&bf.len), sizeof(bf.len)).good() && bf.len > 0)
            {
                MazeTemplateInfo tmpl;
                tmpl.level_number = level_num;
                tmpl.score = score;
                tmpl.width = bf.widthSpace;
                tmpl.height = bf.heightSpace;
                tmpl.data_len = static_cast<std::size_t>(bf.len);
                tmpl.file_path = entry.path().string();
                tmpl.title = "Level " + std::to_string(level_num);

                if(score >= 1500)
                    tmpl.difficulty = "EXTREME";
                else if(score >= 1200)
                    tmpl.difficulty = "HARD";
                else if(score >= 800)
                    tmpl.difficulty = "MEDIUM";
                else
                    tmpl.difficulty = "EASY";

                result.push_back(std::move(tmpl));
            }
        }

        std::sort(result.begin(), result.end(), [](const MazeTemplateInfo &a, const MazeTemplateInfo &b) {
            if(a.score != b.score)
                return a.score > b.score;
            return a.level_number < b.level_number;
        });

        return result;
    }

    bool load_template(GuiAppState &state, int template_idx)
    {
        if(template_idx < 0 || template_idx >= static_cast<int>(state.templates.size()))
            return false;

        const auto &tmpl = state.templates[template_idx];
        std::ifstream f(tmpl.file_path, std::ios::in | std::ios::binary);
        if(!f.is_open())
            return false;

        std::uint32_t score = 0;
        brain_breakfast bf;
        if(!(f.read(reinterpret_cast<char *>(&score), sizeof(score)).good() &&
             f.read(reinterpret_cast<char *>(&bf.widthSpace), sizeof(bf.widthSpace)).good() &&
             f.read(reinterpret_cast<char *>(&bf.heightSpace), sizeof(bf.heightSpace)).good() &&
             f.read(reinterpret_cast<char *>(&bf.len), sizeof(bf.len)).good() && bf.len > 0))
        {
            return false;
        }

        std::vector<char> raw_data(bf.len);
        if(!f.read(raw_data.data(), bf.len).good())
            return false;

        bf.data = raw_data.data();

        if(!state.map)
            state.map = std::make_unique<brain_map>(1, 1);

        state.map->load(bf);
        state.width = static_cast<int>(state.map->get_width());
        state.height = static_cast<int>(state.map->get_height());
        state.gen_time_ms = 0.0;
        state.current_template_level = tmpl.level_number;

        state.start_site = find_nearest_open_site(*state.map, {1, 1});
        state.end_site = find_nearest_open_site(*state.map, {state.width - 2, state.height - 2});

        run_pathfinding(state);
        return true;
    }

    void generate_maze(GuiAppState &state)
    {
        int w = std::max(5, state.width);
        int h = std::max(5, state.height);
        state.width = w;
        state.height = h;

        if(!state.map || static_cast<int>(state.map->get_width()) != w || static_cast<int>(state.map->get_height()) != h)
        {
            state.map = std::make_unique<brain_map>(static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h));
        }

        the_site gen_start {0, 0};
        the_site gen_end {w - 1, h - 1};

        maze_algorithm alg = state.maze_gen_idx >= 0
            ? kAlgorithms[state.maze_gen_idx].alg
            : static_cast<maze_algorithm>(std::rand() % 13);
        auto t0 = std::chrono::steady_clock::now();
        state.map->create_maze_ex(alg, gen_start, gen_end);
        auto t1 = std::chrono::steady_clock::now();
        state.gen_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        state.current_template_level = -1;
        state.start_site = find_nearest_open_site(*state.map, {0, 0});
        state.end_site = find_nearest_open_site(*state.map, {w - 1, h - 1});

        run_pathfinding(state);
    }

    void load_fonts(ImGuiIO &io)
    {
        const char *font_paths[] = {
            "/usr/share/fonts/TTF/DejaVuSans.ttf",
            "/usr/share/fonts/noto/NotoSans-Regular.ttf",
            "/usr/share/fonts/liberation/LiberationSans-Regular.ttf",
            nullptr
        };

        for(const char **p = font_paths; *p != nullptr; ++p)
        {
            FILE *f = std::fopen(*p, "rb");
            if(f)
            {
                std::fclose(f);
                ImFontConfig config;
                config.OversampleH = 2;
                config.OversampleV = 2;
                io.Fonts->AddFontFromFileTTF(*p, 13.5f, &config, io.Fonts->GetGlyphRangesCyrillic());
                return;
            }
        }
        io.Fonts->AddFontDefault();
    }
} // namespace

int main(int, char **)
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    if(!glfwInit())
    {
        std::fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow *window = glfwCreateWindow(1360, 840, "library-brainmap - Complex Maze Templates & A* Navigation", nullptr, nullptr);
    if(!window)
    {
        glfwTerminate();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable VSync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.PopupRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.ItemSpacing = ImVec2(5.0f, 3.0f);
    style.ItemInnerSpacing = ImVec2(4.0f, 3.0f);
    style.FramePadding = ImVec2(5.0f, 2.5f);
    style.WindowPadding = ImVec2(5.0f, 5.0f);

    load_fonts(io);

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    GuiAppState state;
    state.templates = scan_maze_templates();
    if(!state.templates.empty())
    {
        load_template(state, 0); // Load #1 most complex maze (Level 214) by default
    }
    else
    {
        generate_maze(state);
    }

    while(!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Step-by-step animation advancement
        if(state.is_playing && state.total_steps > 0)
        {
            state.step_accumulator += io.DeltaTime * state.speed;
            int advance = static_cast<int>(state.step_accumulator);
            if(advance > 0)
            {
                state.current_step = std::min(state.total_steps, state.current_step + advance);
                state.step_accumulator -= static_cast<float>(advance);
                apply_step_display(state);
                if(state.current_step >= state.total_steps)
                {
                    state.is_playing = false;
                }
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImVec2(static_cast<float>(display_w), static_cast<float>(display_h)));
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
                                       ImGuiWindowFlags_NoBringToFrontOnFocus;

        ImGui::Begin("MainLayout", nullptr, window_flags);

        // Compact Control Panel
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.12f, 0.13f, 0.16f, 1.0f));
        ImGui::BeginChild("Controls", ImVec2(0.0f, 62.0f), ImGuiChildFlags_Borders, ImGuiWindowFlags_NoScrollbar);

        // Row 1: Heuristics, Adjacency, Actions (Zero Comboboxes!)
        ImGui::TextDisabled("Heuristic (A*):");
        ImGui::SameLine();
        for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kHeuristics)); ++i)
        {
            bool is_selected = (state.heuristic_idx == i);
            if(is_selected)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.45f, 0.85f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.55f, 0.95f, 1.0f));
            }
            if(ImGui::Button(kHeuristics[i].short_label))
            {
                state.heuristic_idx = i;
                run_pathfinding(state);
            }
            if(is_selected)
                ImGui::PopStyleColor(2);
            if(ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", kHeuristics[i].label);
            ImGui::SameLine();
        }

        ImGui::TextDisabled("| Adjacency:");
        ImGui::SameLine();
        for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kIdentities)); ++i)
        {
            bool is_selected = (state.identity_idx == i);
            if(is_selected)
            {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.18f, 0.45f, 0.85f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.55f, 0.95f, 1.0f));
            }
            if(ImGui::Button(kIdentities[i].short_label))
            {
                state.identity_idx = i;
                run_pathfinding(state);
            }
            if(is_selected)
                ImGui::PopStyleColor(2);
            if(ImGui::IsItemHovered())
                ImGui::SetTooltip("%s", kIdentities[i].label);
            ImGui::SameLine();
        }

        ImGui::TextDisabled("|");
        ImGui::SameLine();
        if(ImGui::Button("Clear"))
        {
            if(state.map)
            {
                state.map->clear(true);
                state.gen_time_ms = 0.0;
                state.current_template_level = -1;
                run_pathfinding(state);
            }
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Clear all walls");

        ImGui::SameLine();
        if(ImGui::Button("Generate"))
        {
            generate_maze(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Generate a maze with the selected generator");

        ImGui::SameLine();
        ImGui::SetNextItemWidth(210.0f);
        {
            const char *maze_preview = state.maze_gen_idx < 0 ? "Random" : kAlgorithms[state.maze_gen_idx].label;
            if(ImGui::BeginCombo("##mazegen", maze_preview))
            {
                if(ImGui::Selectable("Random", state.maze_gen_idx < 0))
                    state.maze_gen_idx = -1;
                for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kAlgorithms)); ++i)
                {
                    bool sel = (state.maze_gen_idx == i);
                    if(ImGui::Selectable(kAlgorithms[i].label, sel))
                    {
                        state.maze_gen_idx = i;
                        generate_maze(state);
                    }
                    if(sel)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Maze generator algorithm");

        // Row 2: Search Mode (Uni / Bi), Explored Checkbox, Playback, Speed, Steps Slider, Steps Progress Bar
        ImGui::SetNextItemWidth(170.0f);
        if(ImGui::BeginCombo("##pathfinder", kPathfinders[state.pathfinder_idx].label))
        {
            for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kPathfinders)); ++i)
            {
                bool sel = (state.pathfinder_idx == i);
                if(ImGui::Selectable(kPathfinders[i].label, sel))
                {
                    state.pathfinder_idx = i;
                    run_pathfinding(state);
                }
                if(sel)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Pathfinder algorithm");

        ImGui::SameLine();
        if(ImGui::RadioButton("1-Way", !state.bidirectional))
        {
            state.bidirectional = false;
            run_pathfinding(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Unidirectional search");

        ImGui::SameLine();
        if(ImGui::RadioButton("2-Way BiDirectional", state.bidirectional))
        {
            state.bidirectional = true;
            run_pathfinding(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Bidirectional search (A*/Dijkstra only; BFS/DFS stay unidirectional)");

        ImGui::SameLine();
        ImGui::Checkbox("A* Grid", &state.show_explored);

        ImGui::SameLine();
        if(ImGui::Checkbox("Flow field", &state.show_flow))
            run_pathfinding(state);
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Dijkstra map toward the goal: heat map + arrows");

        ImGui::SameLine();
        ImGui::Text("|");

        // Playback buttons
        ImGui::SameLine();
        if(ImGui::Button("<<##rewind"))
        {
            state.is_playing = false;
            state.current_step = 0;
            apply_step_display(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Rewind to start (Step 0)");

        ImGui::SameLine();
        if(ImGui::Button("<##step_back"))
        {
            state.is_playing = false;
            state.current_step = std::max(0, state.current_step - 1);
            apply_step_display(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Step backward (-1)");

        ImGui::SameLine();
        const char *play_btn_label = state.is_playing ? "|| Pause" : "|> Play";
        if(ImGui::Button(play_btn_label))
        {
            if(!state.is_playing && state.current_step >= state.total_steps)
            {
                state.current_step = 0;
                apply_step_display(state);
            }
            state.is_playing = !state.is_playing;
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Start / pause search animation");

        ImGui::SameLine();
        if(ImGui::Button(">##step_fwd"))
        {
            state.is_playing = false;
            state.current_step = std::min(state.total_steps, state.current_step + 1);
            apply_step_display(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Step forward (+1)");

        ImGui::SameLine();
        if(ImGui::Button(">>##forward_end"))
        {
            state.is_playing = false;
            state.current_step = state.total_steps;
            apply_step_display(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Fast-forward to end");

        // Speed control
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::SliderFloat("##speed", &state.speed, 1.0f, 300.0f, "%.0f st/s");
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Animation speed (steps per second)");

        // Steps scrubber slider
        ImGui::SameLine();
        ImGui::SetNextItemWidth(130.0f);
        if(ImGui::SliderInt("##steps_scrub", &state.current_step, 0, std::max(0, state.total_steps), "Step %d"))
        {
            state.is_playing = false;
            apply_step_display(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Scrub search steps");

        // Steps visual progress bar
        ImGui::SameLine();
        float progress = state.total_steps > 0 ? static_cast<float>(state.current_step) / static_cast<float>(state.total_steps) : 0.0f;
        char bar_text[64];
        std::snprintf(bar_text, sizeof(bar_text), "%d/%d (%.0f%%)", state.current_step, state.total_steps, progress * 100.0f);
        ImGui::ProgressBar(progress, ImVec2(-1.0f, 18.0f), bar_text);

        ImGui::EndChild();
        ImGui::PopStyleColor();

        // Middle Area: Left (TemplatePanel), Right (CanvasChild)
        float status_bar_height = 24.0f;
        ImVec2 content_avail = ImGui::GetContentRegionAvail();
        content_avail.y -= status_bar_height;

        // Left Sidebar: Complex Maze Templates List
        float sidebar_width = 280.0f;
        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f, 0.11f, 0.14f, 1.0f));
        ImGui::BeginChild("TemplatePanel", ImVec2(sidebar_width, content_avail.y), ImGuiChildFlags_Borders);
        {
            ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "MAZE TEMPLATES (%zu)", state.templates.size());

            if(ImGui::Button("Top Complex (#1)", ImVec2(130.0f, 22.0f)))
            {
                if(!state.templates.empty())
                {
                    int best_idx = 0;
                    for(int i = 1; i < static_cast<int>(state.templates.size()); ++i)
                    {
                        if(state.templates[i].score > state.templates[best_idx].score)
                            best_idx = i;
                    }
                    load_template(state, best_idx);
                }
            }
            if(ImGui::IsItemHovered()) ImGui::SetTooltip("Load the #1 most complex template (Level 214, score 1793)");

            ImGui::SameLine();
            if(ImGui::Button("Random Complex", ImVec2(130.0f, 22.0f)))
            {
                if(!state.templates.empty())
                {
                    int pool = std::min(30, static_cast<int>(state.templates.size()));
                    std::vector<int> top_indices(state.templates.size());
                    std::iota(top_indices.begin(), top_indices.end(), 0);
                    std::sort(top_indices.begin(), top_indices.end(), [&](int a, int b) {
                        return state.templates[a].score > state.templates[b].score;
                    });
                    int pick = top_indices[std::rand() % pool];
                    load_template(state, pick);
                }
            }
            if(ImGui::IsItemHovered()) ImGui::SetTooltip("Load a random high-difficulty maze template");

            ImGui::Separator();
            ImGui::TextDisabled("Sort:");
            ImGui::SameLine();
            if(ImGui::RadioButton("Score v", state.sort_mode == 0))
            {
                state.sort_mode = 0;
                std::sort(state.templates.begin(), state.templates.end(), [](const MazeTemplateInfo &a, const MazeTemplateInfo &b) {
                    if(a.score != b.score) return a.score > b.score;
                    return a.level_number < b.level_number;
                });
            }
            ImGui::SameLine();
            if(ImGui::RadioButton("Level #", state.sort_mode == 1))
            {
                state.sort_mode = 1;
                std::sort(state.templates.begin(), state.templates.end(), [](const MazeTemplateInfo &a, const MazeTemplateInfo &b) {
                    return a.level_number < b.level_number;
                });
            }

            const char *diff_names[] = {"All", "Extreme", "Hard", "Normal"};
            for(int d = 0; d < 4; ++d)
            {
                if(d > 0) ImGui::SameLine();
                bool is_sel = (state.difficulty_filter == d);
                if(is_sel) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.45f, 0.75f, 1.0f));
                if(ImGui::SmallButton(diff_names[d]))
                {
                    state.difficulty_filter = d;
                }
                if(is_sel) ImGui::PopStyleColor();
            }

            ImGui::SetNextItemWidth(-1.0f);
            ImGui::InputTextWithHint("##filter", "Search level #...", state.filter_buf, sizeof(state.filter_buf));

            ImGui::Separator();

            ImGui::BeginChild("TemplateScrollList", ImVec2(0.0f, 0.0f), false);
            for(int i = 0; i < static_cast<int>(state.templates.size()); ++i)
            {
                const auto &tmpl = state.templates[i];

                if(state.difficulty_filter == 1 && tmpl.score < 1500) continue;
                if(state.difficulty_filter == 2 && (tmpl.score < 1200 || tmpl.score >= 1500)) continue;
                if(state.difficulty_filter == 3 && tmpl.score >= 1200) continue;

                if(state.filter_buf[0] != '\0')
                {
                    std::string num_str = std::to_string(tmpl.level_number);
                    std::string score_str = std::to_string(tmpl.score);
                    if(num_str.find(state.filter_buf) == std::string::npos &&
                       score_str.find(state.filter_buf) == std::string::npos &&
                       tmpl.difficulty.find(state.filter_buf) == std::string::npos)
                    {
                        continue;
                    }
                }

                bool is_selected = (state.current_template_level == tmpl.level_number);

                ImVec4 badge_color;
                if(tmpl.score >= 1500)
                    badge_color = ImVec4(1.0f, 0.35f, 0.35f, 1.0f);
                else if(tmpl.score >= 1200)
                    badge_color = ImVec4(1.0f, 0.65f, 0.2f, 1.0f);
                else if(tmpl.score >= 800)
                    badge_color = ImVec4(0.9f, 0.85f, 0.3f, 1.0f);
                else
                    badge_color = ImVec4(0.4f, 0.8f, 0.5f, 1.0f);

                char item_label[128];
                std::snprintf(item_label, sizeof(item_label), "Level %-3d  [%4u pts]", tmpl.level_number, tmpl.score);

                if(ImGui::Selectable(item_label, is_selected))
                {
                    load_template(state, i);
                }

                ImGui::SameLine(ImGui::GetWindowWidth() - 75.0f);
                ImGui::TextColored(badge_color, "%s", tmpl.difficulty.c_str());

                if(ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Level %d: Score %u (%s)\nClick to load this maze template",
                                      tmpl.level_number, tmpl.score, tmpl.difficulty.c_str());
                }
            }
            ImGui::EndChild();
        }
        ImGui::EndChild();
        ImGui::PopStyleColor();

        ImGui::SameLine();

        ImGui::BeginChild("CanvasChild", ImVec2(0.0f, content_avail.y), false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        {
            ImDrawList *draw_list = ImGui::GetWindowDrawList();
            ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
            ImVec2 canvas_sz = ImGui::GetContentRegionAvail();

            // Background canvas
            draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), IM_COL32(20, 23, 31, 255));

            if(state.map && state.map->get_width() > 0 && state.map->get_height() > 0)
            {
                int map_w = static_cast<int>(state.map->get_width());
                int map_h = static_cast<int>(state.map->get_height());

                float pad = 8.0f;
                float avail_w = std::max(1.0f, canvas_sz.x - 2.0f * pad);
                float avail_h = std::max(1.0f, canvas_sz.y - 2.0f * pad);
                float cell_size = std::min(avail_w / map_w, avail_h / map_h);
                float grid_w = cell_size * map_w;
                float grid_h = cell_size * map_h;
                float offset_x = canvas_p0.x + (canvas_sz.x - grid_w) * 0.5f;
                float offset_y = canvas_p0.y + (canvas_sz.y - grid_h) * 0.5f;

                // Base corridor background
                draw_list->AddRectFilled(ImVec2(offset_x, offset_y),
                                         ImVec2(offset_x + grid_w, offset_y + grid_h),
                                         IM_COL32(33, 38, 51, 255));

                const the_neuron *front_n = state.map->front();

                // Draw cells
                for(int x = 0; x < map_w; ++x)
                {
                    for(int y = 0; y < map_h; ++y)
                    {
                        the_site s {x, y};
                        float cx = offset_x + x * cell_size;
                        float cy = offset_y + y * cell_size;

                        if(state.map->has_lock(s))
                        {
                            draw_list->AddRectFilled(
                                ImVec2(cx, cy),
                                ImVec2(cx + cell_size + 0.35f, cy + cell_size + 0.35f),
                                IM_COL32(56, 66, 92, 255));
                        }
                        else if(state.show_explored)
                        {
                            const the_neuron *n = state.map->get(s);
                            std::size_t idx = static_cast<std::size_t>(n - front_n);
                            if(idx < state.search_flags.size())
                            {
                                std::uint8_t fl = state.search_flags[idx];
                                bool is_fwd = (fl & (NEURON_CAPTURE_OPEN_LIST | NEURON_CAPTURE_CLOSED_LIST)) != 0;
                                bool is_bwd = (fl & (NEURON_CAPTURE_BWD_OPEN_LIST | NEURON_CAPTURE_BWD_CLOSED_LIST)) != 0;

                                if(is_fwd && is_bwd)
                                {
                                    draw_list->AddRectFilled(
                                        ImVec2(cx, cy),
                                        ImVec2(cx + cell_size, cy + cell_size),
                                        IM_COL32(255, 140, 0, 210));
                                }
                                else if(fl & NEURON_CAPTURE_CLOSED_LIST)
                                {
                                    draw_list->AddRectFilled(
                                        ImVec2(cx, cy),
                                        ImVec2(cx + cell_size, cy + cell_size),
                                        IM_COL32(41, 130, 205, 135));
                                }
                                else if(fl & NEURON_CAPTURE_OPEN_LIST)
                                {
                                    draw_list->AddRectFilled(
                                        ImVec2(cx, cy),
                                        ImVec2(cx + cell_size, cy + cell_size),
                                        IM_COL32(65, 215, 240, 120));
                                }
                                else if(fl & NEURON_CAPTURE_BWD_CLOSED_LIST)
                                {
                                    draw_list->AddRectFilled(
                                        ImVec2(cx, cy),
                                        ImVec2(cx + cell_size, cy + cell_size),
                                        IM_COL32(195, 60, 225, 135));
                                }
                                else if(fl & NEURON_CAPTURE_BWD_OPEN_LIST)
                                {
                                    draw_list->AddRectFilled(
                                        ImVec2(cx, cy),
                                        ImVec2(cx + cell_size, cy + cell_size),
                                        IM_COL32(255, 115, 235, 120));
                                }
                            }
                        }
                    }
                }

                // Flow field overlay: cost heat map and an arrow toward the goal in every cell
                if(state.show_flow && state.field.is_built())
                {
                    const weight_t max_cost = std::max<weight_t>(1, state.field.max_cost());
                    for(int x = 0; x < map_w; ++x)
                    {
                        for(int y = 0; y < map_h; ++y)
                        {
                            the_site s {x, y};
                            if(state.map->has_lock(s) || !state.field.reachable(s))
                                continue;
                            float t = static_cast<float>(state.field.cost(s)) / static_cast<float>(max_cost);
                            float cx = offset_x + x * cell_size;
                            float cy = offset_y + y * cell_size;
                            draw_list->AddRectFilled(
                                ImVec2(cx, cy),
                                ImVec2(cx + cell_size, cy + cell_size),
                                IM_COL32(static_cast<int>(255 * t), static_cast<int>(200 * (1.0f - t)), 90, 110));
                            the_site d = state.field.direction(s);
                            if(cell_size >= 9.0f && (d.x != 0 || d.y != 0))
                            {
                                ImVec2 c(cx + cell_size * 0.5f, cy + cell_size * 0.5f);
                                ImVec2 e(c.x + d.x * cell_size * 0.38f, c.y + d.y * cell_size * 0.38f);
                                draw_list->AddLine(c, e, IM_COL32(255, 255, 255, 190), 1.5f);
                                draw_list->AddCircleFilled(e, 1.8f, IM_COL32(255, 255, 255, 220));
                            }
                        }
                    }
                }

                // Highlight active expanding node in step mode
                if(state.active_expand_site.x >= 0 && state.current_step < state.total_steps)
                {
                    bool is_bwd = false;
                    if(state.current_step > 0 && state.current_step - 1 < static_cast<int>(state.recorded_steps.size()))
                    {
                        is_bwd = state.recorded_steps[state.current_step - 1].is_backward;
                    }

                    float ax = offset_x + state.active_expand_site.x * cell_size;
                    float ay = offset_y + state.active_expand_site.y * cell_size;
                    ImU32 fill_col = is_bwd ? IM_COL32(255, 110, 240, 190) : IM_COL32(255, 230, 50, 190);
                    draw_list->AddRectFilled(
                        ImVec2(ax, ay),
                        ImVec2(ax + cell_size, ay + cell_size),
                        fill_col);
                    draw_list->AddRect(
                        ImVec2(ax, ay),
                        ImVec2(ax + cell_size, ay + cell_size),
                        IM_COL32(255, 255, 255, 250), 0.0f, 0, 2.0f);
                }

                // Draw path cells & smooth polyline (only when current step has completed the search)
                bool show_path = (state.current_step >= state.total_steps) ||
                                 (state.current_step > 0 && state.recorded_steps[state.current_step - 1].path_found);

                if(show_path && state.path_result.status == navigation_status::Opened && !state.path_result.connections.empty())
                {
                    for(const auto &pt : state.path_result.connections)
                    {
                        float cx = offset_x + pt.x * cell_size;
                        float cy = offset_y + pt.y * cell_size;
                        draw_list->AddRectFilled(
                            ImVec2(cx, cy),
                            ImVec2(cx + cell_size, cy + cell_size),
                            IM_COL32(250, 199, 46, 115));
                    }

                    std::vector<ImVec2> path_points;
                    path_points.reserve(state.path_result.connections.size());
                    for(const auto &pt : state.path_result.connections)
                    {
                        path_points.emplace_back(
                            offset_x + (pt.x + 0.5f) * cell_size,
                            offset_y + (pt.y + 0.5f) * cell_size);
                    }

                    if(path_points.size() > 1)
                    {
                        float line_thick = std::max(2.0f, cell_size * 0.38f);
                        draw_list->AddPolyline(path_points.data(), static_cast<int>(path_points.size()),
                                               IM_COL32(255, 214, 51, 255), 0, line_thick);
                    }
                }

                // Start Marker (Green)
                if(state.map->contains(state.start_site))
                {
                    float sx = offset_x + state.start_site.x * cell_size;
                    float sy = offset_y + state.start_site.y * cell_size;
                    draw_list->AddRectFilled(
                        ImVec2(sx, sy),
                        ImVec2(sx + cell_size, sy + cell_size),
                        IM_COL32(46, 224, 112, 255));
                }

                // End Marker (Red/Coral)
                if(state.map->contains(state.end_site))
                {
                    float ex = offset_x + state.end_site.x * cell_size;
                    float ey = offset_y + state.end_site.y * cell_size;
                    draw_list->AddRectFilled(
                        ImVec2(ex, ey),
                        ImVec2(ex + cell_size, ey + cell_size),
                        IM_COL32(245, 71, 87, 255));
                }

                // Grid Border
                draw_list->AddRect(
                    ImVec2(offset_x, offset_y),
                    ImVec2(offset_x + grid_w, offset_y + grid_h),
                    IM_COL32(115, 133, 173, 153), 0.0f, 0, 1.2f);

                // Mouse Interaction inside Grid
                ImVec2 mouse_pos = io.MousePos;
                bool is_hovered = (mouse_pos.x >= offset_x && mouse_pos.x < offset_x + grid_w &&
                                   mouse_pos.y >= offset_y && mouse_pos.y < offset_y + grid_h);

                if(is_hovered)
                {
                    int gx = static_cast<int>((mouse_pos.x - offset_x) / cell_size);
                    int gy = static_cast<int>((mouse_pos.y - offset_y) / cell_size);
                    the_site hover_site {gx, gy};

                    // Highlight hovered cell
                    float hx = offset_x + gx * cell_size;
                    float hy = offset_y + gy * cell_size;
                    draw_list->AddRect(
                        ImVec2(hx, hy),
                        ImVec2(hx + cell_size, hy + cell_size),
                        IM_COL32(255, 255, 255, 120), 0.0f, 0, 1.0f);

                    // 1. Left Mouse Button -> Red Marker (End / Goal site)
                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                    {
                        state.is_dragging_end = true;
                        state.last_mouse_site = hover_site;
                        state.map->set_lock(hover_site, false);
                        state.end_site = hover_site;
                        run_pathfinding(state);
                    }
                    else if(ImGui::IsMouseDown(ImGuiMouseButton_Left) && state.is_dragging_end)
                    {
                        if(hover_site != state.last_mouse_site)
                        {
                            state.last_mouse_site = hover_site;
                            state.map->set_lock(hover_site, false);
                            state.end_site = hover_site;
                            run_pathfinding(state);
                        }
                    }

                    // 2. Right Mouse Button -> Green Marker (Start site)
                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                    {
                        state.is_dragging_start = true;
                        state.last_mouse_site = hover_site;
                        state.map->set_lock(hover_site, false);
                        state.start_site = hover_site;
                        run_pathfinding(state);
                    }
                    else if(ImGui::IsMouseDown(ImGuiMouseButton_Right) && state.is_dragging_start)
                    {
                        if(hover_site != state.last_mouse_site)
                        {
                            state.last_mouse_site = hover_site;
                            state.map->set_lock(hover_site, false);
                            state.start_site = hover_site;
                            run_pathfinding(state);
                        }
                    }

                    // 3. Middle Mouse Button -> Wall (toggle/draw/erase walls)
                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
                    {
                        if(hover_site != state.start_site && hover_site != state.end_site)
                        {
                            state.is_dragging_wall = true;
                            state.wall_drag_value = !state.map->has_lock(hover_site);
                            state.last_mouse_site = hover_site;
                            state.map->set_lock(hover_site, state.wall_drag_value);
                            run_pathfinding(state);
                        }
                    }
                    else if(ImGui::IsMouseDown(ImGuiMouseButton_Middle) && state.is_dragging_wall)
                    {
                        if(hover_site != state.last_mouse_site && hover_site != state.start_site && hover_site != state.end_site)
                        {
                            state.last_mouse_site = hover_site;
                            state.map->set_lock(hover_site, state.wall_drag_value);
                            run_pathfinding(state);
                        }
                    }
                }

                if(!ImGui::IsMouseDown(ImGuiMouseButton_Left))
                    state.is_dragging_end = false;
                if(!ImGui::IsMouseDown(ImGuiMouseButton_Right))
                    state.is_dragging_start = false;
                if(!ImGui::IsMouseDown(ImGuiMouseButton_Middle))
                    state.is_dragging_wall = false;
            }
        }
        ImGui::EndChild();

        // Compact Bottom Status Bar
        std::size_t open_cells = 0;
        std::size_t locked_cells = 0;
        std::size_t explored_cells = 0;
        std::size_t fwd_cells = 0;
        std::size_t bwd_cells = 0;

        if(state.map)
        {
            std::size_t total = state.map->size();
            for(std::size_t i = 0; i < total; ++i)
            {
                const the_neuron *n = state.map->front() + i;
                if(state.map->has_lock(n))
                    ++locked_cells;
                else
                    ++open_cells;

                if(i < state.search_flags.size())
                {
                    std::uint8_t fl = state.search_flags[i];
                    if(fl & (NEURON_CAPTURE_OPEN_LIST | NEURON_CAPTURE_CLOSED_LIST))
                        ++fwd_cells;
                    if(fl & (NEURON_CAPTURE_BWD_OPEN_LIST | NEURON_CAPTURE_BWD_CLOSED_LIST))
                        ++bwd_cells;
                    if(fl != 0)
                        ++explored_cells;
                }
            }
        }

        const char *status_str = "Undefined";
        ImVec4 status_color = ImVec4(0.7f, 0.7f, 0.7f, 1.0f);
        switch(state.path_result.status)
        {
            case navigation_status::Opened:
                status_str = "Opened (Path found)";
                status_color = ImVec4(0.2f, 0.9f, 0.4f, 1.0f);
                break;
            case navigation_status::Closed:
                status_str = "Closed (No path)";
                status_color = ImVec4(0.95f, 0.3f, 0.3f, 1.0f);
                break;
            case navigation_status::Locked:
                status_str = "Locked (Start/Goal blocked)";
                status_color = ImVec4(1.0f, 0.6f, 0.2f, 1.0f);
                break;
            default:
                break;
        }

        ImGui::Separator();
        ImGui::TextColored(status_color, "%s", status_str);
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.95f, 0.4f, 0.4f, 1.0f), "[LMB: Red (Goal)]");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.3f, 0.9f, 0.45f, 1.0f), "[RMB: Green (Start)]");
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.65f, 0.75f, 0.9f, 1.0f), "[MMB: Wall]");
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        if(state.current_template_level > 0)
        {
            auto it = std::find_if(state.templates.begin(), state.templates.end(), [&](const MazeTemplateInfo &t) {
                return t.level_number == state.current_template_level;
            });
            if(it != state.templates.end())
            {
                ImGui::Text("Template: %s (%u pts) |", it->title.c_str(), it->score);
                ImGui::SameLine();
            }
        }
        if(effective_bidirectional(state))
        {
            ImGui::Text("Path: %zu | %s (Fwd: %zu, Bwd: %zu, Total: %zu / %d steps) | Walls: %zu | Search: %.2f ms [BiDirectional]",
                        state.path_result.connections.size(),
                        kPathfinders[state.pathfinder_idx].name,
                        fwd_cells,
                        bwd_cells,
                        explored_cells,
                        state.total_steps,
                        locked_cells,
                        state.path_time_ms);
        }
        else
        {
            ImGui::Text("Path: %zu | %s: %zu / %d steps | Open: %zu | Walls: %zu | Search: %.2f ms [UniDirectional]",
                        state.path_result.connections.size(),
                        kPathfinders[state.pathfinder_idx].name,
                        explored_cells,
                        state.total_steps,
                        open_cells,
                        locked_cells,
                        state.path_time_ms);
        }

        ImGui::End();

        // Rendering
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.08f, 0.09f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
