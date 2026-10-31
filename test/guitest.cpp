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
#include <memory>
#include <set>
#include <string>
#include <vector>

using namespace brain;

namespace
{
    struct AlgorithmItem
    {
        MazeAlgorithm alg;
        const char *label;
    };

    constexpr AlgorithmItem kAlgorithms[] = {
        {MazeAlgorithm::RecursiveBacktracer, "0: Recursive Backtracer (DFS)"},
        {MazeAlgorithm::RecursiveDivision, "1: Recursive Division"},
        {MazeAlgorithm::SelectiveStormChaos, "2: Selective Storm Chaos"},
        {MazeAlgorithm::ForcedChaos, "3: Forced Chaos"},
        {MazeAlgorithm::RandomizedPrim, "4: Randomized Prim's"},
        {MazeAlgorithm::RandomizedKruskal, "5: Randomized Kruskal's"},
        {MazeAlgorithm::Ellers, "6: Eller's Algorithm"},
        {MazeAlgorithm::HuntAndKill, "7: Hunt-and-Kill"},
        {MazeAlgorithm::BinaryTree, "8: Binary Tree"},
        {MazeAlgorithm::Sidewinder, "9: Sidewinder"},
        {MazeAlgorithm::AldousBroder, "10: Aldous-Broder"},
        {MazeAlgorithm::Wilsons, "11: Wilson's (Loop-Erased)"},
        {MazeAlgorithm::CellularAutomata, "12: Cellular Automata (Cave)"},
    };

    struct HeuristicItem
    {
        HeuristicMethod method;
        const char *label;
    };

    constexpr HeuristicItem kHeuristics[] = {
        {HeuristicMethod::Manhattan, "Manhattan"},
        {HeuristicMethod::Pythagorean, "Euclidean (Pythagorean)"},
        {HeuristicMethod::Chebyshev, "Chebyshev"},
        {HeuristicMethod::Octile, "Octile"},
    };

    struct IdentityItem
    {
        MatrixIdentity identity;
        const char *label;
    };

    constexpr IdentityItem kIdentities[] = {
        {MatrixIdentity::PlusMethod, "Plus (4-dir)"},
        {MatrixIdentity::DiagonalMethod, "Diagonal (8-dir)"},
        {MatrixIdentity::CrossMethod, "Cross (4-diag)"},
    };

    enum class BrushMode
    {
        ToggleWall = 0,
        MoveStart = 1,
        MoveEnd = 2
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
        int heuristic_idx = 1;
        int identity_idx = 0;
        int brush_mode_idx = 0;

        int width = 51;
        int height = 51;

        bool bidirectional = true;
        bool show_explored = true;

        bool is_dragging = false;
        bool drag_lock_value = true;
        the_site last_drag_site {-1, -1};

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
            case HeuristicMethod::Pythagorean:
                heuristic_fn = immune_system::heuristic_pythagorean;
                break;
            case HeuristicMethod::Manhattan:
                heuristic_fn = immune_system::heuristic_manhtattan;
                break;
            case HeuristicMethod::Chebyshev:
                heuristic_fn = immune_system::heuristic_chebyshev;
                break;
            case HeuristicMethod::Octile:
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

        if(!state.bidirectional)
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
                    if(std::max(top_fwd->f, bwd_info[top_bwd_idx].f) >= best_path_cost ||
                       (top_fwd->g + bwd_info[top_bwd_idx].g) >= best_path_cost)
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

        auto t0 = std::chrono::steady_clock::now();
        state.map->find(state.path_result, state.start_site, state.end_site, state.bidirectional ? SearchMode::BiDirectional : SearchMode::UniDirectional);
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

        auto t0 = std::chrono::steady_clock::now();
        state.map->create_maze_ex(kAlgorithms[state.algorithm_idx].alg, gen_start, gen_end);
        auto t1 = std::chrono::steady_clock::now();
        state.gen_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        state.start_site = find_nearest_open_site(*state.map, {0, 0});
        state.end_site = find_nearest_open_site(*state.map, {w - 1, h - 1});

        run_pathfinding(state);
    }

    void apply_pointer_action(GuiAppState &state, the_site site, bool is_initial_press)
    {
        if(!state.map || !state.map->contains(site))
            return;

        BrushMode brush = static_cast<BrushMode>(state.brush_mode_idx);
        if(brush == BrushMode::MoveStart)
        {
            state.map->set_lock(site, false);
            state.start_site = site;
            run_pathfinding(state);
        }
        else if(brush == BrushMode::MoveEnd)
        {
            state.map->set_lock(site, false);
            state.end_site = site;
            run_pathfinding(state);
        }
        else
        {
            if(site == state.start_site || site == state.end_site)
                return;

            if(is_initial_press)
                state.drag_lock_value = !state.map->has_lock(site);

            if(state.map->has_lock(site) != state.drag_lock_value)
            {
                state.map->set_lock(site, state.drag_lock_value);
                run_pathfinding(state);
            }
        }
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

    GLFWwindow *window = glfwCreateWindow(1280, 820, "library-brainmap - Maze Generator & A* Navigation (Dear ImGui)", nullptr, nullptr);
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
    (void)io;
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
    generate_maze(state);

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

        // Row 1: Maze Algorithm, Heuristic, Matrix, Dimensions, Actions, Brush
        ImGui::SetNextItemWidth(185.0f);
        if(ImGui::BeginCombo("##alg", kAlgorithms[state.algorithm_idx].label))
        {
            for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kAlgorithms)); ++i)
            {
                bool is_selected = (state.algorithm_idx == i);
                if(ImGui::Selectable(kAlgorithms[i].label, is_selected))
                {
                    state.algorithm_idx = i;
                    generate_maze(state);
                }
                if(is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Maze generation algorithm");

        ImGui::SameLine();
        ImGui::SetNextItemWidth(175.0f);
        if(ImGui::BeginCombo("##heur", kHeuristics[state.heuristic_idx].label))
        {
            for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kHeuristics)); ++i)
            {
                bool is_selected = (state.heuristic_idx == i);
                if(ImGui::Selectable(kHeuristics[i].label, is_selected))
                {
                    state.heuristic_idx = i;
                    run_pathfinding(state);
                }
                if(is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("A* Heuristic");

        ImGui::SameLine();
        ImGui::SetNextItemWidth(115.0f);
        if(ImGui::BeginCombo("##id", kIdentities[state.identity_idx].label))
        {
            for(int i = 0; i < static_cast<int>(IM_ARRAYSIZE(kIdentities)); ++i)
            {
                bool is_selected = (state.identity_idx == i);
                if(ImGui::Selectable(kIdentities[i].label, is_selected))
                {
                    state.identity_idx = i;
                    run_pathfinding(state);
                }
                if(is_selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Graph adjacency (Matrix)");

        ImGui::SameLine();
        ImGui::SetNextItemWidth(45.0f);
        if(ImGui::DragInt("##W", &state.width, 2.0f, 5, 201, "W:%d"))
        {
            state.width = std::clamp(state.width, 5, 201);
        }

        ImGui::SameLine();
        ImGui::SetNextItemWidth(45.0f);
        if(ImGui::DragInt("##H", &state.height, 2.0f, 5, 201, "H:%d"))
        {
            state.height = std::clamp(state.height, 5, 201);
        }

        ImGui::SameLine();
        if(ImGui::Button("Maze"))
        {
            generate_maze(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Generate maze");

        ImGui::SameLine();
        if(ImGui::Button("Clear"))
        {
            if(state.map)
            {
                state.map->clear(true);
                state.gen_time_ms = 0.0;
                run_pathfinding(state);
            }
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Clear all walls");

        ImGui::SameLine();
        const char *brush_labels[] = {
            "Brush: Wall",
            "Brush: Start",
            "Brush: Goal"
        };
        ImGui::SetNextItemWidth(135.0f);
        ImGui::Combo("##brush", &state.brush_mode_idx, brush_labels, IM_ARRAYSIZE(brush_labels));

        // Row 2: Search Mode (Uni / Bi), Explored Checkbox, Playback, Speed, Steps Slider, Steps Progress Bar
        if(ImGui::RadioButton("1-Way A*", !state.bidirectional))
        {
            state.bidirectional = false;
            run_pathfinding(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Standard unidirectional A* search");

        ImGui::SameLine();
        if(ImGui::RadioButton("2-Way BiDirectional", state.bidirectional))
        {
            state.bidirectional = true;
            run_pathfinding(state);
        }
        if(ImGui::IsItemHovered()) ImGui::SetTooltip("Bidirectional A* search (meeting frontiers)");

        ImGui::SameLine();
        ImGui::Checkbox("A* Grid", &state.show_explored);

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

        // Canvas Area
        float status_bar_height = 24.0f;
        ImVec2 canvas_avail = ImGui::GetContentRegionAvail();
        canvas_avail.y -= status_bar_height;

        ImGui::BeginChild("CanvasChild", canvas_avail, false, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
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

                if(show_path && state.path_result.status == NavigationStatus::Opened && !state.path_result.connections.empty())
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

                    // Right click -> Move End
                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Right))
                    {
                        state.map->set_lock(hover_site, false);
                        state.end_site = hover_site;
                        run_pathfinding(state);
                    }

                    // Left click & drag
                    if(ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                    {
                        state.is_dragging = true;
                        state.last_drag_site = hover_site;
                        apply_pointer_action(state, hover_site, true);
                    }
                    else if(ImGui::IsMouseDown(ImGuiMouseButton_Left) && state.is_dragging)
                    {
                        if(hover_site != state.last_drag_site)
                        {
                            state.last_drag_site = hover_site;
                            apply_pointer_action(state, hover_site, false);
                        }
                    }
                }

                if(!ImGui::IsMouseDown(ImGuiMouseButton_Left))
                {
                    state.is_dragging = false;
                    state.last_drag_site = {-1, -1};
                }
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
            case NavigationStatus::Opened:
                status_str = "Opened (Path found)";
                status_color = ImVec4(0.2f, 0.9f, 0.4f, 1.0f);
                break;
            case NavigationStatus::Closed:
                status_str = "Closed (No path)";
                status_color = ImVec4(0.95f, 0.3f, 0.3f, 1.0f);
                break;
            case NavigationStatus::Locked:
                status_str = "Locked (Start/Goal blocked)";
                status_color = ImVec4(1.0f, 0.6f, 0.2f, 1.0f);
                break;
            default:
                break;
        }

        ImGui::Separator();
        ImGui::TextColored(status_color, "%s", status_str);
        ImGui::SameLine();
        if(state.bidirectional)
        {
            ImGui::Text("| Path: %zu | A* (Fwd: %zu, Bwd: %zu, Total: %zu / %d steps) | Walls: %zu | Gen: %.2f ms | A*: %.2f ms [BiDirectional]",
                        state.path_result.connections.size(),
                        fwd_cells,
                        bwd_cells,
                        explored_cells,
                        state.total_steps,
                        locked_cells,
                        state.gen_time_ms,
                        state.path_time_ms);
        }
        else
        {
            ImGui::Text("| Path: %zu | A*: %zu / %d steps | Open: %zu / Walls: %zu | Gen: %.2f ms | A*: %.2f ms [UniDirectional]",
                        state.path_result.connections.size(),
                        explored_cells,
                        state.total_steps,
                        open_cells,
                        locked_cells,
                        state.gen_time_ms,
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
