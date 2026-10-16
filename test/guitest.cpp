#include <gtk/gtk.h>
#include <brain_map.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
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
        {HeuristicMethod::Pythagorean, "Pythagorean"},
        {HeuristicMethod::Chebyshev, "Chebyshev"},
        {HeuristicMethod::Octile, "Octile"},
    };

    struct IdentityItem
    {
        MatrixIdentity identity;
        const char *label;
    };

    constexpr IdentityItem kIdentities[] = {
        {MatrixIdentity::PlusMethod, "Plus (4-направления)"},
        {MatrixIdentity::DiagonalMethod, "Diagonal (8-направлений)"},
        {MatrixIdentity::CrossMethod, "Cross (4-диагонали)"},
    };

    enum class BrushMode
    {
        ToggleWall = 0,
        MoveStart = 1,
        MoveEnd = 2
    };

    struct GuiAppState
    {
        std::unique_ptr<brain_map> map;
        brain_map::result_site path_result;
        std::vector<std::uint8_t> search_flags;

        the_site start_site {0, 0};
        the_site end_site {50, 50};

        MazeAlgorithm current_algorithm = MazeAlgorithm::RecursiveBacktracer;
        HeuristicMethod current_heuristic = HeuristicMethod::Manhattan;
        MatrixIdentity current_identity = MatrixIdentity::PlusMethod;
        BrushMode brush_mode = BrushMode::ToggleWall;
        bool show_explored = true;
        bool drag_lock_value = true;
        double drag_start_x = 0.0;
        double drag_start_y = 0.0;

        double gen_time_ms = 0.0;
        double path_time_ms = 0.0;

        GtkWidget *drawing_area = nullptr;
        GtkWidget *status_label = nullptr;
        GtkWidget *alg_dropdown = nullptr;
        GtkWidget *heur_dropdown = nullptr;
        GtkWidget *id_dropdown = nullptr;
        GtkWidget *brush_dropdown = nullptr;
        GtkWidget *width_spin = nullptr;
        GtkWidget *height_spin = nullptr;
        GtkWidget *explored_check = nullptr;
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

    void update_status(GuiAppState *state)
    {
        if(!state || !state->map || !state->status_label)
            return;

        std::size_t open_cells = 0;
        std::size_t locked_cells = 0;
        std::size_t explored_cells = 0;
        std::size_t total = state->map->size();

        for(std::size_t i = 0; i < total; ++i)
        {
            const the_neuron *n = state->map->front() + i;
            if(state->map->has_lock(n))
                ++locked_cells;
            else
                ++open_cells;

            if(i < state->search_flags.size() &&
               (state->search_flags[i] == NEURON_CAPTURE_OPEN_LIST ||
                state->search_flags[i] == NEURON_CAPTURE_CLOSED_LIST))
            {
                ++explored_cells;
            }
        }

        const char *status_str = "Undefined";
        switch(state->path_result.status)
        {
            case NavigationStatus::Opened:
                status_str = "Opened (Путь найден)";
                break;
            case NavigationStatus::Closed:
                status_str = "Closed (Нет пути)";
                break;
            case NavigationStatus::Locked:
                status_str = "Locked (Старт/Финиш заблокирован)";
                break;
            default:
                break;
        }

        char buf[512];
        std::snprintf(
            buf,
            sizeof(buf),
            "Статус: %s  |  Длина пути: %zu  |  Исследовано A*: %zu  |  Открыто: %zu / Стен: %zu  |  Генерация: %.2f мс  |  A*: %.2f мс",
            status_str,
            state->path_result.connections.size(),
            explored_cells,
            open_cells,
            locked_cells,
            state->gen_time_ms,
            state->path_time_ms);

        gtk_label_set_text(GTK_LABEL(state->status_label), buf);
    }

    void run_pathfinding(GuiAppState *state)
    {
        if(!state || !state->map)
            return;

        state->map->set_heuristic(state->current_heuristic);
        state->map->set_identity(state->current_identity);
        state->map->clear(false);
        state->path_result.connections.clear();

        auto t0 = std::chrono::steady_clock::now();
        state->map->find(state->path_result, state->start_site, state->end_site);
        auto t1 = std::chrono::steady_clock::now();
        state->path_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        std::size_t total = state->map->size();
        state->search_flags.resize(total);
        const the_neuron *base = state->map->front();
        for(std::size_t i = 0; i < total; ++i)
        {
            state->search_flags[i] = base[i].flags;
        }

        state->map->clear(false);
        update_status(state);

        if(state->drawing_area)
            gtk_widget_queue_draw(state->drawing_area);
    }

    void generate_maze(GuiAppState *state)
    {
        if(!state)
            return;

        int w = state->width_spin ? gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(state->width_spin)) : 51;
        int h = state->height_spin ? gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(state->height_spin)) : 51;
        w = std::max(5, w);
        h = std::max(5, h);

        if(!state->map || static_cast<int>(state->map->get_width()) != w || static_cast<int>(state->map->get_height()) != h)
        {
            state->map = std::make_unique<brain_map>(static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h));
        }

        the_site gen_start {0, 0};
        the_site gen_end {w - 1, h - 1};

        auto t0 = std::chrono::steady_clock::now();
        state->map->create_maze_ex(state->current_algorithm, gen_start, gen_end);
        auto t1 = std::chrono::steady_clock::now();
        state->gen_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        state->start_site = find_nearest_open_site(*state->map, {0, 0});
        state->end_site = find_nearest_open_site(*state->map, {w - 1, h - 1});

        run_pathfinding(state);
    }

    bool pixel_to_site(GuiAppState *state, double px, double py, the_site &out_site)
    {
        if(!state || !state->map || !state->drawing_area)
            return false;

        int area_w = gtk_widget_get_width(state->drawing_area);
        int area_h = gtk_widget_get_height(state->drawing_area);
        int map_w = static_cast<int>(state->map->get_width());
        int map_h = static_cast<int>(state->map->get_height());
        if(map_w <= 0 || map_h <= 0 || area_w <= 0 || area_h <= 0)
            return false;

        double pad = 12.0;
        double avail_w = std::max(1.0, area_w - 2.0 * pad);
        double avail_h = std::max(1.0, area_h - 2.0 * pad);
        double cell_size = std::min(avail_w / map_w, avail_h / map_h);
        double grid_w = cell_size * map_w;
        double grid_h = cell_size * map_h;
        double offset_x = (area_w - grid_w) * 0.5;
        double offset_y = (area_h - grid_h) * 0.5;

        int gx = static_cast<int>(std::floor((px - offset_x) / cell_size));
        int gy = static_cast<int>(std::floor((py - offset_y) / cell_size));
        if(gx < 0 || gy < 0 || gx >= map_w || gy >= map_h)
            return false;

        out_site = {gx, gy};
        return true;
    }

    void on_draw(GtkDrawingArea *, cairo_t *cr, int width, int height, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);

        // Background
        cairo_set_source_rgb(cr, 0.08, 0.09, 0.12);
        cairo_paint(cr);

        if(!state || !state->map)
            return;

        int map_w = static_cast<int>(state->map->get_width());
        int map_h = static_cast<int>(state->map->get_height());
        if(map_w <= 0 || map_h <= 0)
            return;

        double pad = 12.0;
        double avail_w = std::max(1.0, width - 2.0 * pad);
        double avail_h = std::max(1.0, height - 2.0 * pad);
        double cell_size = std::min(avail_w / map_w, avail_h / map_h);
        double grid_w = cell_size * map_w;
        double grid_h = cell_size * map_h;
        double offset_x = (width - grid_w) * 0.5;
        double offset_y = (height - grid_h) * 0.5;

        // Open corridor base background
        cairo_set_source_rgb(cr, 0.13, 0.15, 0.20);
        cairo_rectangle(cr, offset_x, offset_y, grid_w, grid_h);
        cairo_fill(cr);

        const the_neuron *front_n = state->map->front();

        // Draw cells
        for(int x = 0; x < map_w; ++x)
        {
            for(int y = 0; y < map_h; ++y)
            {
                the_site s {x, y};
                double cx = offset_x + x * cell_size;
                double cy = offset_y + y * cell_size;

                if(state->map->has_lock(s))
                {
                    cairo_set_source_rgb(cr, 0.22, 0.26, 0.36);
                    cairo_rectangle(cr, cx, cy, cell_size + 0.35, cell_size + 0.35);
                    cairo_fill(cr);
                }
                else if(state->show_explored)
                {
                    const the_neuron *n = state->map->get(s);
                    std::size_t idx = static_cast<std::size_t>(n - front_n);
                    if(idx < state->search_flags.size())
                    {
                        std::uint8_t fl = state->search_flags[idx];
                        if(fl == NEURON_CAPTURE_CLOSED_LIST)
                        {
                            cairo_set_source_rgba(cr, 0.16, 0.48, 0.72, 0.42);
                            cairo_rectangle(cr, cx, cy, cell_size, cell_size);
                            cairo_fill(cr);
                        }
                        else if(fl == NEURON_CAPTURE_OPEN_LIST)
                        {
                            cairo_set_source_rgba(cr, 0.25, 0.78, 0.88, 0.35);
                            cairo_rectangle(cr, cx, cy, cell_size, cell_size);
                            cairo_fill(cr);
                        }
                    }
                }
            }
        }

        // Draw path cells & smooth line
        if(state->path_result.status == NavigationStatus::Opened && !state->path_result.connections.empty())
        {
            cairo_set_source_rgba(cr, 0.98, 0.78, 0.18, 0.45);
            for(const auto &pt : state->path_result.connections)
            {
                double cx = offset_x + pt.x * cell_size;
                double cy = offset_y + pt.y * cell_size;
                cairo_rectangle(cr, cx, cy, cell_size, cell_size);
                cairo_fill(cr);
            }

            cairo_set_source_rgb(cr, 1.0, 0.84, 0.20);
            cairo_set_line_width(cr, std::max(2.0, cell_size * 0.38));
            cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
            cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);

            bool first = true;
            for(const auto &pt : state->path_result.connections)
            {
                double px = offset_x + (pt.x + 0.5) * cell_size;
                double py = offset_y + (pt.y + 0.5) * cell_size;
                if(first)
                {
                    cairo_move_to(cr, px, py);
                    first = false;
                }
                else
                {
                    cairo_line_to(cr, px, py);
                }
            }
            cairo_stroke(cr);
        }

        // Draw Start marker (Green)
        if(state->map->contains(state->start_site))
        {
            double sx = offset_x + state->start_site.x * cell_size;
            double sy = offset_y + state->start_site.y * cell_size;
            cairo_set_source_rgb(cr, 0.18, 0.88, 0.44);
            cairo_rectangle(cr, sx, sy, cell_size, cell_size);
            cairo_fill(cr);
        }

        // Draw End marker (Red/Coral)
        if(state->map->contains(state->end_site))
        {
            double ex = offset_x + state->end_site.x * cell_size;
            double ey = offset_y + state->end_site.y * cell_size;
            cairo_set_source_rgb(cr, 0.96, 0.28, 0.34);
            cairo_rectangle(cr, ex, ey, cell_size, cell_size);
            cairo_fill(cr);
        }

        // Subtle outer border
        cairo_set_source_rgba(cr, 0.45, 0.52, 0.68, 0.6);
        cairo_set_line_width(cr, 1.5);
        cairo_rectangle(cr, offset_x, offset_y, grid_w, grid_h);
        cairo_stroke(cr);
    }

    void apply_pointer_action(GuiAppState *state, double x, double y, bool is_initial_press)
    {
        the_site site;
        if(!pixel_to_site(state, x, y, site))
            return;

        if(state->brush_mode == BrushMode::MoveStart)
        {
            state->map->set_lock(site, false);
            state->start_site = site;
            run_pathfinding(state);
        }
        else if(state->brush_mode == BrushMode::MoveEnd)
        {
            state->map->set_lock(site, false);
            state->end_site = site;
            run_pathfinding(state);
        }
        else
        {
            if(site == state->start_site || site == state->end_site)
                return;

            if(is_initial_press)
                state->drag_lock_value = !state->map->has_lock(site);

            if(state->map->has_lock(site) != state->drag_lock_value)
            {
                state->map->set_lock(site, state->drag_lock_value);
                run_pathfinding(state);
            }
        }
    }

    void on_drag_begin(GtkGestureDrag *, double start_x, double start_y, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        state->drag_start_x = start_x;
        state->drag_start_y = start_y;
        apply_pointer_action(state, start_x, start_y, true);
    }

    void on_drag_update(GtkGestureDrag *, double offset_x, double offset_y, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        apply_pointer_action(state, state->drag_start_x + offset_x, state->drag_start_y + offset_y, false);
    }

    void on_right_click(GtkGestureClick *, int, double x, double y, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        the_site site;
        if(!pixel_to_site(state, x, y, site))
            return;

        state->map->set_lock(site, false);
        state->end_site = site;
        run_pathfinding(state);
    }

    void on_algorithm_changed(GObject *dropdown, GParamSpec *, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        guint idx = gtk_drop_down_get_selected(GTK_DROP_DOWN(dropdown));
        if(idx < G_N_ELEMENTS(kAlgorithms))
        {
            state->current_algorithm = kAlgorithms[idx].alg;
            generate_maze(state);
        }
    }

    void on_heuristic_changed(GObject *dropdown, GParamSpec *, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        guint idx = gtk_drop_down_get_selected(GTK_DROP_DOWN(dropdown));
        if(idx < G_N_ELEMENTS(kHeuristics))
        {
            state->current_heuristic = kHeuristics[idx].method;
            run_pathfinding(state);
        }
    }

    void on_identity_changed(GObject *dropdown, GParamSpec *, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        guint idx = gtk_drop_down_get_selected(GTK_DROP_DOWN(dropdown));
        if(idx < G_N_ELEMENTS(kIdentities))
        {
            state->current_identity = kIdentities[idx].identity;
            run_pathfinding(state);
        }
    }

    void on_brush_changed(GObject *dropdown, GParamSpec *, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        guint idx = gtk_drop_down_get_selected(GTK_DROP_DOWN(dropdown));
        state->brush_mode = static_cast<BrushMode>(idx);
    }

    void on_explored_toggled(GtkCheckButton *btn, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        state->show_explored = gtk_check_button_get_active(btn);
        if(state->drawing_area)
            gtk_widget_queue_draw(state->drawing_area);
    }

    void on_generate_clicked(GtkButton *, gpointer user_data)
    {
        generate_maze(static_cast<GuiAppState *>(user_data));
    }

    void on_clear_walls_clicked(GtkButton *, gpointer user_data)
    {
        auto *state = static_cast<GuiAppState *>(user_data);
        if(!state || !state->map)
            return;
        state->map->clear(true);
        state->gen_time_ms = 0.0;
        run_pathfinding(state);
    }

    void on_window_destroy(GtkWidget *, gpointer user_data)
    {
        delete static_cast<GuiAppState *>(user_data);
    }

    void on_activate(GtkApplication *app, gpointer)
    {
        auto *state = new GuiAppState();

        GtkWidget *window = gtk_application_window_new(app);
        gtk_window_set_title(GTK_WINDOW(window), "library-brainmap — Генератор лабиринтов и A* Навигация (GTK4)");
        gtk_window_set_default_size(GTK_WINDOW(window), 1180, 820);
        g_signal_connect(window, "destroy", G_CALLBACK(on_window_destroy), state);

        GtkWidget *root_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
        gtk_widget_set_margin_start(root_box, 10);
        gtk_widget_set_margin_end(root_box, 10);
        gtk_widget_set_margin_top(root_box, 10);
        gtk_widget_set_margin_bottom(root_box, 10);
        gtk_window_set_child(GTK_WINDOW(window), root_box);

        // Top controls row 1
        GtkWidget *controls_row1 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
        gtk_box_append(GTK_BOX(root_box), controls_row1);

        // Algorithm selector
        gtk_box_append(GTK_BOX(controls_row1), gtk_label_new("Алгоритм:"));
        const char *alg_labels[G_N_ELEMENTS(kAlgorithms) + 1];
        for(std::size_t i = 0; i < G_N_ELEMENTS(kAlgorithms); ++i)
            alg_labels[i] = kAlgorithms[i].label;
        alg_labels[G_N_ELEMENTS(kAlgorithms)] = nullptr;
        state->alg_dropdown = gtk_drop_down_new_from_strings(alg_labels);
        g_signal_connect(state->alg_dropdown, "notify::selected", G_CALLBACK(on_algorithm_changed), state);
        gtk_box_append(GTK_BOX(controls_row1), state->alg_dropdown);

        // Heuristic selector
        gtk_box_append(GTK_BOX(controls_row1), gtk_label_new("Эвристика:"));
        const char *heur_labels[G_N_ELEMENTS(kHeuristics) + 1];
        for(std::size_t i = 0; i < G_N_ELEMENTS(kHeuristics); ++i)
            heur_labels[i] = kHeuristics[i].label;
        heur_labels[G_N_ELEMENTS(kHeuristics)] = nullptr;
        state->heur_dropdown = gtk_drop_down_new_from_strings(heur_labels);
        g_signal_connect(state->heur_dropdown, "notify::selected", G_CALLBACK(on_heuristic_changed), state);
        gtk_box_append(GTK_BOX(controls_row1), state->heur_dropdown);

        // MatrixIdentity selector
        gtk_box_append(GTK_BOX(controls_row1), gtk_label_new("Матрица:"));
        const char *id_labels[G_N_ELEMENTS(kIdentities) + 1];
        for(std::size_t i = 0; i < G_N_ELEMENTS(kIdentities); ++i)
            id_labels[i] = kIdentities[i].label;
        id_labels[G_N_ELEMENTS(kIdentities)] = nullptr;
        state->id_dropdown = gtk_drop_down_new_from_strings(id_labels);
        g_signal_connect(state->id_dropdown, "notify::selected", G_CALLBACK(on_identity_changed), state);
        gtk_box_append(GTK_BOX(controls_row1), state->id_dropdown);

        // Top controls row 2
        GtkWidget *controls_row2 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
        gtk_box_append(GTK_BOX(root_box), controls_row2);

        gtk_box_append(GTK_BOX(controls_row2), gtk_label_new("Ширина:"));
        state->width_spin = gtk_spin_button_new_with_range(11, 201, 2);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(state->width_spin), 51);
        gtk_box_append(GTK_BOX(controls_row2), state->width_spin);

        gtk_box_append(GTK_BOX(controls_row2), gtk_label_new("Высота:"));
        state->height_spin = gtk_spin_button_new_with_range(11, 201, 2);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(state->height_spin), 51);
        gtk_box_append(GTK_BOX(controls_row2), state->height_spin);

        GtkWidget *gen_btn = gtk_button_new_with_label("Сгенерировать Maze");
        g_signal_connect(gen_btn, "clicked", G_CALLBACK(on_generate_clicked), state);
        gtk_box_append(GTK_BOX(controls_row2), gen_btn);

        GtkWidget *clear_btn = gtk_button_new_with_label("Очистить стены");
        g_signal_connect(clear_btn, "clicked", G_CALLBACK(on_clear_walls_clicked), state);
        gtk_box_append(GTK_BOX(controls_row2), clear_btn);

        gtk_box_append(GTK_BOX(controls_row2), gtk_label_new("Кисть (ЛКМ):"));
        const char *brush_labels[] = {"Стена (Рисовать/Стирать)", "Переместить Старт (Зелёный)", "Переместить Финиш (Красный)", nullptr};
        state->brush_dropdown = gtk_drop_down_new_from_strings(brush_labels);
        g_signal_connect(state->brush_dropdown, "notify::selected", G_CALLBACK(on_brush_changed), state);
        gtk_box_append(GTK_BOX(controls_row2), state->brush_dropdown);

        state->explored_check = gtk_check_button_new_with_label("Показать исследование A*");
        gtk_check_button_set_active(GTK_CHECK_BUTTON(state->explored_check), TRUE);
        g_signal_connect(state->explored_check, "toggled", G_CALLBACK(on_explored_toggled), state);
        gtk_box_append(GTK_BOX(controls_row2), state->explored_check);

        // Drawing canvas
        state->drawing_area = gtk_drawing_area_new();
        gtk_widget_set_hexpand(state->drawing_area, TRUE);
        gtk_widget_set_vexpand(state->drawing_area, TRUE);
        gtk_drawing_area_set_draw_func(GTK_DRAWING_AREA(state->drawing_area), on_draw, state, nullptr);
        gtk_box_append(GTK_BOX(root_box), state->drawing_area);

        // Mouse drag controller (left button)
        GtkGesture *drag = gtk_gesture_drag_new();
        gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(drag), GDK_BUTTON_PRIMARY);
        g_signal_connect(drag, "drag-begin", G_CALLBACK(on_drag_begin), state);
        g_signal_connect(drag, "drag-update", G_CALLBACK(on_drag_update), state);
        gtk_widget_add_controller(state->drawing_area, GTK_EVENT_CONTROLLER(drag));

        // Right-click controller to quickly move End position
        GtkGesture *right_click = gtk_gesture_click_new();
        gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(right_click), GDK_BUTTON_SECONDARY);
        g_signal_connect(right_click, "pressed", G_CALLBACK(on_right_click), state);
        gtk_widget_add_controller(state->drawing_area, GTK_EVENT_CONTROLLER(right_click));

        // Bottom status bar
        state->status_label = gtk_label_new("");
        gtk_widget_set_halign(state->status_label, GTK_ALIGN_START);
        gtk_box_append(GTK_BOX(root_box), state->status_label);

        generate_maze(state);
        gtk_window_present(GTK_WINDOW(window));
    }
} // namespace

int main(int argc, char **argv)
{
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    GtkApplication *app = gtk_application_new("org.ronin.brainmap.guitest", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(on_activate), nullptr);
    int status = g_application_run(G_APPLICATION(app), argc, argv);
    g_object_unref(app);
    return status;
}
