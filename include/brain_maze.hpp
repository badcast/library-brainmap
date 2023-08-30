// declarations brain_map part-file
#ifndef _BRAIN_MAZE_HPP_
#define _BRAIN_MAZE_HPP_

#include "brain_map.hpp"

#if not defined(BRAIN_TEMPLATE)
#error "main header not included"
#endif

namespace maze_system
{
    BRAIN_TEMPLATE
    static void maze_recursive_division(BRAIN_DEFINE *map, int l, int r, int t, int b)
    {
        if(r < l || b < t)
            return;

        int rw = r - l;
        int rh = b - t;

        if(rw < 2 || rh < 2)
            return;

        int hs = map->random_number(t + 1, b - 1);
        int vs = map->random_number(l + 1, r - 1);

        // horizontal
        for(int x = l; x <= r; ++x)
        {
            map->set_lock(map->get(x, hs), true);
        }

        // vertical
        for(int y = t; y <= b; ++y)
        {
            map->set_lock(map->get(y, vs), true);
        }

        int dx = map->random_number(l, r);
        int dy = map->random_number(t, b);
        map->set_lock(map->get(dx, dy), false);

        recursive_division(map, l, vs - 1, t, hs - 1);
        recursive_division(map, vs + 1, r, t, hs - 1);
        recursive_division(map, l, vs - 1, hs + 1, b);
        recursive_division(map, vs + 1, r, hs + 1, b);
    }

    BRAIN_TEMPLATE
    static void maze_recursive_backtrace(BRAIN_DEFINE *map)
    {
        static decltype(map->identity) maze_identity;
        static auto __g_bss__ {brain::immune_system::get_matrix(brain::MatrixIdentity::PlusMethod, maze_identity)};

        int dx, dy;
        INeuron *free_neighbours[4];
        INeuron *current, *self;
        ISite current_site, self_site;
        std::vector<INeuron *> stack;

        map->clear();
        map->fill_locks(true);

        current = map->front();
        current->flags = brain::NEURON_CAPTURE_CAPITALIZED;

        do
        {
            current_site = map->get_point(current);

            // get neighbors
            for(dx = dy = 0; dx < maze_identity.length; ++dx)
            {
                // Get the closest neurons
                self_site.x = current_site.x + maze_identity.horizontal[dx] * 2;
                self_site.y = current_site.y + maze_identity.vertical[dx] * 2;
                self = map->get(self_site);
                if(self == nullptr || self->flags == brain::NEURON_CAPTURE_CAPITALIZED)
                {
                    continue;
                }

                free_neighbours[dy++] = self;
            }

            if(!dy)
            {
                current = stack.back();
                stack.pop_back();
            }
            else
            {

                self = free_neighbours[map->random_number(0, dy)];
                self_site = map->get_point(self);

                // Bottom wall
                if(current_site.x == self_site.x)
                {
                    if(current_site.y > self_site.y)
                    {
                        current_site.y--;
                        if(map->contains(current_site))
                            map->set_lock(current_site, false);
                    }
                    else
                    {
                        self_site.y--;
                        if(map->contains(self_site))
                            map->set_lock(self_site, false);
                    }
                }
                // Left wall
                else
                {
                    if(current_site.x > self_site.x)
                    {
                        current_site.x--;
                        if(map->contains(current_site))
                            map->set_lock(current_site, false);
                    }
                    else
                    {
                        self_site.x--;
                        if(map->contains(self_site))
                            map->set_lock(self_site, false);
                    }
                }

                // locked and visited
                self->flags = brain::NEURON_CAPTURE_CAPITALIZED;
                map->set_lock(self, false);
                stack.push_back(self);
                current = self;
            }
        } while(!stack.empty());
    }
    BRAIN_TEMPLATE
    static void maze_recursive_division(BRAIN_DEFINE *map)
    {
        recursive_division(map, 0, map->get_width() - 1, 0, map->get_height() - 1);
    }
}; // namespace maze_system

BRAIN_TEMPLATE
void BRAIN_DEFINE::create_maze()
{
    maze_system::maze_recursive_backtrace(this);
}

#endif
