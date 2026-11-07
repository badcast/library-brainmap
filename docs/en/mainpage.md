# library-brainmap {#mainpage}

[English](../en/index.html) | [Русский](../ru/index.html)

![Path finding and maze generation preview](preview.gif)

**library-brainmap** is a header-only C++17 library for **maze generation** and **path finding** on a grid of "neurons".
It ships with an interactive ImGui visualizer (`guitest`) that animates every search step.

## Features

- Path finders (`brain::pathfinder_algo`): **A\***, **Dijkstra**, **Breadth-First Search**, **Depth-First Search**, **Greedy Best-First**.
- **Flow Field** (Dijkstra map, `brain::flow_field`): one search from the goal gives every cell its cost and next step.
- Unidirectional and bidirectional search (`brain::search_mode`) for A\*, Dijkstra and BFS. DFS and Greedy always search in one direction.
- Heuristics (`brain::heuristic_method`): Manhattan (default), Chebyshev, Octile, Hypot, Pythagorean and a custom one.
- 13 maze generators (`brain::maze_algorithm`): Recursive Backtracker, Recursive Division, Prim, Kruskal, Eller,
  Hunt-and-Kill, Binary Tree, Sidewinder, Aldous-Broder, Wilson, Cellular Automata and others.
- Custom step costs, 4/8-direction adjacency, optional diagonal corner cutting.
- Save and load of the whole map.

## Quick start

```cpp
#include <brain_map.hpp>

using namespace brain;

int main()
{
    brain_map map(51, 51);

    // generate a maze
    map.create_maze_ex(maze_algorithm::RecursiveBacktracer, {0, 0}, {50, 50});

    // find a path
    brain_map::result_site result;
    bool found = map.find(pathfinder_algo::AStar, result, the_site {0, 0}, the_site {50, 50},
                          search_mode::UniDirectional);

    if (found)
    {
        for (const the_site &p : result.connections)
            ; // p.x, p.y
    }
}
```

Every call of `find()` starts from a clean search state, the walls (locks) are kept.
You may also set the default algorithm once with `brain::basic_brain_map::set_pathfinder()`.

## Algorithms at a glance

| Algorithm | Optimal path | Uses heuristic | Typical use |
|---|---|---|---|
| A\* | yes (with an admissible heuristic) | yes | general purpose, fastest |
| Dijkstra | yes | no | weighted maps without a good heuristic |
| BFS | for equal step costs | no | simple, unweighted maps |
| DFS | no | no | reachability, visualisation |
| Greedy Best-First | no | yes (only) | very fast, "good enough" paths |
| Flow Field | yes | no | many agents moving to one goal |

> **Note:** the default heuristic is Manhattan: it is exact for the default step costs
> (orthogonal 1, diagonal 2), so A\* returns the same cost as Dijkstra.
> The Pythagorean heuristic (squared distance) is fast but not admissible.

## Flow field for many agents

```cpp
brain::flow_field field;
field.build(map, the_site {50, 50});        // one Dijkstra run from the goal

brain::the_site step = field.direction({3, 7}); // unit step toward the goal, O(1)
std::vector<brain::the_site> path;
field.path({3, 7}, path);                       // or the whole path
```

Call `build()` again after the map changes.

## Building

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

## API reference

Start with the class `brain::basic_brain_map` (typedef `brain::brain_map`) and the enums
`brain::pathfinder_algo`, `brain::search_mode`, `brain::heuristic_method`, `brain::maze_algorithm`.
