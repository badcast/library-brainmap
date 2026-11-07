# library-brainmap {#mainpage}

[English](../en/index.html) | [Русский](../ru/index.html)

![Поиск пути и генерация лабиринтов](preview.gif)

**library-brainmap** — header-only библиотека на C++17 для **генерации лабиринтов** и **поиска пути** на сетке «нейронов».
В комплекте есть интерактивный визуализатор на ImGui (`guitest`), который анимирует каждый шаг поиска.

## Возможности

- Алгоритмы поиска пути (`brain::pathfinder_algo`): **A\***, **Dijkstra**, **поиск в ширину**, **поиск в глубину**.
- Однонаправленный и двунаправленный поиск (`brain::search_mode`). BFS и DFS всегда ищут в одну сторону.
- Эвристики (`brain::heuristic_method`): Манхэттен (по умолчанию), Чебышёв, Octile, Hypot, Pythagorean и пользовательская.
- 13 генераторов лабиринтов (`brain::maze_algorithm`): Recursive Backtracker, Recursive Division, Прим, Краскал,
  Эллер, Hunt-and-Kill, Binary Tree, Sidewinder, Олдос-Бродер, Вильсон, Cellular Automata и другие.
- Собственная стоимость шагов, соседство по 4 или 8 направлениям, необязательное срезание углов.
- Сохранение и загрузка всей карты.

## Быстрый старт

```cpp
#include <brain_map.hpp>

using namespace brain;

int main()
{
    brain_map map(51, 51);

    // сгенерировать лабиринт
    map.create_maze_ex(maze_algorithm::RecursiveBacktracer, {0, 0}, {50, 50});

    // найти путь
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

Каждый вызов `find()` начинает с чистого состояния поиска, стены (блокировки) сохраняются.
Алгоритм по умолчанию можно задать один раз через `brain::basic_brain_map::set_pathfinder()`.

## Алгоритмы вкратце

| Алгоритм | Оптимальный путь | Эвристика | Применение |
|---|---|---|---|
| A\* | да (при допустимой эвристике) | да | универсальный и самый быстрый |
| Dijkstra | да | нет | карты с весами без хорошей эвристики |
| BFS | при равной стоимости шагов | нет | простые карты без весов |
| DFS | нет | нет | проверка достижимости, визуализация |

> **Примечание:** по умолчанию используется эвристика Манхэттена: она точна для стандартной стоимости шагов
> (прямой шаг 1, диагональный 2), поэтому A\* даёт ту же стоимость, что и Dijkstra.
> Эвристика Pythagorean (квадрат расстояния) быстрая, но недопустимая.

## Сборка

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

## Справочник API

Начните с класса `brain::basic_brain_map` (псевдоним `brain::brain_map`) и перечислений
`brain::pathfinder_algo`, `brain::search_mode`, `brain::heuristic_method`, `brain::maze_algorithm`.
