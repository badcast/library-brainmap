/************************************************************************   *** *** *** *** *** *** *** *** ***  ***   *********************
 *                                                                          ***                                  ***
 * ASTAR Algorithm composed to RoninEngine (Navigation AI)                  ***              badcast             ***
 * creation date: 18.01.2023                                                ***                                  ***
 * author: badcast <lmecomposer@gmail.com>                                  *** *** *** *** *** *** *** *** ***  ***
 * C++17 minimum required
 * License: GPLv3
 *
 * ENG
 *
 *      The fastest and most preferable type of heuristic can depend on the specific problem, input data, and requirements. Each heuristic
 * function has its own characteristics and can be effective in certain situations. However, in general, the heuristic that more accurately
 * estimates the remaining cost and better guides the search algorithm will be preferable.
 *
 * Here are some general recommendations for choosing a heuristic function:
 *
 *     Pythagore distance: This heuristic is often effective and preferable when movement between states is unconstrained and obstacle-free.
 *It is based on the geometric distance between points and can be a good choice for tasks where accuracy and execution speed are important.
 *
 *     Manhattan distance: This heuristic works well in grid-based environments where movement is limited to horizontal and vertical
 * directions. It is usually straightforward to compute and can be effective for tasks where obstacles and constraints have geometric
 * limitations.
 *
 *     Chebyshev distance: This heuristic uses the maximum difference between coordinates and works well when diagonal as well as
 * horizontal/vertical movements are allowed. It can be useful for tasks where diagonal movements are possible or when differences in the
 *cost of movements in different directions need to be considered.
 *
 * It is important to perform testing and compare the performance of different heuristic functions on your specific data to choose the most
 * suitable one for your task.
 *
 * RUS
 *
 *     Самый быстрый и предпочтительный тип эвристики может зависеть от конкретной задачи, входных данных и требований. Каждая эвристическая
 * функция имеет свои особенности и может быть эффективной в определенных ситуациях. Однако, в общем случае, эвристика, которая более точно
 * оценивает оставшуюся стоимость и лучше направляет алгоритм поиска, будет предпочтительной.
 *
 * Вот некоторые общие рекомендации для выбора эвристической функции:
 *
 *     Фифагорово расстояние (Euclidean distance): Эта эвристика обычно является эффективной и предпочтительной, когда движение между
 * состояниями осуществляется без ограничений и без препятствий. Она основана на геометрическом расстоянии между точками и может быть
 *хорошим выбором для задач, где точность и скорость выполнения важны.
 *
 *     Расстояние по Манхэттену (Manhattan distance): Эта эвристика хорошо работает в сеточных средах, где движение осуществляется только по
 * горизонтали и вертикали. Она обычно проста в вычислении и может быть эффективной для задач, где преграды и препятствия имеют
 *геометрические ограничения.
 *
 *     Расстояние Чебышева (Chebyshev distance): Эта эвристика использует максимальную разность между координатами и хорошо работает в
 *случаях, когда разрешены перемещения по диагонали и горизонтали/вертикали. Она может быть полезной для задач, где возможны перемещения по
 *диагонали или когда нужно учесть различия в стоимости перемещений в разных направлениях.
 *
 * Важно провести тестирование и сравнить производительность разных эвристических функций на ваших конкретных данных, чтобы выбрать наиболее
 * подходящую для вашей задачи.
 *
 *  ** see HeuristicMethod
 *  ** basic_across_map::set_heuristic(HeuristicMethod)
 *
 *******************************************************************************************************************************************/

#ifndef _ACROSS_HPP_
#define _ACROSS_HPP_

#include <cstdint>
#include <cmath>
#include <vector>
#include <set>
#include <stdexcept>
#include <cstring>
#include <algorithm>

#define ACROSS_TEMPLATE template <typename ISite, typename INeuron>
#define ACROSS_DEFINE basic_brain_map<ISite, INeuron>

namespace across
{
    enum class NavigationStatus
    {
        Undefined,
        Locked,
        Closed,
        Opened
    };

    enum class MatrixIdentity
    {
        DiagonalMethod = 0,
        PlusMethod = 1,
        CrossMethod = 2
    };

    enum class HeuristicMethod
    {
        Invalid = -1,
        Pythagorean = 0,
        Manhattan = 1,
        Chebyshev = 2
    };

    typedef std::uint32_t weight_t;

    struct the_neuron
    {
        std::uint8_t flags;
        weight_t g;         // - стоимость пути от начальной_точки до текущей_точки
        weight_t h;         // - оценка эвристики (обычно расстояние до конечной_точки)
        weight_t f;         // - общая оценка стоимости (f = g + h)
        the_neuron *parent; // родитель
    };

    struct the_site
    {
        int x;
        int y;
    };

    bool operator==(const the_site &lhs, const the_site &rhs);

    bool operator!=(const the_site &lhs, const the_site &rhs);

    struct brain_breakfast
    {
        weight_t widthSpace;
        weight_t heightSpace;
        int len;
        void *data;
        brain_breakfast() : widthSpace(0), heightSpace(0), len(0), data(nullptr)
        {
        }
    };

    ACROSS_TEMPLATE
    class immune_system;
    ACROSS_TEMPLATE
    class basic_brain_map;

    template <typename IList>
    struct navigate_result
    {
        NavigationStatus status;
        typename IList::value_type from;
        typename IList::value_type to;
        IList connections;
    };

    ACROSS_TEMPLATE
    class basic_brain_map
    {
    public:
        friend class immune_system<ISite, INeuron>;

        template <typename T>
        using basic_list = std::vector<T>;

        using site_type = ISite;
        using neuron_type = INeuron;
        using immune_system = immune_system<ISite, INeuron>;
        using list_site = basic_list<ISite>;
        using list_neuron = basic_list<INeuron *>;
        using result_site = navigate_result<list_site>;
        using result_neuron = navigate_result<list_neuron>;

        explicit basic_brain_map(int xlength, int ylength);

        ~basic_brain_map();

        inline void clear(bool clearLocks = false);
        inline void fill(bool fillLocks = false);
        inline void randomize_hardware(int flagFilter = 0xdeadbeff);
        inline std::size_t size();
        inline int reserved_bits();

        inline bool set_identity(MatrixIdentity identity);

        inline bool set_heuristic(HeuristicMethod method);
        inline HeuristicMethod get_heuristic();

        inline int get_width();
        inline int get_height();

        inline INeuron *get(int x, int y);
        inline INeuron *get(const ISite &point);

        inline bool contains(const ISite &site);
        inline bool contains(const INeuron *neuron);

        inline INeuron *front();
        inline INeuron *back();

        // pointer with point
        inline bool has_lock(const ISite &site);
        inline void set_lock(const ISite &site, const bool lock_state);

        // pointer with pointer
        inline bool has_lock(const INeuron *neuron);
        inline void set_lock(const INeuron *neuron, const bool lock_state);

        inline const ISite get_point(const INeuron *neuron);

        template <typename ListType = result_site>
        bool find(navigate_result<ListType> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);
        template <typename ListType = result_site>
        bool find(navigate_result<ListType> &navResult, const ISite &first, const ISite &last);

        inline void load(const brain_breakfast &);
        inline void save(brain_breakfast *);

        inline std::size_t get_cached_size();

    protected:
        INeuron *neurons;
        weight_t (*__heuristic__)(const ISite &, const ISite &);
        int _seg_off;
        int _xsize, _ysize;

        struct
        {
            int length;
            std::int8_t *horizontal;
            std::int8_t *vertical;
            std::int8_t *g_weight;
        } identity;

        void _internal_realloc();
    };

#include "across_impl.hpp"

    typedef basic_brain_map<the_site, the_neuron> Brain;

} // namespace across

#undef ACROSS_TEMPLATE
#undef ACROSS_DEFINE

#endif
