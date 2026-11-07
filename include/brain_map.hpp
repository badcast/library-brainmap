/************************************************************************   *** *** *** *** *** *** *** *** ***  ***
 * *********************
 *                                                                          ***                                  ***
 * ASTAR Algorithm composed to RoninEngine (Navigation AI)                  ***         badcast for cast         ***
 * creation date: 18.01.2023                                                ***                                  ***
 * author: badcast <lmecomposer@gmail.com>                                  *** *** *** *** *** *** *** *** ***  ***
 * C++17 minimum required
 * License: GPLv3
 *
 *  ** see HeuristicMethod
 *  ** basic_brain_map::set_heuristic(HeuristicMethod)
 *
 *******************************************************************************************************************************************/

#ifndef _BRAIN_MAP_HPP_
#define _BRAIN_MAP_HPP_

#include <cstdint>
#include <cstdlib>
#include <new>
#include <type_traits>
#include <cmath>
#include <vector>
#include <set>
#include <limits>
#include <stdexcept>
#include <cstring>
#include <algorithm>
#include <functional>
#include <queue>
#include <unordered_map>
#include <tuple>
#include <utility>

#define BRAIN_TEMPLATE template <typename ISite, typename INeuron>
#define BRAIN_DEFINE basic_brain_map<ISite, INeuron>

namespace brain
{
enum class navigation_status
{
    Undefined,
    Locked,
    Closed,
    Opened
};

enum class matrix_identity
{
    DiagonalMethod = 0,
    PlusMethod = 1,
    CrossMethod = 2
};

enum class search_mode
{
    UniDirectional = 0,
    BiDirectional = 1
};

enum class heuristic_method
{
    Invalid = -1,
    Pythagorean = 0,
    Euclidean = 0,
    Manhattan = 1,
    Chebyshev = 2,
    Octile = 3,
    Hypot = 4,
    // A heuristic supplied by the user with set_custom_heuristic()
    Custom = 5
};

enum class maze_algorithm
{
    RecursiveBacktracer = 0,
    RecursiveDivison = 1,
    RecursiveDivision = 1,
    SelectiveStormChaos = 2,
    ForcedChaos = 3,
    RandomizedPrim = 4,
    RandomizedKruskal = 5,
    Ellers = 6,
    HuntAndKill = 7,
    BinaryTree = 8,
    Sidewinder = 9,
    AldousBroder = 10,
    Wilsons = 11,
    CellularAutomata = 12
};

/// Pathfinder algorithms, AStar is the default one
enum class pathfinder_algo
{
    AStar = 0,              ///< best-first search guided by the heuristic, optimal with an admissible heuristic
    Dijkstra = 1,           ///< best-first search without a heuristic, optimal for any non-negative step costs
    BreadthFirstSearch = 2, ///< explores the map layer by layer, optimal for equal step costs
    DepthFirstSearch = 3,   ///< goes as deep as possible first, the path is not optimal
    GreedyBestFirst = 4,    ///< like A*, but sorted by the heuristic only: very fast, the path is not optimal
    JumpPointSearch = 5,    ///< A* with symmetry-breaking jump points on uniform 8-direction grids
    ThetaStar = 6,          ///< any-angle A* with line-of-sight parent relaxation
    IDAStar = 7             ///< iterative-deepening A* with threshold pruning, optimal with an admissible heuristic
};

typedef int weight_t;

// heuristic cost estimate from the (dx, dy) distance to the goal
typedef weight_t (*heuristic_function)(weight_t dx, weight_t dy);

struct the_neuron
{
    std::uint8_t flags;
    std::uint16_t stamp; // low 16 bits of the search id this state belongs to (see basic_brain_map::find)
    weight_t g;          // - стоимость пути от начальной_точки до текущей_точки
    weight_t h;          // - оценка эвристики (обычно расстояние до конечной_точки)
    weight_t f;          // - общая оценка стоимости (f = g + h)
    the_neuron *parent;  //
};

struct the_site
{
    int x;
    int y;
};

struct brain_identity
{
    int length;
    std::int8_t *horizontal;
    std::int8_t *vertical;
    weight_t *g_weight;
};

inline bool operator==(const the_site &lhs, const the_site &rhs);

inline bool operator!=(const the_site &lhs, const the_site &rhs);

inline the_site &operator+=(the_site &lhs, const the_site &rhs);

struct brain_breakfast
{
    /**
     * @brief Horizontal length of data
     */
    std::uint32_t widthSpace;
    /**
     * @brief Vertical length of data
     */
    std::uint32_t heightSpace;
    /**
     * @brief Length of data
     */
    int len;
    /**
     * @brief The bits of locks (raw data)
     */
    void *data;
    /**
     * @brief Default constructor
     */
    brain_breakfast()
        : widthSpace(0)
        , heightSpace(0)
        , len(0)
        , data(nullptr)
    {}
};

BRAIN_TEMPLATE
class basic_brain_map;

BRAIN_TEMPLATE
class basic_flow_field;

BRAIN_TEMPLATE
class basic_hpa_graph;

template <typename IList>
struct navigate_result
{
    navigation_status status;
    typename IList::value_type from;
    typename IList::value_type to;
    IList connections;
};

BRAIN_TEMPLATE
class basic_brain_map
{
    friend class basic_flow_field<ISite, INeuron>;
    friend class basic_hpa_graph<ISite, INeuron>;

public:
    template <typename T>
    using basic_list = std::vector<T>;

    using site_type = ISite;
    using neuron_type = INeuron;
    using list_site = basic_list<ISite>;
    using list_neuron = basic_list<INeuron *>;
    using result_site = navigate_result<list_site>;
    using result_neuron = navigate_result<list_neuron>;
    using randomize_function = std::function<int(void)>;

    explicit basic_brain_map(std::uint32_t xlength, std::uint32_t ylength);

    ~basic_brain_map();

    // The map owns its raw memory: copying would free it twice, use save()/load() to duplicate the locks
    basic_brain_map(const basic_brain_map &) = delete;
    basic_brain_map &operator=(const basic_brain_map &) = delete;
    basic_brain_map(basic_brain_map &&other) noexcept;
    basic_brain_map &operator=(basic_brain_map &&other) noexcept;

    /**
     * @brief get the raw brain data (with locks)
     * @return raw data of the brain map (with locks)
     */
    inline void *data() const;

    /**
     * @brief clear neurons
     * @clearLocks also clear locked data
     */
    inline void clear(bool clearLocks = false);

    /**
     * @brief fill data
     * @param fillLocks can fill locks
     */
    inline void fill(bool fillLocks = false);

    /**
     * @brief fill lock data
     * @param state fill with
     */
    inline void fill_locks(bool state);

    /**
     * @brief randomize_hardware the function for randomized data
     * @param flagFilter the filter only source
     */
    inline void randomize_hardware(int flagFilter = 0xdeadbeff);

    /**
     * @brief set the custom randomize functions
     * @param randomizer The candidate for set (default std::rand())
     */
    inline void set_random_function(const randomize_function &randomizer);

    /**
     * @brief create maze for map
     */
    inline void create_maze();

    /**
     * @brief create maze for map with extensions
     * @param mazeAlgorithm The Algoritm for generate maze system
     * @param startOf The candidate for start generation
     * @param endOf The candidate for end generation
     */
    inline void create_maze_ex(maze_algorithm mazeAlgoritm, ISite startOf = {0, 0}, ISite endOf = {0, 0});

    /**
     * @brief size of the neurons
     * @return size
     */
    inline std::size_t size();

    /**
     * @brief free bits and not used diaposoned from 0 to 7
     * @return free bits
     */
    inline int reserved_bits();

    /**
     * @brief set the identity
     * @see MatrixIdentity
     * @return accepted identity value
     */
    inline bool set_identity(matrix_identity identity);

    /**
     * @brief get the identity method
     * @see MatrixIdentity
     * @return MatrixIdentity value
     */
    inline matrix_identity get_identity();

    /**
     * @brief use own step costs instead of the built-in ones of the matrix identity
     * @param orthogonal cost of a horizontal/vertical step (0 together with diagonal = built-in costs again)
     * @param diagonal cost of a diagonal step
     * @note the costs are applied to the current and to every later identity; keep the heuristic in the same scale
     */
    inline void set_custom_weights(weight_t orthogonal, weight_t diagonal);

    /**
     * @brief use an own heuristic function (get_heuristic() then returns HeuristicMethod::Custom)
     * @return false when the function is null
     */
    inline bool set_custom_heuristic(heuristic_function heuristic);

    /**
     * @brief allow (true) or forbid (false, default) a diagonal step between two locked cells touching at their corner
     */
    inline void set_custom_corner_cutting(bool allow);

    /**
     * @brief get the corner cutting rule
     */
    inline bool get_custom_corner_cutting() const;

    /**
     * @brief set the heuristic method
     * @see HeuristicMethod
     * @return accepted heuristic method value
     */
    inline bool set_heuristic(heuristic_method method);

    /**
     * @brief get the heuristic method
     * @see HeuristicMethod
     * @return get heuristic method value
     */
    inline heuristic_method get_heuristic();

    /**
     * @brief horizontal length of the neurons
     * @return horizontal length
     */
    inline std::uint32_t get_width();

    /**
     * @brief vertical length of the neurons
     * @return vertical length
     */
    inline std::uint32_t get_height();

    /**
     * @brief get the neuron
     * @param x the horizontal site
     * @param y the vertical site
     * @return get neuron pointer
     */
    inline INeuron *get(int x, int y);

    /**
     * @brief get the neuron
     * @param site the neuron site location
     * @return get neuron pointer
     */
    inline INeuron *get(const ISite &site);

    /**
     * @brief contains site location
     * @param site for select
     * @return status contains
     */
    inline bool contains(const ISite &site);

    /**
     * @brief contains neuron
     * @param neuron for select
     * @return status contains
     */
    inline bool contains(const INeuron *neuron);

    /**
     * @brief front neuron (the most first)
     * @return get neuron pointer
     */
    inline INeuron *front();

    /**
     * @brief back neuron (the most last)
     * @return get neuron pointer
     */
    inline INeuron *back();

    /**
     * @brief check location has been locked
     * @param site location for select
     * @return status locked
     */
    inline bool has_lock(const ISite &site);

    /**
     * @brief set lock site location
     * @param site the location for select
     * @param lock_state lock state
     */
    inline void set_lock(const ISite &site, const bool lock_state);

    /**
     * @brief check neuron has been locked
     * @param neuron for select
     * @return status locked
     */
    inline bool has_lock(const INeuron *neuron);

    /**
     * @brief set lock neuron
     * @param site the neuron for select
     * @param lock_state lock state
     */
    inline void set_lock(const INeuron *neuron, const bool lock_state);

    /**
     * @brief convert neuron to location (site) obejct
     * @param select for convertable
     * @return location result
     */
    inline const ISite get_point(const INeuron *neuron);

    /**
     * @brief find neurons for selects
     * @param navResult the result of the selected (for the site result type or the neurons)
     * @param firstNeuron the start of the find to end
     * @param lastNeuron the end of the find to start
     * @return status of the finded
     */
    template <typename list_type = result_site>
    bool find(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool find(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron, search_mode mode);

    /**
     * @brief find a path with the selected algorithm. The search state of the map is reset before the search,
     * locks are kept. BreadthFirstSearch and DepthFirstSearch ignore search_mode::BiDirectional.
     * @param pathfinder the algorithm
     * @param navResult the result of the search
     * @param firstNeuron the start
     * @param lastNeuron the goal
     * @param mode unidirectional or bidirectional search
     * @return true if the path is found
     */
    template <typename list_type = result_site>
    bool find(
        pathfinder_algo pathfinder,
        navigate_result<list_type> &navResult,
        INeuron *firstNeuron,
        INeuron *lastNeuron,
        search_mode mode);

    /**
     * @brief find locations for selects
     * @param navResult the result of the selected (for the site result type or the neurons)
     * @param first the start of the find to end
     * @param last the end of the find to start
     * @return status of the finded
     */
    template <typename list_type = result_site>
    bool find(navigate_result<list_type> &navResult, const ISite &first, const ISite &last);

    template <typename list_type = result_site>
    bool find(navigate_result<list_type> &navResult, const ISite &first, const ISite &last, search_mode mode);

    template <typename list_type = result_site>
    bool find(
        pathfinder_algo pathfinder,
        navigate_result<list_type> &navResult,
        const ISite &first,
        const ISite &last,
        search_mode mode);

    /**
     * @brief record the order in which the next searches expand sites (nullptr disables). Used by visualisers.
     * @param trace the vector to append the expanded sites to, owned by the caller
     */
    inline void set_trace(std::vector<ISite> *trace);

    /**
     * @brief set search mode (UniDirectional or BiDirectional)
     */
    inline void set_search_mode(search_mode mode);

    /**
     * @brief set the default pathfinder algorithm used by find() without an explicit pathfinder
     */
    inline void set_pathfinder(pathfinder_algo pathfinder);

    /**
     * @brief get the default pathfinder algorithm
     */
    inline pathfinder_algo get_pathfinder() const;

    /**
     * @brief get current search mode
     */
    inline search_mode get_search_mode() const;

    /**
     * @brief load breakfast
     */
    inline void load(const brain_breakfast &);

    /**
     * @brief save to breakfast
     */
    inline void save(brain_breakfast *);

    /**
     * @brief cached size of the finded last result
     * @return size of the cache
     */
    inline std::size_t get_cached_size();

    /**
     * @brief Get Neighbours offset selective
     * @param identity method
     * @param offset selective
     * @return the result of the selected (for the site result type or the neurons)
     */
    template <typename list_type = list_site, typename target_type = typename list_type::value_type>
    inline list_type get_neighbours(matrix_identity matrixIdentity, const target_type &from);

    /**
     * @brief Get the random value of min to max
     * @param min the minimum
     * @param max the maximum
     * @return unique randomized value
     */
    inline int random_number(int min, int max);

protected:
    INeuron *neurons = nullptr;
    randomize_function _rand_ = std::rand;
    heuristic_function _heuristic_fn;
    std::uint32_t _seg_off;    // offset of the first neuron (the lock bits come first, padded for the alignment)
    std::uint32_t _lock_bytes; // size of the lock bits in bytes
    std::uint32_t _xsize, _ysize;
    brain_identity identity;
    search_mode _search_mode = search_mode::UniDirectional;
    pathfinder_algo _pathfinder = pathfinder_algo::AStar;
    std::vector<ISite> *_trace = nullptr;
    // set_custom_*: own step costs (indexed like the identity directions), corner cutting rule
    bool _custom_weights = false;
    weight_t _custom_orthogonal = 0, _custom_diagonal = 0;
    weight_t _custom_g_weight[8] {};
    bool _corner_cutting = false;
    // every search gets a new id, so the neuron states of the older searches are stale without a reset of the map
    std::uint32_t _search_id = 0;

    // state of the backward side of the bidirectional searches, indexed like the neurons
    struct bwd_slot
    {
        weight_t g = 0, h = 0, f = 0;
        INeuron *parent = nullptr;
        std::uint32_t stamp = 0;
    };
    std::vector<bwd_slot> _bwd_cache;

    void _begin_search();
    void _touch(INeuron *neuron);
    // get() for the searches: resets the state of a neuron that belongs to an older search
    INeuron *_node(const ISite &site);
    bwd_slot &_bwd_at(std::size_t index);

    void _apply_custom_weights();

    // the lock bit of a cell by its linear index (x * height + y)
    bool _lock_bit(std::size_t index) const;
    void _set_lock_bit(std::size_t index, bool state);

    template <typename list_type = result_site>
    bool _find_unidirectional(
        pathfinder_algo pathfinder, navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_bidirectional(
        pathfinder_algo pathfinder, navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    // Single-direction implementations, one per pathfinder_algo
    template <typename list_type = result_site>
    bool _find_astar(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    // A* (use_heuristic), Dijkstra (!use_heuristic) and Greedy (greedy) share this best-first search
    template <typename list_type = result_site>
    bool _find_best_first(
        navigate_result<list_type> &navResult,
        INeuron *firstNeuron,
        INeuron *lastNeuron,
        bool use_heuristic,
        bool greedy = false);

    template <typename list_type = result_site>
    bool _find_greedy(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_bfs_bidirectional(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_dijkstra(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_bfs(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_dfs(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_jps(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_theta(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    bool _find_ida(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    // shared helpers of the _find_* implementations
    template <typename list_type = result_site>
    bool _find_begin(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    template <typename list_type = result_site>
    void _find_build_path(navigate_result<list_type> &navResult, INeuron *last);

    // writes the chain of nodes (start to goal) to the result in the site or the neuron form
    template <typename list_type = result_site>
    void _find_emit_nodes(navigate_result<list_type> &navResult, const std::vector<INeuron *> &nodes);

    // start == goal: writes the one-node path and returns true
    template <typename list_type = result_site>
    bool _find_trivial(navigate_result<list_type> &navResult, INeuron *firstNeuron, INeuron *lastNeuron);

    bool _find_can_step(const ISite &from, int dir_index);

    void _internal_realloc();
};

#include "brain_impl.hpp"
#include "brain_maze.hpp"
#include "brain_flow_field.hpp"
#include "brain_hpa.hpp"

typedef basic_brain_map<the_site, the_neuron> brain_map;
typedef basic_flow_field<the_site, the_neuron> flow_field;
typedef basic_hpa_graph<the_site, the_neuron> hpa_graph;

} // namespace brain

#undef BRAIN_TEMPLATE
#undef BRAIN_DEFINE

#endif
