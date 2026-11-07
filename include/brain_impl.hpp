// declarations Brain Map (algorithm) part-file
#ifndef _BRAIN_IMPL_HPP_
#define _BRAIN_IMPL_HPP_

// TODO: Make Bi-Directional

#include "brain_map.hpp"

#if not defined(BRAIN_TEMPLATE)
#error "main header not included"
#endif

#ifndef BRAIN_TEMPLATE
#define BRAIN_TEMPLATE template <typename ISite, typename INeuron>
#endif

#ifndef BRAIN_DEFINE
#define BRAIN_DEFINE basic_brain_map<ISite, INeuron>
#endif

enum
{
    ByteSize = 8
};

enum
{
    // Neutral Neuron
    NEURON_CAPTURE_RELEASED = 0,
    // Captured opened Neuron
    NEURON_CAPTURE_OPEN_LIST = 1,
    // Captured closed Neuron
    NEURON_CAPTURE_CLOSED_LIST = 2,
    // Captured, capitalize Neuron for MAZE
    NEURON_CAPTURE_CAPITALIZED = 4,
    // Captured opened Neuron (Backward search)
    NEURON_CAPTURE_BWD_OPEN_LIST = 8,
    // Captured closed Neuron (Backward search)
    NEURON_CAPTURE_BWD_CLOSED_LIST = 16
};

// site state
#define BLOCK_SITE                                    \
    if constexpr (std::is_same_v<target_type, ISite>) \
    {
// neuron state
#define BLOCK_NEURON                                           \
    }                                                          \
    else if constexpr (std::is_same_v<target_type, INeuron *>) \
    {
#define BLOCK_END }

class brain_range_error : public std::range_error
{
public:
    brain_range_error()
        : std::range_error("range")
    {}
};

namespace immune_system
{
/**
 * Pythagorean distance (dx^2 + dy^2). Not admissible: A* with it is fast but may return non-optimal paths.
 */
static brain::weight_t heuristic_pythagorean(brain::weight_t dx, brain::weight_t dy)
{
    return static_cast<brain::weight_t>(dx * dx + dy * dy);
}

/**
 * Hypot (true Euclidean) distance.
 */
static brain::weight_t heuristic_hypot(brain::weight_t dx, brain::weight_t dy)
{
    return static_cast<brain::weight_t>(std::round(std::hypot(static_cast<double>(dx), static_cast<double>(dy))));
}

/**
 * Manhattan distance.
 */
static brain::weight_t heuristic_manhtattan(brain::weight_t dx, brain::weight_t dy)
{
    return std::abs(dx) + std::abs(dy);
}

/**
 * Chebyshev distance.
 */
static brain::weight_t heuristic_chebyshev(brain::weight_t dx, brain::weight_t dy)
{
    return std::max(std::abs(dx), std::abs(dy));
}

/**
 * Octile distance.
 */
static brain::weight_t heuristic_octile(brain::weight_t dx, brain::weight_t dy)
{
    static const float F = std::sqrt(2.f) - 1;
    dx = std::abs(dx);
    dy = std::abs(dy);
    return (dx < dy) ? F * dx + dy : F * dy + dx;
}

// main comparator for hash_list
template <typename INeuron>
static bool compare_neuron(const INeuron *lhs, const INeuron *rhs)
{
    return lhs->f < rhs->f;
}

template <typename IdentityWrapper>
static int get_matrix(brain::matrix_identity matrixIdentity, IdentityWrapper &place)
{
    /*
     DIAGONAL

     * * *
     * x *
     * * *

    */
    static std::int8_t M_DIAGONAL_H_POINT[] {-1, 0, 1, -1, 1, -1, 0, 1};
    static std::int8_t M_DIAGONAL_V_POINT[] {-1, -1, -1, 0, 0, 1, 1, 1};
    static std::int8_t M_G_DIAGONAL_WEIGHT[] {2, 1, 2, 1, 1, 2, 1, 2};
    /*
     PLUS

       *
     * x *
       *

    */
    static std::int8_t M_PLUS_H_POINT[] {-1, 1, 0, 0};
    static std::int8_t M_PLUS_V_POINT[] {0, 0, -1, 1};
    static std::int8_t M_G_PLUS_WEIGHT[] {1, 1, 1, 1};
    /*
     CROSS

     *   *
       x
     *   *

    */
    static std::int8_t M_CROSS_H_POINT[] {-1, 1, -1, 1};
    static std::int8_t M_CROSS_V_POINT[] {-1, -1, 1, 1};

    switch (matrixIdentity)
    {
        case brain::matrix_identity::DiagonalMethod:
            place.horizontal = M_DIAGONAL_H_POINT;
            place.vertical = M_DIAGONAL_V_POINT;
            place.g_weight = M_G_DIAGONAL_WEIGHT;
            return place.length = sizeof(M_DIAGONAL_H_POINT);
            break;
        case brain::matrix_identity::PlusMethod:
            place.horizontal = M_PLUS_H_POINT;
            place.vertical = M_PLUS_V_POINT;
            place.g_weight = M_G_PLUS_WEIGHT;
            return place.length = sizeof(M_PLUS_H_POINT);
            break;
        case brain::matrix_identity::CrossMethod:
            place.horizontal = M_CROSS_H_POINT;
            place.vertical = M_CROSS_V_POINT;
            place.g_weight = M_G_PLUS_WEIGHT;
            return place.length = sizeof(M_CROSS_H_POINT);
            break;
    }
    return 0;
}
} // namespace immune_system

BRAIN_TEMPLATE
BRAIN_DEFINE::basic_brain_map(std::uint32_t xlength, std::uint32_t ylength)
{
    if (1 > xlength || 1 > ylength)
        throw std::runtime_error("dimensions is zero");

    this->neurons = nullptr;
    this->_xsize = xlength;
    this->_ysize = ylength;

    // The Identity vector lines
    set_identity(matrix_identity::DiagonalMethod);
    // Manhattan is the default: it is exact for the step costs (orthogonal 1, diagonal 2), so A* stays optimal
    set_heuristic(heuristic_method::Manhattan);
    // Alloc
    _internal_realloc();
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::_internal_realloc()
{
    std::size_t xlength = _xsize, ylength = _ysize;

    if (this->neurons != nullptr)
    {
        std::free(this->neurons);
    }

    auto lockedDiv
        = std::lldiv(std::max<std::uint32_t>(ylength = (xlength * ylength), ByteSize), ByteSize); // add locked bits
    this->_seg_off = xlength = lockedDiv.quot + lockedDiv.rem;
    xlength = ylength * sizeof(INeuron) + xlength;
    this->neurons = static_cast<INeuron *>(std::malloc(xlength));

    if (this->neurons == nullptr)
        throw std::bad_alloc();

    // clear allocated data to ZERO
    clear(true);
}

BRAIN_TEMPLATE
BRAIN_DEFINE::~basic_brain_map()
{
    if (this->neurons != nullptr)
        std::free(this->neurons);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::randomize_hardware(int flagFilter)
{
    std::uint32_t lhs, rhs = _seg_off;
    clear(true);
    do
    {
        lhs = static_cast<std::uint32_t>(_rand_() & flagFilter);
        memcpy(
            reinterpret_cast<void *>(reinterpret_cast<std::size_t>(neurons) + _seg_off - rhs),
            &lhs,
            std::min(rhs, (std::uint32_t) sizeof(std::uint32_t)));
        rhs -= std::min(rhs, static_cast<std::uint32_t>(sizeof(std::uint32_t)));
    } while (rhs > 0);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_random_function(const randomize_function &randomizer)
{
    _rand_ = randomizer;
}

BRAIN_TEMPLATE
std::size_t BRAIN_DEFINE::size()
{
    return static_cast<std::size_t>(_xsize) * _ysize;
}

BRAIN_TEMPLATE
int BRAIN_DEFINE::reserved_bits()
{
    int bits = size() % ByteSize;
    if (bits)
        bits = ByteSize - bits;
    return bits;
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::set_identity(matrix_identity matrixIdentity)
{
    int len;
    len = immune_system::get_matrix(matrixIdentity, this->identity);
    this->_apply_custom_weights();
    return len > 0;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::_apply_custom_weights()
{
    if (!_custom_weights)
        return;
    for (int s = 0; s < identity.length && s < 8; ++s)
    {
        const bool diagonal = identity.horizontal[s] != 0 && identity.vertical[s] != 0;
        _custom_g_weight[s] = static_cast<std::int8_t>(diagonal ? _custom_diagonal : _custom_orthogonal);
    }
    identity.g_weight = _custom_g_weight;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_custom_weights(weight_t orthogonal, weight_t diagonal)
{
    _custom_orthogonal = orthogonal;
    _custom_diagonal = diagonal;
    _custom_weights = orthogonal > 0 && diagonal > 0;
    // Back to the built-in costs of the current identity when both are zero
    if (_custom_weights)
        _apply_custom_weights();
    else
        set_identity(get_identity());
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::set_custom_heuristic(heuristic_function heuristic)
{
    if (heuristic == nullptr)
        return false;
    this->__heuristic__ = heuristic;
    return true;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_custom_corner_cutting(bool allow)
{
    this->_corner_cutting = allow;
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::get_custom_corner_cutting() const
{
    return this->_corner_cutting;
}

BRAIN_TEMPLATE
matrix_identity BRAIN_DEFINE::get_identity()
{
    decltype(identity) id;

    int result = static_cast<int>(matrix_identity::DiagonalMethod);
    int last = static_cast<int>(matrix_identity::CrossMethod) + 1;
    for (; result < last;)
    {
        immune_system::get_matrix(matrix_identity(result), id);
        if (id.horizontal == identity.horizontal)
        {
            break;
        }
        ++result;
    }

    return matrix_identity(result);
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::set_heuristic(heuristic_method method)
{
    bool status = true;
    switch (method)
    {
        case heuristic_method::Pythagorean:
            __heuristic__ = immune_system::heuristic_pythagorean;
            break;
        case heuristic_method::Hypot:
            __heuristic__ = immune_system::heuristic_hypot;
            break;
        case heuristic_method::Manhattan:
            __heuristic__ = immune_system::heuristic_manhtattan;
            break;
        case heuristic_method::Chebyshev:
            __heuristic__ = immune_system::heuristic_chebyshev;
            break;
        case heuristic_method::Octile:
            __heuristic__ = immune_system::heuristic_octile;
            break;
        default:
            status = false;
            break;
    }
    return status;
}

BRAIN_TEMPLATE
heuristic_method BRAIN_DEFINE::get_heuristic()
{
    if (__heuristic__ == immune_system::heuristic_pythagorean)
        return heuristic_method::Pythagorean;
    else if (__heuristic__ == immune_system::heuristic_hypot)
        return heuristic_method::Hypot;
    else if (__heuristic__ == immune_system::heuristic_manhtattan)
        return heuristic_method::Manhattan;
    else if (__heuristic__ == immune_system::heuristic_chebyshev)
        return heuristic_method::Chebyshev;
    else if (__heuristic__ == immune_system::heuristic_octile)
        return heuristic_method::Octile;
    else if (__heuristic__ != nullptr)
        return heuristic_method::Custom;
    else
        return heuristic_method::Invalid;
}

BRAIN_TEMPLATE
std::uint32_t BRAIN_DEFINE::get_width()
{
    return _xsize;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_search_mode(search_mode mode)
{
    this->_search_mode = mode;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_pathfinder(pathfinder_algo pathfinder)
{
    this->_pathfinder = pathfinder;
}

BRAIN_TEMPLATE
pathfinder_algo BRAIN_DEFINE::get_pathfinder() const
{
    return this->_pathfinder;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_trace(std::vector<ISite> *trace)
{
    this->_trace = trace;
}

BRAIN_TEMPLATE
search_mode BRAIN_DEFINE::get_search_mode() const
{
    return this->_search_mode;
}

BRAIN_TEMPLATE
std::uint32_t BRAIN_DEFINE::get_height()
{
    return _ysize;
}

BRAIN_TEMPLATE
void *BRAIN_DEFINE::data() const
{
    return static_cast<void *>(neurons);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::clear(bool clearLocks)
{
    std::uint32_t length = _xsize * _ysize * sizeof(INeuron);
    std::uint32_t leftOffset;
    if (clearLocks)
    {
        leftOffset = 0;
        length += this->_seg_off;
    }
    else
    {
        leftOffset = this->_seg_off;
    }
    memset(reinterpret_cast<void *>(reinterpret_cast<std::size_t>(neurons) + leftOffset), 0, length);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::fill(bool fillLocks)
{
    std::uint32_t length = _xsize * _ysize * sizeof(INeuron);
    std::uint32_t leftoffset;
    if (fillLocks)
    {
        leftoffset = 0;
        length += this->_seg_off;
    }
    else
    {
        leftoffset = this->_seg_off;
    }
    memset(reinterpret_cast<void *>(reinterpret_cast<std::size_t>(neurons) + leftoffset), 0xff, length);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::fill_locks(bool state)
{
    memset(reinterpret_cast<void *>(reinterpret_cast<std::size_t>(neurons)), 0xff * state, _seg_off);
}

BRAIN_TEMPLATE
INeuron *BRAIN_DEFINE::get(const ISite &range)
{
    INeuron *result = nullptr;
    if (contains(range))
        result = reinterpret_cast<INeuron *>(
            reinterpret_cast<std::size_t>(neurons) + _seg_off + (range.x * _ysize + range.y) * sizeof(INeuron));
    return result;
}

BRAIN_TEMPLATE
std::size_t BRAIN_DEFINE::get_cached_size()
{
    std::size_t cal = 0;

    INeuron *n = front();
    INeuron *ne = back() + 1;
    while (n < ne)
    {
        cal += n->f;
        ++n;
    }
    return cal;
}

BRAIN_TEMPLATE
INeuron *BRAIN_DEFINE::front()
{
    return reinterpret_cast<INeuron *>(reinterpret_cast<std::size_t>(neurons) + _seg_off);
}

BRAIN_TEMPLATE
INeuron *BRAIN_DEFINE::back()
{
    return front() + (size() - 1);
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::has_lock(const ISite &range)
{
    auto divide = std::lldiv(range.x * std::size_t(_ysize) + range.y, ByteSize);
    auto pointer = reinterpret_cast<std::uint8_t *>(neurons) + divide.quot;
    return (*pointer) & (1 << divide.rem);
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::has_lock(const INeuron *neuron)
{
    auto divide = std::lldiv(static_cast<std::size_t>(neuron - front()), ByteSize);
    auto pointer = reinterpret_cast<std::uint8_t *>(neurons) + divide.quot;
    return (*pointer) & (1 << divide.rem);
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::contains(const ISite &range)
{
    return (
        static_cast<std::uint32_t>(range.x) < _xsize && static_cast<std::uint32_t>(range.y) < _ysize && range.x > ~0
        && range.y > ~0);
}

BRAIN_TEMPLATE
const ISite BRAIN_DEFINE::get_point(const INeuron *neuron)
{
    auto divide = std::div((std::size_t(neuron) - std::size_t(neurons) - _seg_off) / sizeof(INeuron), (int) _ysize);
    return {divide.quot, divide.rem};
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_lock(const ISite &range, const bool state)
{
    auto divide = std::div(range.x * std::size_t(_ysize) + range.y, ByteSize);
    auto pointer = (reinterpret_cast<std::uint8_t *>(neurons)) + divide.quot;
    divide.quot = (1 << divide.rem);
    // FIXME: Optimize to ~divide.quout (xor)
    // Reset bit
    (*pointer) ^= (*pointer) & (divide.quot);
    // Write bit
    (*pointer) |= divide.quot * (state == true);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_lock(const INeuron *neuron, const bool state)
{
    auto divide = std::lldiv(static_cast<std::size_t>(neuron - front()), ByteSize);
    auto pointer = (reinterpret_cast<std::uint8_t *>(neurons)) + divide.quot;
    divide.quot = (1 << divide.rem);
    // FIXME: Optimize to ~divide.quout (xor)
    // Reset bit
    (*pointer) ^= (*pointer) & (divide.quot);
    // Write bit
    (*pointer) |= divide.quot * (state == true);
}

BRAIN_TEMPLATE
INeuron *BRAIN_DEFINE::get(int x, int y)
{
    return get({x, y});
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::contains(const INeuron *neuron)
{
    return contains(get_point(neuron));
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::load(const brain_breakfast &breakfast)
{
    if (!breakfast.widthSpace || !breakfast.heightSpace)
        throw std::runtime_error("Argument param, width or height is empty");

    if (breakfast.data == nullptr)
        throw std::runtime_error("pointer have null");

    if (this->neurons == breakfast.data)
        throw std::runtime_error("neurons == extrim, unflow effect");

    this->_xsize = breakfast.widthSpace;
    this->_ysize = breakfast.heightSpace;
    this->_seg_off = breakfast.len;

    _internal_realloc();

    std::memcpy(this->neurons, breakfast.data, breakfast.len);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::save(brain_breakfast *save_to)
{
    if (save_to == nullptr)
    {
        throw std::runtime_error("null pointer");
    }
    save_to->widthSpace = this->_xsize;
    save_to->heightSpace = this->_ysize;
    save_to->len = _seg_off;
    save_to->data = std::malloc(_seg_off);
    if (save_to->data == nullptr)
    {
        throw std::runtime_error("out of mem");
    }
    std::memcpy(save_to->data, this->neurons, _seg_off);
}

BRAIN_TEMPLATE
template <typename list_type, typename target_type>
list_type BRAIN_DEFINE::get_neighbours(matrix_identity matrixIdentity, const target_type &from)
{
    decltype(this->identity) identity;
    list_type list;
    ISite current_site, self_site;

    immune_system::get_matrix(matrixIdentity, identity);
    if constexpr (std::is_same_v<target_type, ISite>)
        current_site = from;
    else
        current_site = get_point(from);

    for (int s = 0; s < identity.length; ++s)
    {
        // Get the closest neurons
        self_site.x = current_site.x + identity.horizontal[s];
        self_site.y = current_site.y + identity.vertical[s];
        if (contains(self_site))
        {
            if constexpr (std::is_same_v<target_type, ISite>)
                list.push_back(self_site);
            else
                list.push_back(get(self_site));
        }
    }

    return list;
}

BRAIN_TEMPLATE
int BRAIN_DEFINE::random_number(int min, int max)
{
    if (max <= min)
        return min;
    return min + std::abs(_rand_()) % (max - min);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::find(navigate_result<list_type> &navigationResult, const ISite &first, const ISite &last)
{
    return this->find(this->_pathfinder, navigationResult, this->get(first), this->get(last), this->_search_mode);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::find(
    navigate_result<list_type> &navigationResult, const ISite &first, const ISite &last, search_mode mode)
{
    return this->find(this->_pathfinder, navigationResult, this->get(first), this->get(last), mode);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::find(
    pathfinder_algo pathfinder,
    navigate_result<list_type> &navigationResult,
    const ISite &first,
    const ISite &last,
    search_mode mode)
{
    return this->find(pathfinder, navigationResult, this->get(first), this->get(last), mode);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::find(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    return this->find(this->_pathfinder, navigationResult, firstNeuron, lastNeuron, this->_search_mode);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::find(
    navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron, search_mode mode)
{
    return this->find(this->_pathfinder, navigationResult, firstNeuron, lastNeuron, mode);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::find(
    pathfinder_algo pathfinder,
    navigate_result<list_type> &navigationResult,
    INeuron *firstNeuron,
    INeuron *lastNeuron,
    search_mode mode)
{
    // Every search starts from a clean state (locks are kept)
    this->clear(false);

    if (mode == search_mode::BiDirectional)
    {
        return this->_find_bidirectional(pathfinder, navigationResult, firstNeuron, lastNeuron);
    }
    return this->_find_unidirectional(pathfinder, navigationResult, firstNeuron, lastNeuron);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_unidirectional(
    pathfinder_algo pathfinder, navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    switch (pathfinder)
    {
        case pathfinder_algo::Dijkstra:
            return this->_find_dijkstra(navigationResult, firstNeuron, lastNeuron);
        case pathfinder_algo::BreadthFirstSearch:
            return this->_find_bfs(navigationResult, firstNeuron, lastNeuron);
        case pathfinder_algo::DepthFirstSearch:
            return this->_find_dfs(navigationResult, firstNeuron, lastNeuron);
        case pathfinder_algo::GreedyBestFirst:
            return this->_find_greedy(navigationResult, firstNeuron, lastNeuron);
        case pathfinder_algo::AStar:
        default:
            return this->_find_astar(navigationResult, firstNeuron, lastNeuron);
    }
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::_find_can_step(const ISite &from, int dir_index)
{
    // Prevent diagonal corner-cutting through adjacent locked tiles
    if (!_corner_cutting && identity.horizontal[dir_index] != 0 && identity.vertical[dir_index] != 0)
    {
        if (has_lock({from.x + identity.horizontal[dir_index], from.y})
            || has_lock({from.x, from.y + identity.vertical[dir_index]}))
        {
            return false;
        }
    }
    return true;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_begin(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    using target_type = typename list_type::value_type;

    if (firstNeuron == nullptr || lastNeuron == nullptr)
    {
        navigationResult.status = navigation_status::Undefined;
        return false;
    }
    if (has_lock(firstNeuron) || has_lock(lastNeuron))
    {
        navigationResult.status = navigation_status::Locked;
        return false;
    }

    navigationResult.status = navigation_status::Closed;

    BLOCK_SITE
    navigationResult.from = get_point(firstNeuron);
    navigationResult.to = get_point(lastNeuron);
    BLOCK_NEURON
    navigationResult.from = firstNeuron;
    navigationResult.to = lastNeuron;
    BLOCK_END

    return true;
}

BRAIN_TEMPLATE
template <typename list_type>
void BRAIN_DEFINE::_find_build_path(navigate_result<list_type> &navigationResult, INeuron *last)
{
    using target_type = typename list_type::value_type;

    navigationResult.status = navigation_status::Opened;

    int size = 0;
    for (INeuron *n = last; n; n = n->parent)
        ++size;

    navigationResult.connections.resize(size);
    for (INeuron *n = last; n; n = n->parent)
    {
        --size;
        BLOCK_SITE
        navigationResult.connections[size] = get_point(n);
        BLOCK_NEURON
        navigationResult.connections[size] = n;
        BLOCK_END
    }
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_astar(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    return _find_best_first(navigationResult, firstNeuron, lastNeuron, true);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_best_first(
    navigate_result<list_type> &navigationResult,
    INeuron *firstNeuron,
    INeuron *lastNeuron,
    bool use_heuristic,
    bool greedy)
{
    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    // Binary heap with lazy deletion: an improved neuron is pushed again and the stale entry is skipped on pop.
    struct open_entry
    {
        weight_t f;
        INeuron *neuron;
    };
    auto worse = [](const open_entry &lhs, const open_entry &rhs) { return lhs.f > rhs.f; };
    std::priority_queue<open_entry, std::vector<open_entry>, decltype(worse)> openList(worse);

    INeuron *current, *self;
    ISite first_site = get_point(firstNeuron), last_site = get_point(lastNeuron), self_site, current_site;

    firstNeuron->g = 0;
    firstNeuron->h = use_heuristic ? __heuristic__(first_site.x - last_site.x, first_site.y - last_site.y) : 0;
    firstNeuron->f = firstNeuron->h; // g == 0
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;

    openList.push({firstNeuron->f, firstNeuron});

    while (openList.empty() == false)
    {
        open_entry entry = openList.top();
        openList.pop();
        current = entry.neuron;
        // stale entry: already expanded, or a better one was pushed after it
        if (current->flags == NEURON_CAPTURE_CLOSED_LIST || entry.f != current->f)
            continue;

        if (current == lastNeuron)
        {
            _find_build_path(navigationResult, current);
            break;
        }
        current->flags = NEURON_CAPTURE_CLOSED_LIST;

        current_site = get_point(current);
        if (_trace)
            _trace->push_back(current_site);

        for (int s = 0; s < identity.length; ++s)
        {
            self_site.x = current_site.x + identity.horizontal[s];
            self_site.y = current_site.y + identity.vertical[s];
            self = get(self_site);
            if (self == nullptr || self->flags == NEURON_CAPTURE_CLOSED_LIST || has_lock(self_site))
                continue;
            if (!_find_can_step(current_site, s))
                continue;

            int newG = current->g + identity.g_weight[s];
            if (self->flags == NEURON_CAPTURE_OPEN_LIST)
            {
                if (newG >= self->g)
                    continue;
            }
            else
            {
                self->flags = NEURON_CAPTURE_OPEN_LIST;
                self->h = use_heuristic ? __heuristic__(self_site.x - last_site.x, self_site.y - last_site.y) : 0;
            }

            self->parent = current;
            self->g = newG;
            // Greedy sorts by the heuristic only
            self->f = greedy ? self->h : self->g + self->h;

            openList.push({self->f, self});
        }
    }

    return navigationResult.status == navigation_status::Opened;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_dijkstra(
    navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    // Dijkstra is the best-first search without a heuristic: f == g
    return _find_best_first(navigationResult, firstNeuron, lastNeuron, false);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_greedy(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    return _find_best_first(navigationResult, firstNeuron, lastNeuron, true, true);
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_bfs(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    std::queue<INeuron *> queue;
    INeuron *current, *self;
    ISite self_site, current_site;

    firstNeuron->g = firstNeuron->h = firstNeuron->f = 0;
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
    queue.push(firstNeuron);

    while (!queue.empty())
    {
        current = queue.front();
        queue.pop();
        current->flags = NEURON_CAPTURE_CLOSED_LIST;

        if (current == lastNeuron)
        {
            _find_build_path(navigationResult, current);
            break;
        }

        current_site = get_point(current);
        for (int s = 0; s < identity.length; ++s)
        {
            self_site.x = current_site.x + identity.horizontal[s];
            self_site.y = current_site.y + identity.vertical[s];
            self = get(self_site);
            // visited (open or closed) neurons are never revisited
            if (self == nullptr || self->flags == NEURON_CAPTURE_OPEN_LIST
                || self->flags == NEURON_CAPTURE_CLOSED_LIST || has_lock(self_site))
                continue;
            if (!_find_can_step(current_site, s))
                continue;

            self->flags = NEURON_CAPTURE_OPEN_LIST;
            self->parent = current;
            self->g = current->g + identity.g_weight[s];
            self->h = 0;
            self->f = self->g;
            queue.push(self);
        }
    }

    return navigationResult.status == navigation_status::Opened;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_bfs_bidirectional(
    navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    using target_type = typename list_type::value_type;

    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    if (firstNeuron == lastNeuron)
    {
        firstNeuron->parent = nullptr;
        _find_build_path(navigationResult, firstNeuron);
        return true;
    }

    // The backward side keeps its own g and parent, the forward side uses the neuron fields.
    struct bwd_t
    {
        weight_t g = 0;
        INeuron *parent = nullptr;
    };
    INeuron *base = this->front();
    std::vector<bwd_t> bwd(this->size());
    auto idx = [base](INeuron *n) { return static_cast<std::size_t>(n - base); };

    std::vector<INeuron *> fwd_layer {firstNeuron}, bwd_layer {lastNeuron}, next_layer;
    firstNeuron->g = 0;
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
    lastNeuron->flags = NEURON_CAPTURE_BWD_OPEN_LIST;

    INeuron *meeting = nullptr;
    weight_t best = std::numeric_limits<weight_t>::max() / 2;

    while (!fwd_layer.empty() && !bwd_layer.empty() && meeting == nullptr)
    {
        // expand the whole smaller layer, so the first meeting layer gives the fewest steps
        bool forward = fwd_layer.size() <= bwd_layer.size();
        std::vector<INeuron *> &layer = forward ? fwd_layer : bwd_layer;
        next_layer.clear();

        for (INeuron *current : layer)
        {
            ISite current_site = get_point(current);
            weight_t current_g = forward ? current->g : bwd[idx(current)].g;
            for (int s = 0; s < identity.length; ++s)
            {
                ISite self_site {current_site.x + identity.horizontal[s], current_site.y + identity.vertical[s]};
                INeuron *self = get(self_site);
                if (self == nullptr || has_lock(self_site) || !_find_can_step(current_site, s))
                    continue;

                const std::uint8_t own = forward ? NEURON_CAPTURE_OPEN_LIST : NEURON_CAPTURE_BWD_OPEN_LIST;
                const std::uint8_t other = forward ? NEURON_CAPTURE_BWD_OPEN_LIST : NEURON_CAPTURE_OPEN_LIST;
                weight_t newG = current_g + identity.g_weight[s];

                if (self->flags & other)
                {
                    // the frontiers touch: join the two sides and keep the cheapest join of this layer
                    weight_t other_g = forward ? bwd[idx(self)].g : self->g;
                    if (newG + other_g < best)
                    {
                        best = newG + other_g;
                        meeting = self;
                        if (forward)
                        {
                            // self belongs to the backward tree: remember how the forward side reaches it
                            self->g = newG;
                            self->parent = current;
                        }
                        else
                        {
                            bwd[idx(self)].g = newG;
                            bwd[idx(self)].parent = current;
                        }
                    }
                    continue;
                }
                if (self->flags & own)
                    continue;

                self->flags |= own;
                if (forward)
                {
                    self->g = newG;
                    self->parent = current;
                }
                else
                {
                    bwd[idx(self)].g = newG;
                    bwd[idx(self)].parent = current;
                }
                next_layer.push_back(self);
            }
        }
        layer.swap(next_layer);
    }

    if (meeting == nullptr)
        return false;

    navigationResult.status = navigation_status::Opened;
    std::vector<INeuron *> nodes;
    for (INeuron *n = meeting; n != nullptr; n = n->parent)
        nodes.push_back(n);
    std::reverse(nodes.begin(), nodes.end());
    for (INeuron *n = bwd[idx(meeting)].parent; n != nullptr; n = bwd[idx(n)].parent)
        nodes.push_back(n);

    navigationResult.connections.clear();
    for (INeuron *n : nodes)
    {
        BLOCK_SITE
        navigationResult.connections.push_back(get_point(n));
        BLOCK_NEURON
        navigationResult.connections.push_back(n);
        BLOCK_END
    }
    return true;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_dfs(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    // stack of (neuron, the neuron it was reached from)
    std::vector<std::pair<INeuron *, INeuron *>> stack;
    INeuron *current, *from, *self;
    ISite self_site, current_site;

    stack.emplace_back(firstNeuron, nullptr);

    while (!stack.empty())
    {
        std::tie(current, from) = stack.back();
        stack.pop_back();
        if (current->flags == NEURON_CAPTURE_CLOSED_LIST)
            continue;

        current->flags = NEURON_CAPTURE_CLOSED_LIST;
        current->parent = from;
        current->g = from ? from->g : 0;
        if (from)
        {
            ISite fs = get_point(from), cs = get_point(current);
            for (int s = 0; s < identity.length; ++s)
            {
                if (fs.x + identity.horizontal[s] == cs.x && fs.y + identity.vertical[s] == cs.y)
                {
                    current->g += identity.g_weight[s];
                    break;
                }
            }
        }
        current->h = 0;
        current->f = current->g;

        if (current == lastNeuron)
        {
            _find_build_path(navigationResult, current);
            break;
        }

        current_site = get_point(current);
        // reverse order so the first direction is explored first
        for (int s = identity.length - 1; s >= 0; --s)
        {
            self_site.x = current_site.x + identity.horizontal[s];
            self_site.y = current_site.y + identity.vertical[s];
            self = get(self_site);
            if (self == nullptr || self->flags == NEURON_CAPTURE_CLOSED_LIST || has_lock(self_site))
                continue;
            if (!_find_can_step(current_site, s))
                continue;
            self->flags = NEURON_CAPTURE_OPEN_LIST;
            stack.emplace_back(self, current);
        }
    }

    return navigationResult.status == navigation_status::Opened;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_bidirectional(
    pathfinder_algo pathfinder, navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    // Bidirectional search is meaningful for A*, Dijkstra and BFS; DFS and Greedy search in one direction
    if (pathfinder == pathfinder_algo::BreadthFirstSearch)
        return this->_find_bfs_bidirectional(navigationResult, firstNeuron, lastNeuron);
    if (pathfinder == pathfinder_algo::DepthFirstSearch || pathfinder == pathfinder_algo::GreedyBestFirst)
        return this->_find_unidirectional(pathfinder, navigationResult, firstNeuron, lastNeuron);

    // Dijkstra is A* with a zero heuristic
    heuristic_function saved_heuristic = __heuristic__;
    if (pathfinder == pathfinder_algo::Dijkstra)
        __heuristic__ = [](weight_t, weight_t) -> weight_t { return 0; };
    struct restore_t
    {
        heuristic_function &ref;
        heuristic_function value;
        ~restore_t() { ref = value; }
    } restore {__heuristic__, saved_heuristic};

    using navigate_result_t = navigate_result<list_type>;
    using target_type = typename list_type::value_type;

    if (firstNeuron == nullptr || lastNeuron == nullptr)
    {
        navigationResult.status = navigation_status::Undefined;
        return false;
    }
    if (has_lock(firstNeuron) || has_lock(lastNeuron))
    {
        navigationResult.status = navigation_status::Locked;
        return false;
    }

    ISite first_site = get_point(firstNeuron);
    ISite last_site = get_point(lastNeuron);

    navigationResult.status = navigation_status::Closed;

    BLOCK_SITE
    navigationResult.from = first_site;
    navigationResult.to = last_site;
    BLOCK_NEURON
    navigationResult.from = firstNeuron;
    navigationResult.to = lastNeuron;
    BLOCK_END

    if (firstNeuron == lastNeuron)
    {
        navigationResult.status = navigation_status::Opened;
        navigationResult.connections.clear();
        BLOCK_SITE
        navigationResult.connections.push_back(first_site);
        BLOCK_NEURON
        navigationResult.connections.push_back(firstNeuron);
        BLOCK_END
        return true;
    }

    struct bwd_neuron_info
    {
        weight_t g = std::numeric_limits<weight_t>::max() / 2;
        weight_t h = 0;
        weight_t f = 0;
        INeuron *parent = nullptr;
    };

    INeuron *base_neuron = this->front();
    std::size_t total = this->size();
    std::vector<bwd_neuron_info> bwd_info(total);

    auto compare_fwd = [](const INeuron *lhs, const INeuron *rhs) {
        if (lhs->f != rhs->f)
            return lhs->f < rhs->f;
        return lhs < rhs;
    };
    std::set<INeuron *, decltype(compare_fwd)> openList_fwd(compare_fwd);

    auto compare_bwd = [&bwd_info, base_neuron](const INeuron *lhs, const INeuron *rhs) {
        std::size_t li = static_cast<std::size_t>(lhs - base_neuron);
        std::size_t ri = static_cast<std::size_t>(rhs - base_neuron);
        if (bwd_info[li].f != bwd_info[ri].f)
            return bwd_info[li].f < bwd_info[ri].f;
        return lhs < rhs;
    };
    std::set<INeuron *, decltype(compare_bwd)> openList_bwd(compare_bwd);

    // Initialize forward
    firstNeuron->g = 0;
    firstNeuron->h = __heuristic__(first_site.x - last_site.x, first_site.y - last_site.y);
    firstNeuron->f = firstNeuron->h;
    firstNeuron->parent = nullptr;
    firstNeuron->flags |= NEURON_CAPTURE_OPEN_LIST;
    openList_fwd.insert(firstNeuron);

    // Initialize backward
    std::size_t last_idx = static_cast<std::size_t>(lastNeuron - base_neuron);
    bwd_info[last_idx].g = 0;
    bwd_info[last_idx].h = __heuristic__(last_site.x - first_site.x, last_site.y - first_site.y);
    bwd_info[last_idx].f = bwd_info[last_idx].h;
    bwd_info[last_idx].parent = nullptr;
    lastNeuron->flags |= NEURON_CAPTURE_BWD_OPEN_LIST;
    openList_bwd.insert(lastNeuron);

    weight_t best_path_cost = std::numeric_limits<weight_t>::max() / 2;
    INeuron *meeting_neuron = nullptr;

    while (!openList_fwd.empty() && !openList_bwd.empty())
    {
        INeuron *top_fwd = *openList_fwd.begin();
        std::size_t top_fwd_idx = static_cast<std::size_t>(top_fwd - base_neuron);
        INeuron *top_bwd = *openList_bwd.begin();
        std::size_t top_bwd_idx = static_cast<std::size_t>(top_bwd - base_neuron);

        if (meeting_neuron != nullptr)
        {
            // Pohl's criterion: the path can't improve once the smallest f of either frontier reaches the best cost.
            // (Do not add a "gF + gB >= best" test here: the top of an A* frontier is not the smallest-g node,
            // so it stops too early and returns non-optimal paths.)
            if (std::max(top_fwd->f, bwd_info[top_bwd_idx].f) >= best_path_cost)
                break;
        }

        bool expand_fwd;
        if (top_fwd->f != bwd_info[top_bwd_idx].f)
            expand_fwd = (top_fwd->f < bwd_info[top_bwd_idx].f);
        else
            expand_fwd = (openList_fwd.size() <= openList_bwd.size());

        if (expand_fwd)
        {
            INeuron *current = top_fwd;
            openList_fwd.erase(openList_fwd.begin());
            current->flags &= ~NEURON_CAPTURE_OPEN_LIST;
            current->flags |= NEURON_CAPTURE_CLOSED_LIST;

            std::size_t cur_idx = static_cast<std::size_t>(current - base_neuron);
            ISite current_site = get_point(current);

            if (current->flags & (NEURON_CAPTURE_BWD_OPEN_LIST | NEURON_CAPTURE_BWD_CLOSED_LIST))
            {
                weight_t total_g = current->g + bwd_info[cur_idx].g;
                if (total_g < best_path_cost)
                {
                    best_path_cost = total_g;
                    meeting_neuron = current;
                }
            }

            for (int s = 0; s < identity.length; ++s)
            {
                ISite self_site {current_site.x + identity.horizontal[s], current_site.y + identity.vertical[s]};
                INeuron *self = get(self_site);
                if (self == nullptr || (self->flags & NEURON_CAPTURE_CLOSED_LIST) || has_lock(self_site))
                    continue;

                if (!_corner_cutting && identity.horizontal[s] != 0 && identity.vertical[s] != 0)
                {
                    if (has_lock({current_site.x + identity.horizontal[s], current_site.y})
                        || has_lock({current_site.x, current_site.y + identity.vertical[s]}))
                    {
                        continue;
                    }
                }

                int newG = current->g + identity.g_weight[s];
                std::size_t self_idx = static_cast<std::size_t>(self - base_neuron);

                if (self->flags & NEURON_CAPTURE_OPEN_LIST)
                {
                    if (newG < self->g)
                    {
                        openList_fwd.erase(self);
                        self->g = newG;
                        self->f = self->g + self->h;
                        self->parent = current;
                        openList_fwd.insert(self);
                    }
                }
                else
                {
                    self->flags |= NEURON_CAPTURE_OPEN_LIST;
                    self->parent = current;
                    self->g = newG;
                    self->h = __heuristic__(self_site.x - last_site.x, self_site.y - last_site.y);
                    self->f = self->g + self->h;
                    openList_fwd.insert(self);
                }

                if (self->flags & (NEURON_CAPTURE_BWD_OPEN_LIST | NEURON_CAPTURE_BWD_CLOSED_LIST))
                {
                    weight_t total_g = self->g + bwd_info[self_idx].g;
                    if (total_g < best_path_cost)
                    {
                        best_path_cost = total_g;
                        meeting_neuron = self;
                    }
                }
            }
        }
        else
        {
            INeuron *current = top_bwd;
            openList_bwd.erase(openList_bwd.begin());
            current->flags &= ~NEURON_CAPTURE_BWD_OPEN_LIST;
            current->flags |= NEURON_CAPTURE_BWD_CLOSED_LIST;

            std::size_t cur_idx = static_cast<std::size_t>(current - base_neuron);
            ISite current_site = get_point(current);

            if (current->flags & (NEURON_CAPTURE_OPEN_LIST | NEURON_CAPTURE_CLOSED_LIST))
            {
                weight_t total_g = current->g + bwd_info[cur_idx].g;
                if (total_g < best_path_cost)
                {
                    best_path_cost = total_g;
                    meeting_neuron = current;
                }
            }

            for (int s = 0; s < identity.length; ++s)
            {
                ISite self_site {current_site.x + identity.horizontal[s], current_site.y + identity.vertical[s]};
                INeuron *self = get(self_site);
                if (self == nullptr || (self->flags & NEURON_CAPTURE_BWD_CLOSED_LIST) || has_lock(self_site))
                    continue;

                if (!_corner_cutting && identity.horizontal[s] != 0 && identity.vertical[s] != 0)
                {
                    if (has_lock({current_site.x + identity.horizontal[s], current_site.y})
                        || has_lock({current_site.x, current_site.y + identity.vertical[s]}))
                    {
                        continue;
                    }
                }

                std::size_t self_idx = static_cast<std::size_t>(self - base_neuron);
                int newG = bwd_info[cur_idx].g + identity.g_weight[s];

                if (self->flags & NEURON_CAPTURE_BWD_OPEN_LIST)
                {
                    if (newG < bwd_info[self_idx].g)
                    {
                        openList_bwd.erase(self);
                        bwd_info[self_idx].g = newG;
                        bwd_info[self_idx].f = bwd_info[self_idx].g + bwd_info[self_idx].h;
                        bwd_info[self_idx].parent = current;
                        openList_bwd.insert(self);
                    }
                }
                else
                {
                    self->flags |= NEURON_CAPTURE_BWD_OPEN_LIST;
                    bwd_info[self_idx].parent = current;
                    bwd_info[self_idx].g = newG;
                    bwd_info[self_idx].h = __heuristic__(self_site.x - first_site.x, self_site.y - first_site.y);
                    bwd_info[self_idx].f = bwd_info[self_idx].g + bwd_info[self_idx].h;
                    openList_bwd.insert(self);
                }

                if (self->flags & (NEURON_CAPTURE_OPEN_LIST | NEURON_CAPTURE_CLOSED_LIST))
                {
                    weight_t total_g = self->g + bwd_info[self_idx].g;
                    if (total_g < best_path_cost)
                    {
                        best_path_cost = total_g;
                        meeting_neuron = self;
                    }
                }
            }
        }
    }

    if (meeting_neuron != nullptr)
    {
        navigationResult.status = navigation_status::Opened;

        std::vector<INeuron *> fwd_nodes;
        INeuron *curr = meeting_neuron;
        while (curr != nullptr)
        {
            fwd_nodes.push_back(curr);
            curr = curr->parent;
        }
        std::reverse(fwd_nodes.begin(), fwd_nodes.end());

        std::vector<INeuron *> bwd_nodes;
        std::size_t meet_idx = static_cast<std::size_t>(meeting_neuron - base_neuron);
        curr = bwd_info[meet_idx].parent;
        while (curr != nullptr)
        {
            bwd_nodes.push_back(curr);
            std::size_t idx = static_cast<std::size_t>(curr - base_neuron);
            curr = bwd_info[idx].parent;
        }

        navigationResult.connections.clear();
        navigationResult.connections.reserve(fwd_nodes.size() + bwd_nodes.size());

        for (INeuron *n : fwd_nodes)
        {
            BLOCK_SITE
            navigationResult.connections.push_back(get_point(n));
            BLOCK_NEURON
            navigationResult.connections.push_back(n);
            BLOCK_END
        }
        for (INeuron *n : bwd_nodes)
        {
            BLOCK_SITE
            navigationResult.connections.push_back(get_point(n));
            BLOCK_NEURON
            navigationResult.connections.push_back(n);
            BLOCK_END
        }
        return true;
    }

    return false;
}

inline bool operator!=(const brain::the_site &a, const brain::the_site &b)
{
    return static_cast<bool>(std::memcmp(&a, &b, sizeof(brain::the_site)));
}

inline bool operator==(const brain::the_site &a, const brain::the_site &b)
{
    return static_cast<bool>(!std::memcmp(&a, &b, sizeof(brain::the_site)));
}

inline brain::the_site &operator+=(brain::the_site &lhs, const brain::the_site &rhs)
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    return lhs;
}

#undef BLOCK_SITE
#undef BLOCK_NEURON
#undef BLOCK_END

#endif
