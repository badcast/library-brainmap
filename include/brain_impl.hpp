// declarations Brain Map (algorithm) part-file
#ifndef _BRAIN_IMPL_HPP_
#define _BRAIN_IMPL_HPP_

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

// entry of the binary heap used by the best-first searches (lazy deletion: stale entries are skipped on pop)
template <typename INeuron>
struct open_entry
{
    brain::weight_t f;
    INeuron *neuron;
};

template <typename INeuron>
struct open_entry_worse
{
    bool operator()(const open_entry<INeuron> &lhs, const open_entry<INeuron> &rhs) const
    {
        return lhs.f > rhs.f;
    }
};

template <typename INeuron>
using open_queue
    = std::priority_queue<open_entry<INeuron>, std::vector<open_entry<INeuron>>, open_entry_worse<INeuron>>;

// detects the stamp field of a neuron type (generation-based reset of the search state)
template <typename T, typename = void>
struct has_stamp : std::false_type
{};

template <typename T>
struct has_stamp<T, std::void_t<decltype(std::declval<T &>().stamp)>> : std::true_type
{};

// cost of a cell that can not be reached
constexpr brain::weight_t unreachable_cost = std::numeric_limits<brain::weight_t>::max() / 2;

// entry of the min-heap of the plain Dijkstra searches over an index space (flow field, HPA*)
struct cost_entry
{
    brain::weight_t cost;
    int id;
};

struct cost_entry_worse
{
    bool operator()(const cost_entry &lhs, const cost_entry &rhs) const
    {
        // equal costs pop by cell id: the order (and so every result) is the same with any STL heap implementation
        return lhs.cost > rhs.cost || (lhs.cost == rhs.cost && lhs.id > rhs.id);
    }
};

using cost_queue = std::priority_queue<cost_entry, std::vector<cost_entry>, cost_entry_worse>;

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
    static brain::weight_t M_G_DIAGONAL_WEIGHT[] {2, 1, 2, 1, 1, 2, 1, 2};
    /*
     PLUS

       *
     * x *
       *

    */
    static std::int8_t M_PLUS_H_POINT[] {-1, 1, 0, 0};
    static std::int8_t M_PLUS_V_POINT[] {0, 0, -1, 1};
    static brain::weight_t M_G_PLUS_WEIGHT[] {1, 1, 1, 1};
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
    const std::size_t cells = static_cast<std::size_t>(_xsize) * _ysize;

    // never leave a dangling pointer behind if the allocation below throws
    std::free(this->neurons);
    this->neurons = nullptr;

    // the segment of the lock bits comes first (the size formula is kept for the saved breakfast data)
    const std::lldiv_t lockedDiv = std::lldiv(std::max<std::size_t>(cells, ByteSize), ByteSize);
    this->_lock_bytes = static_cast<std::uint32_t>(lockedDiv.quot + lockedDiv.rem);
    // the neurons follow the locks: keep them aligned
    constexpr std::size_t alignment = alignof(INeuron);
    this->_seg_off = static_cast<std::uint32_t>((this->_lock_bytes + alignment - 1) / alignment * alignment);
    this->neurons = static_cast<INeuron *>(std::malloc(cells * sizeof(INeuron) + this->_seg_off));

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
BRAIN_DEFINE::basic_brain_map(basic_brain_map &&other) noexcept
    : neurons(other.neurons)
    , _rand_(std::move(other._rand_))
    , _heuristic_fn(other._heuristic_fn)
    , _seg_off(other._seg_off)
    , _lock_bytes(other._lock_bytes)
    , _xsize(other._xsize)
    , _ysize(other._ysize)
    , identity(other.identity)
    , _search_mode(other._search_mode)
    , _pathfinder(other._pathfinder)
    , _trace(other._trace)
    , _custom_weights(other._custom_weights)
    , _custom_orthogonal(other._custom_orthogonal)
    , _custom_diagonal(other._custom_diagonal)
    , _corner_cutting(other._corner_cutting)
    , _search_id(other._search_id)
{
    // identity.g_weight may point into the custom weights of the other map
    _apply_custom_weights();
    other.neurons = nullptr;
    other._xsize = other._ysize = other._seg_off = other._lock_bytes = 0;
}

BRAIN_TEMPLATE
BRAIN_DEFINE &BRAIN_DEFINE::operator=(basic_brain_map &&other) noexcept
{
    if (this != &other)
    {
        std::free(neurons);
        neurons = other.neurons;
        _rand_ = std::move(other._rand_);
        _heuristic_fn = other._heuristic_fn;
        _seg_off = other._seg_off;
        _lock_bytes = other._lock_bytes;
        _xsize = other._xsize;
        _ysize = other._ysize;
        identity = other.identity;
        _search_mode = other._search_mode;
        _pathfinder = other._pathfinder;
        _trace = other._trace;
        _custom_weights = other._custom_weights;
        _custom_orthogonal = other._custom_orthogonal;
        _custom_diagonal = other._custom_diagonal;
        _corner_cutting = other._corner_cutting;
        _search_id = other._search_id;
        _bwd_cache.clear();
        _apply_custom_weights();
        other.neurons = nullptr;
        other._xsize = other._ysize = other._seg_off = other._lock_bytes = 0;
    }
    return *this;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::randomize_hardware(int flagFilter)
{
    std::uint32_t lhs, rhs = _lock_bytes;
    clear(true);
    do
    {
        lhs = static_cast<std::uint32_t>(_rand_() & flagFilter);
        memcpy(
            reinterpret_cast<void *>(reinterpret_cast<std::size_t>(neurons) + _lock_bytes - rhs),
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
        _custom_g_weight[s] = diagonal ? _custom_diagonal : _custom_orthogonal;
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
    this->_heuristic_fn = heuristic;
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
            _heuristic_fn = immune_system::heuristic_pythagorean;
            break;
        case heuristic_method::Hypot:
            _heuristic_fn = immune_system::heuristic_hypot;
            break;
        case heuristic_method::Manhattan:
            _heuristic_fn = immune_system::heuristic_manhtattan;
            break;
        case heuristic_method::Chebyshev:
            _heuristic_fn = immune_system::heuristic_chebyshev;
            break;
        case heuristic_method::Octile:
            _heuristic_fn = immune_system::heuristic_octile;
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
    if (_heuristic_fn == immune_system::heuristic_pythagorean)
        return heuristic_method::Pythagorean;
    else if (_heuristic_fn == immune_system::heuristic_hypot)
        return heuristic_method::Hypot;
    else if (_heuristic_fn == immune_system::heuristic_manhtattan)
        return heuristic_method::Manhattan;
    else if (_heuristic_fn == immune_system::heuristic_chebyshev)
        return heuristic_method::Chebyshev;
    else if (_heuristic_fn == immune_system::heuristic_octile)
        return heuristic_method::Octile;
    else if (_heuristic_fn != nullptr)
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
    std::size_t length = static_cast<std::size_t>(_xsize) * _ysize * sizeof(INeuron);
    std::size_t leftOffset;
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
    std::size_t length = static_cast<std::size_t>(_xsize) * _ysize * sizeof(INeuron);
    std::size_t leftoffset;
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
    memset(reinterpret_cast<void *>(reinterpret_cast<std::size_t>(neurons)), 0xff * state, _lock_bytes);
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
bool BRAIN_DEFINE::_lock_bit(std::size_t index) const
{
    return (reinterpret_cast<const std::uint8_t *>(neurons)[index >> 3] >> (index & 7)) & 1;
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::_set_lock_bit(std::size_t index, bool state)
{
    std::uint8_t &byte = reinterpret_cast<std::uint8_t *>(neurons)[index >> 3];
    const std::uint8_t mask = static_cast<std::uint8_t>(1u << (index & 7));
    if (state)
        byte |= mask;
    else
        byte &= static_cast<std::uint8_t>(~mask);
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::has_lock(const ISite &range)
{
    return _lock_bit(static_cast<std::size_t>(range.x) * _ysize + range.y);
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::has_lock(const INeuron *neuron)
{
    return _lock_bit(static_cast<std::size_t>(neuron - front()));
}

BRAIN_TEMPLATE
bool BRAIN_DEFINE::contains(const ISite &range)
{
    // negative coordinates wrap to huge unsigned values and fail the comparison
    return static_cast<std::uint32_t>(range.x) < _xsize && static_cast<std::uint32_t>(range.y) < _ysize;
}

BRAIN_TEMPLATE
const ISite BRAIN_DEFINE::get_point(const INeuron *neuron)
{
    const std::size_t index = static_cast<std::size_t>(neuron - front());
    const std::size_t x = index / _ysize;
    return {static_cast<int>(x), static_cast<int>(index - x * _ysize)};
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_lock(const ISite &range, const bool state)
{
    _set_lock_bit(static_cast<std::size_t>(range.x) * _ysize + range.y, state);
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::set_lock(const INeuron *neuron, const bool state)
{
    _set_lock_bit(static_cast<std::size_t>(neuron - front()), state);
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
    _internal_realloc();

    // the saved lock bytes must fit the size of the map
    if (breakfast.len < 0 || static_cast<std::uint32_t>(breakfast.len) != this->_lock_bytes)
        throw std::runtime_error("the size of the lock data does not match the size of the map");

    std::memcpy(this->neurons, breakfast.data, this->_lock_bytes);
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
    save_to->len = _lock_bytes;
    save_to->data = std::malloc(_lock_bytes);
    if (save_to->data == nullptr)
    {
        throw std::runtime_error("out of mem");
    }
    std::memcpy(save_to->data, this->neurons, _lock_bytes);
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
    // Every search starts from a clean state (locks are kept): a new search id makes the old neuron states stale
    this->_begin_search();
    if (firstNeuron != nullptr)
        this->_touch(firstNeuron);
    if (lastNeuron != nullptr)
        this->_touch(lastNeuron);

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
        case pathfinder_algo::JumpPointSearch:
            return this->_find_jps(navigationResult, firstNeuron, lastNeuron);
        case pathfinder_algo::ThetaStar:
            return this->_find_theta(navigationResult, firstNeuron, lastNeuron);
        case pathfinder_algo::IDAStar:
            return this->_find_ida(navigationResult, firstNeuron, lastNeuron);
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
void BRAIN_DEFINE::_begin_search()
{
    if constexpr (immune_system::has_stamp<INeuron>::value)
    {
        // The neurons keep the low 16 bits of the search id. They are all reset when the counter wraps around.
        ++_search_id;
        if ((_search_id & 0xffffu) == 0)
        {
            clear(false);
            ++_search_id;
        }
        if (_bwd_cache.size() != size())
            _bwd_cache.assign(size(), bwd_slot {});
    }
    else
    {
        // a neuron type without the stamp field: reset the whole map
        clear(false);
        ++_search_id;
        if (_bwd_cache.size() != size())
            _bwd_cache.assign(size(), bwd_slot {});
    }
}

BRAIN_TEMPLATE
void BRAIN_DEFINE::_touch(INeuron *neuron)
{
    if constexpr (immune_system::has_stamp<INeuron>::value)
    {
        const std::uint16_t current = static_cast<std::uint16_t>(_search_id & 0xffffu);
        if (neuron->stamp != current)
        {
            neuron->flags = 0;
            neuron->g = neuron->h = neuron->f = 0;
            neuron->parent = nullptr;
            neuron->stamp = current;
        }
    }
}

BRAIN_TEMPLATE
INeuron *BRAIN_DEFINE::_node(const ISite &site)
{
    INeuron *neuron = get(site);
    if (neuron != nullptr)
        _touch(neuron);
    return neuron;
}

BRAIN_TEMPLATE
typename BRAIN_DEFINE::bwd_slot &BRAIN_DEFINE::_bwd_at(std::size_t index)
{
    bwd_slot &slot = _bwd_cache[index];
    if (slot.stamp != _search_id)
        slot = bwd_slot {0, 0, 0, nullptr, _search_id};
    return slot;
}

BRAIN_TEMPLATE
template <typename list_type>
void BRAIN_DEFINE::_find_emit_nodes(navigate_result<list_type> &navigationResult, const std::vector<INeuron *> &nodes)
{
    using target_type = typename list_type::value_type;

    navigationResult.status = navigation_status::Opened;
    navigationResult.connections.clear();
    navigationResult.connections.reserve(nodes.size());
    for (INeuron *n : nodes)
    {
        BLOCK_SITE
        navigationResult.connections.push_back(get_point(n));
        BLOCK_NEURON
        navigationResult.connections.push_back(n);
        BLOCK_END
    }
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_trivial(
    navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    if (firstNeuron != lastNeuron)
        return false;
    firstNeuron->parent = nullptr;
    _find_emit_nodes(navigationResult, std::vector<INeuron *> {firstNeuron});
    return true;
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

    // Binary heap with lazy deletion: an improved neuron is pushed again and the stale entry is skipped on pop
    immune_system::open_queue<INeuron> openList;

    INeuron *current, *self;
    ISite first_site = get_point(firstNeuron), last_site = get_point(lastNeuron), self_site, current_site;

    firstNeuron->g = 0;
    firstNeuron->h = use_heuristic ? _heuristic_fn(first_site.x - last_site.x, first_site.y - last_site.y) : 0;
    firstNeuron->f = firstNeuron->h; // g == 0
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;

    openList.push({firstNeuron->f, firstNeuron});

    while (openList.empty() == false)
    {
        immune_system::open_entry<INeuron> entry = openList.top();
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
            self = _node(self_site);
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
                self->h = use_heuristic ? _heuristic_fn(self_site.x - last_site.x, self_site.y - last_site.y) : 0;
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
            self = _node(self_site);
            // visited (open or closed) neurons are never revisited
            if (self == nullptr || self->flags == NEURON_CAPTURE_OPEN_LIST || self->flags == NEURON_CAPTURE_CLOSED_LIST
                || has_lock(self_site))
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
    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    if (_find_trivial(navigationResult, firstNeuron, lastNeuron))
        return true;

    // The backward side keeps its own g and parent, the forward side uses the neuron fields.
    // the backward side lives in the reusable cache of the map
    struct bwd_view
    {
        basic_brain_map *map;
        bwd_slot &operator[](std::size_t index)
        {
            return map->_bwd_at(index);
        }
    } bwd {this};
    INeuron *base = this->front();
    auto idx = [base](INeuron *n) {
        return static_cast<std::size_t>(n - base);
    };

    std::vector<INeuron *> fwd_layer {firstNeuron}, bwd_layer {lastNeuron}, next_layer;
    firstNeuron->g = 0;
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
    lastNeuron->flags = NEURON_CAPTURE_BWD_OPEN_LIST;
    bwd[idx(lastNeuron)].g = 0;

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
                INeuron *self = _node(self_site);
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

    std::vector<INeuron *> nodes;
    for (INeuron *n = meeting; n != nullptr; n = n->parent)
        nodes.push_back(n);
    std::reverse(nodes.begin(), nodes.end());
    for (INeuron *n = bwd[idx(meeting)].parent; n != nullptr; n = bwd[idx(n)].parent)
        nodes.push_back(n);

    _find_emit_nodes(navigationResult, nodes);
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
        if (_trace)
            _trace->push_back(current_site);
        // reverse order so the first direction is explored first
        for (int s = identity.length - 1; s >= 0; --s)
        {
            self_site.x = current_site.x + identity.horizontal[s];
            self_site.y = current_site.y + identity.vertical[s];
            self = _node(self_site);
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
bool BRAIN_DEFINE::_find_jps(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    // JPS requires an 8-direction grid with symmetric orthogonal and diagonal step costs (w_diag <= 2 * w_orth)
    if (identity.length != 8 || get_identity() != matrix_identity::DiagonalMethod)
        return _find_astar(navigationResult, firstNeuron, lastNeuron);

    const weight_t w_orth = identity.g_weight[1];
    const weight_t w_diag = identity.g_weight[0];
    if (w_orth <= 0 || w_diag <= 0 || w_diag > 2 * w_orth)
        return _find_astar(navigationResult, firstNeuron, lastNeuron);

    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    const int w = static_cast<int>(_xsize);
    const int h = static_cast<int>(_ysize);
    const ISite first_site = get_point(firstNeuron);
    const ISite last_site = get_point(lastNeuron);

    if (_find_trivial(navigationResult, firstNeuron, lastNeuron))
        return true;

    auto passable = [&](int x, int y) -> bool {
        return x >= 0 && y >= 0 && x < w && y < h && !has_lock(ISite {x, y});
    };

    auto can_step_dir = [&](int x, int y, int dx, int dy) -> bool {
        if (!passable(x + dx, y + dy))
            return false;
        if (!_corner_cutting && dx != 0 && dy != 0)
            return passable(x + dx, y) && passable(x, y + dy);
        return true;
    };

    auto jump_orth = [&](int x, int y, int dx, int dy) -> ISite {
        for (;;)
        {
            if (!can_step_dir(x, y, dx, dy))
                return ISite {-1, -1};
            x += dx;
            y += dy;
            if (x == last_site.x && y == last_site.y)
                return ISite {x, y};
            if (dx != 0)
            {
                if (_corner_cutting)
                {
                    if ((!passable(x, y + 1) && passable(x + dx, y + 1))
                        || (!passable(x, y - 1) && passable(x + dx, y - 1)))
                        return ISite {x, y};
                }
                else
                {
                    if ((!passable(x - dx, y + 1) && passable(x, y + 1))
                        || (!passable(x - dx, y - 1) && passable(x, y - 1)))
                        return ISite {x, y};
                }
            }
            else
            {
                if (_corner_cutting)
                {
                    if ((!passable(x + 1, y) && passable(x + 1, y + dy))
                        || (!passable(x - 1, y) && passable(x - 1, y + dy)))
                        return ISite {x, y};
                }
                else
                {
                    if ((!passable(x + 1, y - dy) && passable(x + 1, y))
                        || (!passable(x - 1, y - dy) && passable(x - 1, y)))
                        return ISite {x, y};
                }
            }
        }
    };

    auto jump = [&](int x, int y, int dx, int dy) -> ISite {
        if (dx == 0 || dy == 0)
            return jump_orth(x, y, dx, dy);
        for (;;)
        {
            if (!can_step_dir(x, y, dx, dy))
                return ISite {-1, -1};
            x += dx;
            y += dy;
            if (x == last_site.x && y == last_site.y)
                return ISite {x, y};
            if (_corner_cutting)
            {
                if ((!passable(x - dx, y) && can_step_dir(x, y, -dx, dy))
                    || (!passable(x, y - dy) && can_step_dir(x, y, dx, -dy)))
                    return ISite {x, y};
            }
            if (jump_orth(x, y, dx, 0).x >= 0 || jump_orth(x, y, 0, dy).x >= 0)
                return ISite {x, y};
        }
    };

    immune_system::open_queue<INeuron> openList;

    firstNeuron->g = 0;
    firstNeuron->h = _heuristic_fn(first_site.x - last_site.x, first_site.y - last_site.y);
    firstNeuron->f = firstNeuron->h;
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
    openList.push({firstNeuron->f, firstNeuron});

    std::pair<int, int> dirs[8];
    int dir_count = 0;
    auto add_dir = [&](int dx, int dy) {
        for (int i = 0; i < dir_count; ++i)
            if (dirs[i].first == dx && dirs[i].second == dy)
                return;
        dirs[dir_count++] = {dx, dy};
    };

    while (!openList.empty())
    {
        immune_system::open_entry<INeuron> entry = openList.top();
        openList.pop();
        INeuron *current = entry.neuron;
        if (current->flags == NEURON_CAPTURE_CLOSED_LIST || entry.f != current->f)
            continue;

        current->flags = NEURON_CAPTURE_CLOSED_LIST;
        ISite cur_site = get_point(current);
        if (_trace)
            _trace->push_back(cur_site);

        if (current == lastNeuron)
        {
            std::vector<INeuron *> jps_nodes;
            for (INeuron *n = current; n != nullptr; n = n->parent)
                jps_nodes.push_back(n);
            std::reverse(jps_nodes.begin(), jps_nodes.end());

            // the jump points are far apart: add the cells between them
            std::vector<INeuron *> full_path {jps_nodes.front()};
            for (std::size_t i = 1; i < jps_nodes.size(); ++i)
            {
                ISite p = get_point(jps_nodes[i - 1]);
                const ISite target = get_point(jps_nodes[i]);
                const int sx = (target.x > p.x) - (target.x < p.x);
                const int sy = (target.y > p.y) - (target.y < p.y);
                while (p.x != target.x || p.y != target.y)
                {
                    p.x += sx;
                    p.y += sy;
                    full_path.push_back(_node(p));
                }
            }
            _find_emit_nodes(navigationResult, full_path);
            break;
        }

        dir_count = 0;
        if (current->parent == nullptr)
        {
            for (int s = 0; s < 8; ++s)
                add_dir(identity.horizontal[s], identity.vertical[s]);
        }
        else
        {
            ISite ps = get_point(current->parent);
            int dx = (cur_site.x > ps.x) - (cur_site.x < ps.x);
            int dy = (cur_site.y > ps.y) - (cur_site.y < ps.y);
            if (dx != 0 && dy != 0)
            {
                add_dir(dx, 0);
                add_dir(0, dy);
                add_dir(dx, dy);
                if (_corner_cutting)
                {
                    if (!passable(cur_site.x - dx, cur_site.y))
                        add_dir(-dx, dy);
                    if (!passable(cur_site.x, cur_site.y - dy))
                        add_dir(dx, -dy);
                }
            }
            else if (dx != 0)
            {
                add_dir(dx, 0);
                if (_corner_cutting)
                {
                    if (!passable(cur_site.x, cur_site.y + 1))
                        add_dir(dx, 1);
                    if (!passable(cur_site.x, cur_site.y - 1))
                        add_dir(dx, -1);
                }
                else
                {
                    if (!passable(cur_site.x - dx, cur_site.y + 1) && passable(cur_site.x, cur_site.y + 1))
                    {
                        add_dir(0, 1);
                        add_dir(dx, 1);
                    }
                    if (!passable(cur_site.x - dx, cur_site.y - 1) && passable(cur_site.x, cur_site.y - 1))
                    {
                        add_dir(0, -1);
                        add_dir(dx, -1);
                    }
                }
            }
            else
            {
                add_dir(0, dy);
                if (_corner_cutting)
                {
                    if (!passable(cur_site.x + 1, cur_site.y))
                        add_dir(1, dy);
                    if (!passable(cur_site.x - 1, cur_site.y))
                        add_dir(-1, dy);
                }
                else
                {
                    if (!passable(cur_site.x + 1, cur_site.y - dy) && passable(cur_site.x + 1, cur_site.y))
                    {
                        add_dir(1, 0);
                        add_dir(1, dy);
                    }
                    if (!passable(cur_site.x - 1, cur_site.y - dy) && passable(cur_site.x - 1, cur_site.y))
                    {
                        add_dir(-1, 0);
                        add_dir(-1, dy);
                    }
                }
            }
        }

        for (int i = 0; i < dir_count; ++i)
        {
            int dx = dirs[i].first, dy = dirs[i].second;
            ISite jp = jump(cur_site.x, cur_site.y, dx, dy);
            if (jp.x < 0)
                continue;

            INeuron *self = _node(jp);
            if (self == nullptr || self->flags == NEURON_CAPTURE_CLOSED_LIST)
                continue;

            int steps = std::max(std::abs(jp.x - cur_site.x), std::abs(jp.y - cur_site.y));
            weight_t step_cost = (dx != 0 && dy != 0) ? w_diag : w_orth;
            weight_t newG = current->g + steps * step_cost;

            if (self->flags == NEURON_CAPTURE_OPEN_LIST)
            {
                if (newG >= self->g)
                    continue;
            }
            else
            {
                self->flags = NEURON_CAPTURE_OPEN_LIST;
                self->h = _heuristic_fn(jp.x - last_site.x, jp.y - last_site.y);
            }

            self->parent = current;
            self->g = newG;
            self->f = self->g + self->h;
            openList.push({self->f, self});
        }
    }

    return navigationResult.status == navigation_status::Opened;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_theta(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    const ISite first_site = get_point(firstNeuron);
    const ISite last_site = get_point(lastNeuron);

    // Find direction index in identity for a unit step (sx, sy), or -1
    auto find_dir = [&](int sx, int sy) -> int {
        for (int s = 0; s < identity.length; ++s)
            if (identity.horizontal[s] == sx && identity.vertical[s] == sy)
                return s;
        return -1;
    };

    // Trace a Bresenham grid ray from a to b using legal steps of the map
    auto trace_line = [&](const ISite &a, const ISite &b, weight_t &out_cost, std::vector<ISite> *out_cells) -> bool {
        out_cost = 0;
        ISite cur = a;
        int dx = std::abs(b.x - a.x);
        int dy = std::abs(b.y - a.y);
        int sx = (b.x > a.x) - (b.x < a.x);
        int sy = (b.y > a.y) - (b.y < a.y);
        int err = dx - dy;

        while (cur.x != b.x || cur.y != b.y)
        {
            int e2 = 2 * err;
            int step_x = 0, step_y = 0;
            if (e2 > -dy)
                step_x = sx;
            if (e2 < dx)
                step_y = sy;

            int dir = find_dir(step_x, step_y);
            ISite next {cur.x + step_x, cur.y + step_y};
            if (dir >= 0 && contains(next) && !has_lock(next) && _find_can_step(cur, dir))
            {
                if (step_x != 0)
                    err -= dy;
                if (step_y != 0)
                    err += dx;
                cur = next;
                out_cost += identity.g_weight[dir];
                if (out_cells)
                    out_cells->push_back(cur);
                continue;
            }

            // If diagonal step was requested but either not in identity (e.g. PlusMethod) or blocked by a corner,
            // try stepping along the primary axis first
            if (step_x != 0 && step_y != 0)
            {
                bool prefer_x = (dx >= dy);
                int ox = prefer_x ? step_x : 0;
                int oy = prefer_x ? 0 : step_y;
                int odir = find_dir(ox, oy);
                ISite onext {cur.x + ox, cur.y + oy};
                if (odir >= 0 && contains(onext) && !has_lock(onext) && _find_can_step(cur, odir))
                {
                    if (ox != 0)
                        err -= dy;
                    else
                        err += dx;
                    cur = onext;
                    out_cost += identity.g_weight[odir];
                    if (out_cells)
                        out_cells->push_back(cur);
                    continue;
                }
            }
            return false;
        }
        return true;
    };

    immune_system::open_queue<INeuron> openList;

    firstNeuron->g = 0;
    firstNeuron->h = _heuristic_fn(first_site.x - last_site.x, first_site.y - last_site.y);
    firstNeuron->f = firstNeuron->h;
    firstNeuron->parent = nullptr;
    firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
    openList.push({firstNeuron->f, firstNeuron});

    while (!openList.empty())
    {
        immune_system::open_entry<INeuron> entry = openList.top();
        openList.pop();
        INeuron *current = entry.neuron;
        if (current->flags == NEURON_CAPTURE_CLOSED_LIST || entry.f != current->f)
            continue;

        current->flags = NEURON_CAPTURE_CLOSED_LIST;
        ISite cur_site = get_point(current);
        if (_trace)
            _trace->push_back(cur_site);

        if (current == lastNeuron)
        {
            std::vector<ISite> anchors;
            for (INeuron *n = current; n != nullptr; n = n->parent)
                anchors.push_back(get_point(n));
            std::reverse(anchors.begin(), anchors.end());

            std::vector<ISite> full_sites;
            full_sites.push_back(anchors.front());
            weight_t dummy_cost = 0;
            for (std::size_t i = 1; i < anchors.size(); ++i)
                trace_line(anchors[i - 1], anchors[i], dummy_cost, &full_sites);

            std::vector<INeuron *> full_path;
            full_path.reserve(full_sites.size());
            for (const ISite &site : full_sites)
                full_path.push_back(_node(site));
            _find_emit_nodes(navigationResult, full_path);
            break;
        }

        for (int s = 0; s < identity.length; ++s)
        {
            ISite self_site {cur_site.x + identity.horizontal[s], cur_site.y + identity.vertical[s]};
            INeuron *self = _node(self_site);
            if (self == nullptr || self->flags == NEURON_CAPTURE_CLOSED_LIST || has_lock(self_site))
                continue;
            if (!_find_can_step(cur_site, s))
                continue;

            INeuron *cand_parent = current;
            weight_t newG = current->g + identity.g_weight[s];

            if (current->parent != nullptr)
            {
                weight_t los_cost = 0;
                if (trace_line(get_point(current->parent), self_site, los_cost, nullptr))
                {
                    weight_t los_g = current->parent->g + los_cost;
                    if (los_g <= newG)
                    {
                        cand_parent = current->parent;
                        newG = los_g;
                    }
                }
            }

            if (self->flags == NEURON_CAPTURE_OPEN_LIST)
            {
                if (newG >= self->g)
                    continue;
            }
            else
            {
                self->flags = NEURON_CAPTURE_OPEN_LIST;
                self->h = _heuristic_fn(self_site.x - last_site.x, self_site.y - last_site.y);
            }

            self->parent = cand_parent;
            self->g = newG;
            self->f = self->g + self->h;
            openList.push({self->f, self});
        }
    }

    return navigationResult.status == navigation_status::Opened;
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_ida(navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;

    const ISite first_site = get_point(firstNeuron);
    const ISite last_site = get_point(lastNeuron);
    const weight_t inf = std::numeric_limits<weight_t>::max() / 2;

    INeuron *base = this->front();
    std::unordered_map<std::size_t, weight_t> best_g;

    weight_t bound = _heuristic_fn(first_site.x - last_site.x, first_site.y - last_site.y);

    struct frame
    {
        INeuron *neuron;
        ISite site;
        int next_dir;
    };
    std::vector<frame> stack;

    for (;;)
    {
        best_g.clear();
        weight_t next_bound = inf;

        firstNeuron->g = 0;
        firstNeuron->h = _heuristic_fn(first_site.x - last_site.x, first_site.y - last_site.y);
        firstNeuron->f = firstNeuron->h;
        firstNeuron->parent = nullptr;
        firstNeuron->flags = NEURON_CAPTURE_OPEN_LIST;
        best_g[static_cast<std::size_t>(firstNeuron - base)] = 0;

        if (firstNeuron->f > bound)
        {
            next_bound = firstNeuron->f;
        }
        else
        {
            stack.clear();
            stack.push_back({firstNeuron, first_site, 0});
            if (_trace)
                _trace->push_back(first_site);

            while (!stack.empty())
            {
                frame &top = stack.back();
                if (top.neuron == lastNeuron)
                {
                    _find_build_path(navigationResult, lastNeuron);
                    return true;
                }

                if (top.next_dir >= identity.length)
                {
                    top.neuron->flags = NEURON_CAPTURE_CLOSED_LIST;
                    stack.pop_back();
                    continue;
                }

                int s = top.next_dir++;
                ISite nb_site {top.site.x + identity.horizontal[s], top.site.y + identity.vertical[s]};
                INeuron *nb = _node(nb_site);
                if (nb == nullptr || nb->flags == NEURON_CAPTURE_OPEN_LIST || has_lock(nb_site))
                    continue;
                if (!_find_can_step(top.site, s))
                    continue;

                weight_t newG = top.neuron->g + identity.g_weight[s];
                weight_t h = _heuristic_fn(nb_site.x - last_site.x, nb_site.y - last_site.y);
                weight_t f = newG + h;
                if (f > bound)
                {
                    if (f < next_bound)
                        next_bound = f;
                    continue;
                }

                std::size_t nb_idx = static_cast<std::size_t>(nb - base);
                auto known = best_g.find(nb_idx);
                if (known != best_g.end() && newG >= known->second)
                    continue;

                best_g[nb_idx] = newG;
                nb->g = newG;
                nb->h = h;
                nb->f = f;
                nb->parent = top.neuron;
                nb->flags = NEURON_CAPTURE_OPEN_LIST;
                if (_trace)
                    _trace->push_back(nb_site);
                stack.push_back({nb, nb_site, 0});
            }
        }

        if (next_bound >= inf)
            return false;
        bound = next_bound;
    }
}

BRAIN_TEMPLATE
template <typename list_type>
bool BRAIN_DEFINE::_find_bidirectional(
    pathfinder_algo pathfinder, navigate_result<list_type> &navigationResult, INeuron *firstNeuron, INeuron *lastNeuron)
{
    // Bidirectional search is meaningful for A*, Dijkstra and BFS; the rest search in one direction
    if (pathfinder == pathfinder_algo::BreadthFirstSearch)
        return this->_find_bfs_bidirectional(navigationResult, firstNeuron, lastNeuron);
    if (pathfinder == pathfinder_algo::DepthFirstSearch || pathfinder == pathfinder_algo::GreedyBestFirst
        || pathfinder == pathfinder_algo::JumpPointSearch || pathfinder == pathfinder_algo::ThetaStar
        || pathfinder == pathfinder_algo::IDAStar)
        return this->_find_unidirectional(pathfinder, navigationResult, firstNeuron, lastNeuron);

    // Dijkstra is A* with a zero heuristic
    const bool use_heuristic = pathfinder != pathfinder_algo::Dijkstra;
    auto heuristic = [this, use_heuristic](weight_t dx, weight_t dy) -> weight_t {
        return use_heuristic ? _heuristic_fn(dx, dy) : 0;
    };

    if (!_find_begin(navigationResult, firstNeuron, lastNeuron))
        return false;
    if (_find_trivial(navigationResult, firstNeuron, lastNeuron))
        return true;

    const ISite first_site = get_point(firstNeuron);
    const ISite last_site = get_point(lastNeuron);

    // the backward side lives in the reusable cache of the map
    struct bwd_view
    {
        basic_brain_map *map;
        bwd_slot &operator[](std::size_t index)
        {
            return map->_bwd_at(index);
        }
    } bwd_info {this};
    INeuron *base_neuron = this->front();

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
    firstNeuron->h = heuristic(first_site.x - last_site.x, first_site.y - last_site.y);
    firstNeuron->f = firstNeuron->h;
    firstNeuron->parent = nullptr;
    firstNeuron->flags |= NEURON_CAPTURE_OPEN_LIST;
    openList_fwd.insert(firstNeuron);

    // Initialize backward
    std::size_t last_idx = static_cast<std::size_t>(lastNeuron - base_neuron);
    bwd_info[last_idx].g = 0;
    bwd_info[last_idx].h = heuristic(last_site.x - first_site.x, last_site.y - first_site.y);
    bwd_info[last_idx].f = bwd_info[last_idx].h;
    bwd_info[last_idx].parent = nullptr;
    lastNeuron->flags |= NEURON_CAPTURE_BWD_OPEN_LIST;
    openList_bwd.insert(lastNeuron);

    weight_t best_path_cost = std::numeric_limits<weight_t>::max() / 2;
    INeuron *meeting_neuron = nullptr;

    while (!openList_fwd.empty() && !openList_bwd.empty())
    {
        INeuron *top_fwd = *openList_fwd.begin();
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
                INeuron *self = _node(self_site);
                if (self == nullptr || (self->flags & NEURON_CAPTURE_CLOSED_LIST) || has_lock(self_site))
                    continue;

                if (!_find_can_step(current_site, s))
                    continue;

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
                    self->h = heuristic(self_site.x - last_site.x, self_site.y - last_site.y);
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
                INeuron *self = _node(self_site);
                if (self == nullptr || (self->flags & NEURON_CAPTURE_BWD_CLOSED_LIST) || has_lock(self_site))
                    continue;

                if (!_find_can_step(current_site, s))
                    continue;

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
                    bwd_info[self_idx].h = heuristic(self_site.x - first_site.x, self_site.y - first_site.y);
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
        std::vector<INeuron *> nodes;
        for (INeuron *n = meeting_neuron; n != nullptr; n = n->parent)
            nodes.push_back(n);
        std::reverse(nodes.begin(), nodes.end());
        for (INeuron *n = bwd_info[static_cast<std::size_t>(meeting_neuron - base_neuron)].parent; n != nullptr;
             n = bwd_info[static_cast<std::size_t>(n - base_neuron)].parent)
            nodes.push_back(n);

        _find_emit_nodes(navigationResult, nodes);
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
