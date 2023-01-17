// declarations Across Map (algorithm) part-file

template <typename Tp = NeuronPoint, typename TpNeuron = NeuronMember>
class PointerUtils
{
public:
    /// Определяет дистанцию точки от A до точки B
    /// Используется формула Пифагора "(a^2) + (b^2) = c^2"
    ///\par lhs Первоначальная точка
    ///\par rhs Конечная точка
    ///\return Сумма
    static inline int distance_phifagor(const Tp& lhs, const Tp& rhs)
    {
        return pow(lhs.x - rhs.x, 2) + pow(lhs.y - rhs.y, 2); // a->x * a->y + b->x * b->y;
    }

    /// Определяет дистанцию точки от A до точки B
    ///\par lhs Первоначальная точка
    ///\par rhs Конечная точка
    ///\return Сумма
    static inline int distance_manhtatten(const Tp& lhs, const Tp& rhs) { return abs(rhs.x - lhs.x) + abs(rhs.y - lhs.y); }

    /// Определяет, минимальную стоимость
    static auto get_minimum(basic_across_map<Tp>& map, std::list<Tp>* paths) -> decltype(std::begin(*paths))
    {
        int min = INTMAX_MAX;
        auto result = begin(*paths);
        for (auto i = result; i != std::end(*paths); ++i) {
            int g = map.neuronGetTotal(*i);
            if (g <= min) {
                min = g;
                result = i;
            }
        }
        return result;
    }
    static int get_matrix(NavMethodRule method, std::int8_t** matrixH, std::int8_t** matrixV)
    {
        static std::int8_t PLUS_H_POINT[] { -1, 1, 0, 0 };
        static std::int8_t PLUS_V_POINT[] { 0, 0, -1, 1 };

        static std::int8_t M_SQUARE_H_POINT[] { -1, 0, 1, -1, 1, -1, 0, 1 };
        static std::int8_t M_SQUARE_V_POINT[] { -1, -1, -1, 0, 0, 1, 1, 1 };

        static std::int8_t M_CROSS_H_POINT[] { -1, 1, -1, 1 };
        static std::int8_t M_CROSS_V_POINT[] { -1, -1, 1, 1 };

        switch (method) {
        case NavMethodRule::PlusMethod:
            *matrixH = PLUS_H_POINT;
            *matrixV = PLUS_V_POINT;
            return sizeof(PLUS_H_POINT);
            break;
        case NavMethodRule::SquareMethod:
            *matrixH = M_SQUARE_H_POINT;
            *matrixV = M_SQUARE_V_POINT;
            return sizeof(M_SQUARE_H_POINT);
            break;
        case NavMethodRule::CrossMethod:
            *matrixH = M_CROSS_H_POINT;
            *matrixV = M_CROSS_V_POINT;
            return sizeof(M_CROSS_H_POINT);
            break;
        }
        return 0;
    }

    /// Определяет, функцию пойска по направлениям. Таких как: left, up, right, down. etc.
    static void gets(basic_across_map<Tp, TpNeuron>& map, NavMethodRule method, Tp arrange, Tp target, std::list<Tp>* pathTo, std::size_t maxCount = -1, int filterFlag = -1)
    {
        TpNeuron* it = nullptr;
        Tp point;
        int i = 0, c;
        std::int8_t* matrixH;
        std::int8_t* matrixV;
        switch (method) {
        case NavMethodRule::NavigationIntelegency: {
            // TODO: Написать интелектуальный пойск путей для достижения лучших
            // результатов.
            // TODO: Приводить вектор направление для лучшего достижения.
            // TODO: Выводить оптимальный результат, чтобы было меньше итерации
            // TODO: Вывести итог и анализ скорости.
            c = get_matrix(NavMethodRule::SquareMethod, &matrixH, &matrixV);
            do {
                point.x = arrange.x + matrixH[i];
                point.y = arrange.y + matrixV[i];
                it = map.get(point);
                if (it && !map.get_nlocked(point) && (filterFlag == ~0 || it->flags & filterFlag)) {
                    if (point.x == target.x || point.y == target.y) {
                        if (point == target) {
                            i = c;
                            pathTo->clear();
                        }
                        pathTo->emplace_front(point);
                    } else
                        pathTo->emplace_back(point);
                }
                // next step
            } while (maxCount != pathTo->size() && i++ != c);
        } break;
        default:

            c = get_matrix(method, &matrixH, &matrixV);
            for (; i != c; ++i) {
                point.x = arrange.x + matrixH[i];
                point.y = arrange.y + matrixV[i];
                it = map.get(point);
                if (it && !map.get_nlocked(point) && (filterFlag == ~0 || it->flags & filterFlag)) {
                    pathTo->emplace_back(point);
                    if (maxCount == pathTo->size() || point == target)
                        break;
                }
            }

            break;
        }
    }
};
template <typename Tp, typename TpNeuron>
basic_across_map<Tp, TpNeuron>::basic_across_map(int lwidth, int lheight)
{
    if (!lwidth || !lheight)
        throw std::runtime_error("Width or Height is zero!");

    this->widthSpace = lwidth;
    this->heightSpace = lheight;

    std::div_t lockedDiv = div(std::max(lheight = (lwidth * lheight), 8), 8); // add locked bits
    this->segmentOffset = lwidth = lockedDiv.quot + lockedDiv.rem;
    lwidth = lheight * sizeof(TpNeuron) + lwidth + 100;
    this->neurons = static_cast<TpNeuron*>(std::malloc(lwidth));
}

template <typename Tp, typename TpNeuron>
basic_across_map<Tp, TpNeuron>::~basic_across_map()
{
    std::free(this->neurons);
}

template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::randomize(int flagFilter)
{
    std::uint32_t lhs, rhs = segmentOffset;
    clear(true);
    do {
        lhs = static_cast<std::uint32_t>(rand() & flagFilter);
        memcpy(reinterpret_cast<void*>(reinterpret_cast<std::size_t>(neurons) + segmentOffset - rhs), &lhs, std::min(rhs, (std::uint32_t)sizeof(long)));
        rhs -= std::min(rhs, static_cast<std::uint32_t>(sizeof(long)));
    } while (rhs > 0);
}
template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::stress()
{
    NavResult<Tp> result;
    Tp first, next = { static_cast<int>(widthSpace - 1), static_cast<int>(heightSpace - 1) };
    // TODO: next a strees
    find(result, NavMethodRule::NavigationIntelegency, first, next);
}
template <typename Tp, typename TpNeuron>
int basic_across_map<Tp, TpNeuron>::get_width()
{
    return widthSpace;
}
template <typename Tp, typename TpNeuron>
int basic_across_map<Tp, TpNeuron>::get_height()
{
    return heightSpace;
}
template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::clear(bool clearLocks)
{
    std::uint32_t length = widthSpace * heightSpace * sizeof(TpNeuron);
    std::uint32_t leftOffset;
    if (clearLocks) {
        leftOffset = 0;
        length += this->segmentOffset;
    } else {
        leftOffset = this->segmentOffset;
    }
    memset(reinterpret_cast<void*>(reinterpret_cast<std::size_t>(neurons) + leftOffset), 0, length);
}
template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::fill(bool fillLocks)
{
    std::uint32_t length = widthSpace * heightSpace * sizeof(TpNeuron);
    std::uint32_t leftoffset;
    if (!fillLocks) {
        leftoffset = this->segmentOffset;
    } else {
        leftoffset = 0;
        length += this->segmentOffset;
    }
    memset(reinterpret_cast<void*>(reinterpret_cast<std::size_t>(neurons) + leftoffset), 0xff, length);
}
template <typename Tp, typename TpNeuron>
TpNeuron* basic_across_map<Tp, TpNeuron>::get(int x, int y)
{
    return get({ x, y });
}
template <typename Tp, typename TpNeuron>
TpNeuron* basic_across_map<Tp, TpNeuron>::get(const Tp& range)
{
    TpNeuron* result = nullptr;
    if (contains(range))
        result = reinterpret_cast<TpNeuron*>((reinterpret_cast<std::size_t>(neurons)) + ((range.x * heightSpace + range.y) * sizeof(TpNeuron)) + segmentOffset);
    return result;
}

template <typename Tp, typename TpNeuron>
std::uint32_t basic_across_map<Tp, TpNeuron>::get_cached_size()
{
    std::uint32_t cal = 0;

    TpNeuron* n = reinterpret_cast<TpNeuron*>(reinterpret_cast<std::size_t>(neurons) + segmentOffset);
    TpNeuron* nM = n + widthSpace * heightSpace;
    while (n < nM) {
        cal += n->cost + n->h;
        ++n;
    }
    return cal;
}
template <typename Tp, typename TpNeuronMember>
TpNeuronMember* basic_across_map<Tp, TpNeuronMember>::front()
{
    return get(0, 0);
}
template <typename Tp, typename TpNeuron>
TpNeuron* basic_across_map<Tp, TpNeuron>::back()
{
    return get(widthSpace - 1, heightSpace - 1);
}

template <typename Tp, typename TpNeuron>
bool basic_across_map<Tp, TpNeuron>::get_nlocked(const Tp& range)
{
    if (!contains(range))
        throw std::out_of_range("range");
    auto divide = std::div(range.x * heightSpace + range.y, 8);
    auto pointer = reinterpret_cast<std::uint8_t*>(neurons) + divide.quot;
    return (*pointer) & (1 << divide.rem);
}

template <typename Tp, typename TpNeuron>
bool basic_across_map<Tp, TpNeuron>::contains(const Tp& range)
{
    return range.x < widthSpace && range.y < heightSpace && range.x > ~0 && range.y > ~0;
}

template <typename Tp, typename TpNeuron>
std::uint8_t& basic_across_map<Tp, TpNeuron>::get_nflag(const Tp& range)
{
    TpNeuron* n = get(range);
    if (!n)
        throw std::out_of_range("range");
    return n->flags;
}
template <typename Tp, typename TpNeuron>
std::uint32_t& basic_across_map<Tp, TpNeuron>::get_ncost(const Tp& range)
{
    TpNeuron* n = get(range);
    if (!n)
        throw std::out_of_range("range");
    return n->cost;
}
template <typename Tp, typename TpNeuron>
std::uint32_t& basic_across_map<Tp, TpNeuron>::get_nheuristic(const Tp& range)
{
    TpNeuron* n = get(range);
    if (!n)
        throw std::out_of_range("range");
    return n->h;
}
template <typename Tp, typename TpNeuron>
const int basic_across_map<Tp, TpNeuron>::get_nweight(const Tp& range)
{
    if (!contains(range))
        throw std::out_of_range("range");
    return range.x * range.y + range.y * range.y;
}
template <typename Tp, typename TpNeuron>
const std::uint32_t basic_across_map<Tp, TpNeuron>::get_ntotal(const Tp& range)
{
    TpNeuron* n = get(range);
    if (!n)
        throw std::out_of_range("range");
    return n->cost + n->h;
}
template <typename Tp, typename TpNeuron>
const bool basic_across_map<Tp, TpNeuron>::get_nempty(const Tp& range)
{
    return !neuronGetTotal(range);
}
template <typename Tp, typename TpNeuron>
bool basic_across_map<Tp, TpNeuron>::get_nlocked(const TpNeuron* neuron)
{
    return neuronLocked(get_point(neuron));
}
template <typename Tp, typename TpNeuron>
std::uint8_t& basic_across_map<Tp, TpNeuron>::get_nflag(const TpNeuron* neuron)
{
    return neuronGetFlag(get_point(neuron));
}
template <typename Tp, typename TpNeuron>
std::uint32_t& basic_across_map<Tp, TpNeuron>::get_ncost(const TpNeuron* neuron)
{
    return neuronGetCost(get_point(neuron));
}
template <typename Tp, typename TpNeuron>
std::uint32_t& basic_across_map<Tp, TpNeuron>::get_nheuristic(const TpNeuron* neuron)
{
    return neuronHeuristic(get_point(neuron));
}
template <typename Tp, typename TpNeuron>
const int basic_across_map<Tp, TpNeuron>::get_nweight(const TpNeuron* neuron)
{
    return neuronGetWeight(get_point(neuron));
}
template <typename Tp, typename TpNeuron>
const std::uint32_t basic_across_map<Tp, TpNeuron>::get_ntotal(const TpNeuron* neuron)
{
    return neuronEmpty(get_point(neuron));
}
template <typename Tp, typename TpNeuron>
const bool basic_across_map<Tp, TpNeuron>::get_nempty(const TpNeuron* neuron)
{
    return neuronEmpty(get_point(neuron));
}

template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::set_nlock(const TpNeuron* neuron, const bool state)
{
    neuronLock(get_point(neuron), state);
}

template <typename Tp, typename TpNeuron>
const Tp basic_across_map<Tp, TpNeuron>::get_npoint(const TpNeuron* neuron)
{
    auto divide = std::div((std::size_t(neuron) - std::size_t(neurons) - segmentOffset) / sizeof(TpNeuron), heightSpace);
    return { divide.quot, divide.rem };
}

template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::set_nlock(const Tp& range, const bool state)
{
    auto divide = std::div(range.x * heightSpace + range.y, 8);
    auto pointer = (reinterpret_cast<std::uint8_t*>(neurons) + segmentOffset) + divide.quot;
    divide.quot = (1 << divide.rem);
    // TODO: Optimized to ~divide.quout (xor)
    (*pointer) ^= (*pointer) & (divide.quot);
    (*pointer) |= divide.quot * (state == true);
}
template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::load(const AcrossData& AcrossData)
{
    if (!AcrossData.widthSpace || !AcrossData.heightSpace)
        throw std::runtime_error("Argument param, width or height is empty");

    if (AcrossData.neurons == nullptr)
        throw std::runtime_error("Data is null");

    if (this->neurons == AcrossData.neurons)
        throw std::runtime_error("neurons == Across data, unflow effect");

    if (this->neurons != nullptr)
        std::free(this->neurons);

    this->widthSpace = AcrossData.widthSpace;
    this->heightSpace = AcrossData.heightSpace;
    this->neurons = static_cast<TpNeuron*>(std::malloc(widthSpace * heightSpace * sizeof(TpNeuron)));

    memcpy(this->neurons, AcrossData.neurons, widthSpace * heightSpace * sizeof(TpNeuron));
}
template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::save(AcrossData* AcrossData)
{
    if (AcrossData == nullptr)
        return;

    AcrossData->widthSpace = this->widthSpace;
    AcrossData->heightSpace = this->heightSpace;
    AcrossData->neurons = this->neurons;
}
template <typename Tp, typename TpNeuron>
void basic_across_map<Tp, TpNeuron>::find(NavigateionResult& navResult, NavMethodRule method, TpNeuron* firstNeuron, TpNeuron* lastNeuron)
{
    this->find(navResult, method, this->get_npoint(firstNeuron), this->get_npoint(lastNeuron));
}
template <typename TpNeuronPoint, typename TpNeuron>
void basic_across_map<TpNeuronPoint, TpNeuron>::find(NavigateionResult& navResult, NavMethodRule method, TpNeuronPoint first, TpNeuronPoint last)
{
    using PointerUtils = PointerUtils<TpNeuronPoint, TpNeuron>;
    TpNeuron *current, *firstNeuron, *lastNeuron, *select;
    navResult.map = this;
    navResult.firstNeuron = first;
    navResult.lastNeuron = last;

    if ((firstNeuron = get(first)) == nullptr || (lastNeuron = get(last)) == nullptr) {
        navResult.status = NavStatus::Undefined;
        return;
    }
    if (get_nlocked(first) || get_nlocked(last)) {
        navResult.status = NavStatus::Locked;
        return;
    }

    enum : std::uint8_t { FLAG_CLEAN = 0, FLAG_OPEN_LIST = 1, FLAG_CLOSED_LIST = 2, FLAG_TILED_LIST = 4 };

    std::list<TpNeuronPoint> finded;
    typename std::list<TpNeuronPoint>::iterator iter, p1, p2;
    navResult.RelativePaths.clear();
    navResult.RelativePaths.emplace_back(first);
    firstNeuron->h = PointerUtils::distance_manhtatten(first, last);

    // Перестройка
    navResult.status = NavStatus::Opened;
    while (!navResult.RelativePaths.empty()) {
        iter = navResult.RelativePaths.begin(); // get the best Neuron
        current = get(iter.operator*());
        if (current == lastNeuron) {
            break;
        }

        current->flags = FLAG_CLOSED_LIST; // change to closing list
        navResult.RelativePaths.erase(iter);

        // Avail

        PointerUtils::gets(*this, method, this->get_npoint(current), last, &finded);
        for (iter = std::begin(finded); iter != std::end(finded); ++iter) {
            select = get(*iter);
            if (select->flags == FLAG_CLEAN) // free path
            {
                select->flags = FLAG_OPEN_LIST;
                select->cost = current->cost + 1;
                select->h = PointerUtils::distance_manhtatten((*iter), last);

                navResult.RelativePaths.emplace_back((*iter));
                p1 = std::begin(navResult.RelativePaths);
                p2 = std::end(navResult.RelativePaths);

                for (; p1 != p2;) {
                    if (get_ntotal(*p1) > get_ntotal(*iter)) {
                        navResult.RelativePaths.emplace(p1, (*iter));
                        break;
                    }
                    ++p1;
                }
                navResult.RelativePaths.emplace_back((*iter));
            }
        }

        finded.clear(); // clear data
    }

    TpNeuron* lastSelect = nullptr;
    current = lastNeuron;
    navResult.RelativePaths.clear();
    navResult.RelativePaths.emplace_back(last);
    while (current != firstNeuron) {
        if (current == lastSelect) {
            navResult.status = NavStatus::Closed;
            break;
        }
        lastSelect = current;
        PointerUtils::gets(*this, method, get_npoint(current), first, &finded, -1, FLAG_OPEN_LIST | FLAG_CLOSED_LIST);
        for (iter = std::begin(finded); iter != std::end(finded); ++iter) {
            select = get(*iter);
            if ((select->cost && select->cost < current->cost) || select == firstNeuron) {
                current = select;
                navResult.RelativePaths.emplace_front(*iter);
                current->flags = FLAG_TILED_LIST;
            }
        }

        finded.clear();
    }
}

bool operator!=(const NeuronPoint& a, const NeuronPoint& b) { return (bool)std::memcmp(&a, &b, sizeof(NeuronPoint)); }

bool operator==(const NeuronPoint& a, const NeuronPoint& b) { return !std::memcmp(&a, &b, sizeof(NeuronPoint)); }
