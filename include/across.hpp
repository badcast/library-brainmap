#pragma once

#include <cstdint>
#include <list>
#include <stdexcept>

/**
 *
 * ASTAR Algorithm composed by RoninEngine (Navigation AI)
 * date: 18.01.2023
 *
 */

namespace across
{
    template <typename T> class AcrossMap;

    enum NavStatus { Undefined, Locked, Closed, Opened };

    enum NavMethodRule { NavigationIntelegency, PlusMethod, SquareMethod, CrossMethod };

    struct APoint {
        int x;
        int y;
    };

    template <typename Point = APoint>
    struct NavResult {
        NavStatus status;
        Point firstNeuron;
        Point lastNeuron;
        std::list<Point> RelativePaths;
        AcrossMap<Point>* map;
    };

    struct Neuron {
        std::uint8_t flags;
        std::uint32_t h;
        std::uint32_t cost;
    };

    struct AcrossData {
        std::uint32_t widthSpace;
        std::uint32_t heightSpace;
        void* neurons;
    };

    constexpr std::size_t AcrossDataSizeMultiplier = sizeof(Neuron);

    template <typename Tp = APoint> class AcrossMap
    {
        void* neurons;
        int segmentOffset;

    protected:
        int widthSpace, heightSpace;

    public:
        explicit AcrossMap(int width, int height);

        ~AcrossMap();

        void clear(bool clearLocks = false);
        void fill(bool fillLocks = false);
        void randomGenerate(int flagFilter = 0xffffff);
        void stress();

        int getWidth();
        int getHeight();

        Neuron* GetNeuron(int x, int y);
        Neuron* GetNeuron(const Tp& point);

        inline bool neuronContains(const Tp& point);

        inline const Tp neuronGetPoint(const Neuron* neuron);

        // pointer with Point
        inline bool neuronLocked(const Tp& point);
        inline std::uint8_t& neuronGetFlag(const Tp& point);
        inline std::uint32_t& neuronGetCost(const Tp& point);
        inline std::uint32_t& neuronHeuristic(const Tp& point);
        inline const int neuronGetWeight(const Tp& point);
        inline const std::uint32_t neuronGetTotal(const Tp& point);
        inline const bool neuronEmpty(const Tp& point);
        void neuronLock(const Tp& point, const bool state);

        // pointer with pointer
        bool neuronLocked(const Neuron* neuron);
        std::uint8_t& neuronGetFlag(const Neuron* neuron);
        std::uint32_t& neuronGetCost(const Neuron* neuron);
        std::uint32_t& neuronHeuristic(const Neuron* neuron);
        const int neuronGetWeight(const Neuron* neuron);
        const std::uint32_t neuronGetTotal(const Neuron* neuron);
        inline const bool neuronEmpty(const Neuron* neuron);
        void neuronLock(const Neuron* neuron, const bool state);

        void find(NavResult& navResult, NavMethodRule method, Neuron* firstNeuron, Neuron* lastNeuron, NavAlgorithm algorithm);

        void find(NavResult& navResult, NavMethodRule method, Tp first, Tp last, NavAlgorithm algorithm);

        void load(const AcrossData& AcrossData);

        void save(AcrossData* AcrossData);

        std::uint32_t getCachedSize();
    };
} // namespace RoninEngine::AIPathFinder
