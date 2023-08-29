#include "test_header.h"
#include <random>
#include <fstream>

constexpr int maxLevels = 10;
typename brain_map::result_neuron results;

void save_breakfast(const char *path, const brain_breakfast *breakfast, int resultScore)
{
    ofstream f {path};
    if(!f)
    {
        std::cerr << "error save to: " << path << std::endl;
        return;
    }
    f.write(reinterpret_cast<const char *>(&resultScore), sizeof(resultScore));
    f.write(reinterpret_cast<const char *>(&breakfast->widthSpace), sizeof(breakfast->widthSpace));
    f.write(reinterpret_cast<const char *>(&breakfast->heightSpace), sizeof(breakfast->heightSpace));
    f.write(reinterpret_cast<const char *>(&breakfast->len), sizeof(breakfast->len));
    f.write(static_cast<const char *>(breakfast->data), breakfast->len);
    f.close();
}

int load_breakfast(const char *path, brain_breakfast *breakfast)
{
    int resultScore;
    ifstream f {path};

    if(!f)
    {
        std::cerr << "error open from: " << path << std::endl;
        return -1;
    }

    if(!(f.read(reinterpret_cast<char *>(&resultScore), sizeof(resultScore)).good() && resultScore > 0 &&
         f.read(reinterpret_cast<char *>(&breakfast->widthSpace), sizeof(breakfast->widthSpace)).good() &&
         f.read(reinterpret_cast<char *>(&breakfast->heightSpace), sizeof(breakfast->heightSpace)).good() &&
         f.read(reinterpret_cast<char *>(&breakfast->len), sizeof(breakfast->len)).good() && breakfast->len > 0))
    {
        resultScore = -1;
        std::cerr << "error read maze" << std::endl;
    }
    else
    {
        char *maze_raw_data = static_cast<char *>(std::malloc(breakfast->len));
        if(maze_raw_data == nullptr)
        {
            resultScore = -1;
            std::cerr << "out of memory" << std::endl;
        }
        else
        {
            if(!f.read(maze_raw_data, breakfast->len).good())
                free(maze_raw_data);
            else
                breakfast->data = maze_raw_data;
        }
    }

    f.close();
    return resultScore;
}

the_neuron *get_free_neuron(the_neuron *candidate)
{
    brain_map::list_neuron neighbours = map.get_neighbours<brain_map::list_neuron>(MatrixIdentity::DiagonalMethod, candidate);
    for(auto iter = std::begin(neighbours); iter != std::end(neighbours); ++iter)
    {
        if(!map.has_lock(*iter))
        {
            candidate = *iter;
            break;
        }
    }

    return candidate;
}

void save_tests()
{
    int i;
    char buffer[32];
    std::srand(19992023);
    brain_breakfast breakfast;

    t1.x = 1;
    t1.y = 1;
    t2.x = map.get_width() - 2;
    t2.y = map.get_height() - 2;

    for(i = 0; i < maxLevels; ++i)
    {
        // generate maze
        map.create_maze();

        // calculate maze score
        map.find(results, get_free_neuron(map.get(t1)), get_free_neuron(map.get(t2)));
        map.clear();

        map.save(&breakfast);
        // save maze to
        snprintf(buffer, sizeof(buffer), "mazes/level-%d.maze", i + 1);

        save_breakfast(buffer, &breakfast, results.connections.size());

        // clear breakfast
        free(breakfast.data);
    }
}

int main()
{
    int i;
    char buffer[32];
    brain_breakfast breakfast;

    t1.x = 1;
    t1.y = 1;
    t2.x = map.get_width() - 2;
    t2.y = map.get_height() - 2;

    cout << "Path-finding:" << endl;

    maxIterate = maxLevels;
    for(i = 0; i < maxLevels; ++i)
    {
        // save maze to
        snprintf(buffer, sizeof(buffer), "mazes/level-%d.maze", i + 1);

        int scores = load_breakfast(buffer, &breakfast);
        if(scores == -1)
        {
            errors++;
            continue;
        }
        map.load(breakfast);

        // clear breakfast
        free(breakfast.data);

        // calculate maze score
        map.find(results, get_free_neuron(map.get(t1)), get_free_neuron(map.get(t2)));

        // show stats
        std::cout << "Maze file-name: " << buffer << std::endl;
        std::cout << "\tLoaded from scores: " << scores << std::endl;
        std::cout << "\tCalculated from scores: " << results.connections.size() << std::endl << std::endl;

        if(results.connections.size() != scores)
        {
            ++errors;
        }

        ++iterate;
    }

    int result = 0;
    if(iterate != maxIterate || errors != 0)
    {
        cout << "Error on size compare errors=" << errors << " iterated=" << iterate << " have=" << maxIterate << endl;
        result = 1;
    }
    else
    {
        cout << "Iterated complete. Have no error." << endl;
    }

    return result;
}
