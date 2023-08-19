#include "test_header.h"

int main()
{
    cout << "Path-finding:" << endl;

    typename brain_map::result_neuron results;

    n1 = map.front();
    n2 = map.back();

    // generate maze
    map.create_maze();

    map.find(results, n1, n2);

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
