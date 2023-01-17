#include <iostream>
#include <across.hpp>

int check_dmg(across::AcrossMap* map)
{
    int dmgCount = 0;
    auto x = map->front();

    while (x != map->back()) {
        if (x->cost || x->flags || x->h) {
            ++dmgCount;
        }
        ++x;
    }
    return dmgCount;
}

int main()
{
    using namespace std;
    using namespace across;

    cout << "Init Across Map :: write width and height" << endl;
    AcrossMap map = across::AcrossMap(105, 333);
    AcrossMap::NavigateionResult result;

    cout << "Calculating" << endl;
    int x = 1000, y = 0, z = 0;

    while (x--) {
        map.randomize(0xde);

        auto front = map.front();
        auto back = map.back();

        map.find(result, NavMethodRule::NavigationIntelegency, front, back);

        if (result.status == NavStatus::Opened && z < result.RelativePaths.size()) {
            ++y;
            z = result.RelativePaths.size();
        }
    }
    cout << "Damaged: " << check_dmg(&map) << endl;
    cout << "Status: " << y << " opened" << endl;
    cout << "Paths: " << z << endl;
    cout << "End." << endl;
}
