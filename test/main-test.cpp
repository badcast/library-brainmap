#include <iostream>

#define private public
#define protected public
#include <across.hpp>
#undef private
#undef protected

using namespace std;
using namespace across;

Brain map = across::Brain(100, 100); // sizes with 10 * 10 = 100 neurons
Brain::site_type t1, t2;

std::size_t maxIterate = map.size();

void test1();
void test2();
void test3();

int main()
{
    cout << " :: Init Brain Map :: " << endl;

    Brain::result_site result;

    Brain::site_type s, k;
    Brain::neuron_type *n, *m;

    n = map.front();
    s = map.get_point(n);
    m = map.get(s);
    k = map.get_point(m);

    std::thread th1(test1), th2(test2);
    // wait
    th1.join();
    th2.join();

    map.clear(true);

    test3();

    cout << "Calculating" << endl;
    map.find(result, {0, 2}, {2, 2}); // right
    cout << "\tResult.status = "
         << (result.status == NavigationStatus::Opened       ? "Opened"
                 : result.status == NavigationStatus::Closed ? "Closed"
                 : result.status == NavigationStatus::Locked ? "Locked"
                                                             : "Undefined")
         << endl;
    cout << "\tResult.connections.size = " << result.connections.size() << endl;
}

void test1()
{
    Brain::neuron_type *n1, *n2;
    Brain::site_type t1;
    std::size_t iterate = 0;
    std::size_t errors = 0;
    cout << "Test #1 - indexer:" << endl;
    for(n1 = map.front(), n2 = map.back(); n1 <= n2; ++n1)
    {
        t1 = map.get_point(n1);
        if(map.get(t1) != n1)
        {
            errors++;
            cout << " error haved on " << t1.x << "x" << t1.y << endl;
        }
        else
        {
            n1->flags = 128;
            n1->g = 2 + iterate;
            n1->h = 3 + iterate;
            n1->f = 4 + iterate;
        }
        iterate++;
    }
    iterate = 0;
    for(n1 = map.front(), n2 = map.back(); n1 <= n2; ++n1)
    {
        if(n1->g != 2 + iterate || n1->h != 3 + iterate || n1->f != 4 + iterate || n1->flags != 128)
        {
            errors++;
            t1 = map.get_point(n1);
            cout << " damaged on " << t1.x << "x" << t1.y << endl;
        }
        else
        {
            std::memset(n1, 255, sizeof(*n1));
        }
        iterate++;
    }

    if(iterate != maxIterate || errors != 0)
    {
        cout << "Error on size compare errors=" << errors << " iterated=" << iterate << " have=" << maxIterate << endl;
    }
    else
    {
        cout << "Iterated complete. Have no error." << endl;
    }
}

void test2()
{
    Brain::neuron_type *n1, *n2;
    Brain::site_type t1;
    std::size_t iterate = 0;
    std::size_t errors = 0;
    cout << "Test #2 - lock read/write:" << endl;
    for(n1 = map.front(), n2 = map.back(); n1 <= n2; ++n1)
    {
        if(map.has_lock(n1))
        {
            errors++;
            t1 = map.get_point(n1);
            cout << " damaged on " << t1.x << "x" << t1.y << endl;
        }
        else
        {
            map.set_lock(n1, true);
        }
        iterate++;
    }

    if(maxIterate != iterate)
    {
        cout << "No max iterate" << endl;
    }
    if(errors == 0)
    {
        cout << "Iterated complete. Have no error." << endl;
    }
}

void test3()
{
    Brain::neuron_type *n1, *n2;
    Brain::site_type t1;
    std::size_t errors = 0;
    int s;
    cout << "Test #3 - neightbours:" << endl;

    for(n1 = map.front(), n2 = map.back(); n1 <= n2; ++n1)
    {
        for(s = 0; s < map.identity.length; ++s)
        {
            t1 = map.get_point(n1);
            t1.x += map.identity.horizontal[s];
            t1.y += map.identity.vertical[s];

            int x = 0;
        }
    }

    if(errors == 0)
    {
        cout << "Iterated complete. Have no error." << endl;
    }
}
