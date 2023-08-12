#include "test_header.h"

int main() {
    cout << "Indexer:" << endl;
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
