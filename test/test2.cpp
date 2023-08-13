#include "test_header.h"

int main()
{

    cout << "Lock read/write:" << endl;
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
    int result = 0;
    if(maxIterate != iterate)
    {
        result = 1;
        cout << "No max iterate" << endl;
    }
    if(errors == 0)
    {
        cout << "Iterated complete. Have no error." << endl;
    }
    else
        result = 1;
    return result;

}
