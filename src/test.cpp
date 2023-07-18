//to test the std::array container

#include <array>
#include <iostream>
#include <vector>

using namespace std;

int main()
{   
    // array<int, 4> arr = {1,2,3,4};
    vector<array<int, 3>> vec = {{1,2,3}, {1,2,3}, {2,3,4}};
    vec.push_back({0,0,0});

    for(int i=0; i < vec.size(); i++)
    {
        for(int j=0; j<vec[i].size(); j++)
        {
            cout << *(vec[i].data() + j) << " ";
        }
        cout << "\n";
    }
    return 0;
}