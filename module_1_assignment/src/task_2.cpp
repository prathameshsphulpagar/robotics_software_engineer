#include <iostream>
using namespace std;

namespace RobotNamespace::R1
{
    class Robot
    {
    private:
        string name;
    public:

    };
}

namespace RobotNamespace::R2
{
    class Robot
    {
    private:
        string name;

    public:
    };
}
int main()
{
    cout << "Running Task 1" << endl;
    RobotNamespace::R1::Robot Robot1;


    return 0;
}