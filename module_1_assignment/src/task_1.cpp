#include <iostream>
using namespace std;

namespace RobotNamespace::R1
{
    class Robot
    {
    private:
        string name;
        int speed;
        struct Physical
        {
            double weight;
            double size;
            int NumberOfSensor;

        } physical;

    public:
        Robot(string n, double s, double w, double size, int sensor)
        {
            name = n;
            speed = s;
            physical.weight = w;
            physical.size = size;
            physical.NumberOfSensor = sensor;
        }

        void display()
        {
            cout << "Robot Name = " << name << endl;
            cout << "Speed = " << speed << " m/s" << endl;
            cout << "Weight = " << physical.weight << " kg" << endl;
            cout << "size = " << physical.size << " cube meters" << endl;
            cout << "Number of sensors = " << physical.NumberOfSensor << endl;
        }

        void moveForward()
        {
            cout << "Robot = " << name << " is going forward" << endl;
        }

        void moveBackward()
        {
            cout << "Robot = " << name << " is moving backward" << endl;
        }

        void stoped()
        {
            cout << "Robot = " << name << " is stoped" << endl;
        }
    };
}

namespace RobotNamespace::R2
{
    class Robot
    {
    private:
        string name;
        int speed;
        struct Physical
        {
            double weight;
            double size;
            int NumberOfSensor;

        } physical;

    public:
        Robot(string n, double s, double w, double size, int sensor)
        {
            name = n;
            speed = s;
            physical.weight = w;
            physical.size = size;
            physical.NumberOfSensor = sensor;
        }

        void display()
        {
            cout << "Robot Name = " << name << endl;
            cout << "Speed = " << speed << " m/s" << endl;
            cout << "Weight = " << physical.weight << " kg" << endl;
            cout << "size = " << physical.size << " cube meters" << endl;
            cout << "Number of sensors = " << physical.NumberOfSensor << endl;
        }

        void moveForward()
        {
            cout << "Robot = " << name << " is going forward" << endl;
        }

        void moveBackward()
        {
            cout << "Robot = " << name << " is moving backward" << endl;
        }

        void stoped()
        {
            cout << "Robot = " << name << " is stoped" << endl;
        }
    };
}
int main()
{
    cout << "Running Task 1" << endl;
    RobotNamespace::R1::Robot Robot1Class("Robot_1", 2.5, 50, 0.43, 6);
    Robot1Class.display();
    Robot1Class.moveForward();
    Robot1Class.moveBackward();
    Robot1Class.stoped();

    RobotNamespace::R2::Robot Robot2Class("Robot_2", 5, 100, 90, 21);
    Robot2Class.display();
    Robot2Class.moveForward();
    Robot2Class.moveBackward();
    Robot2Class.stoped();
    return 0;
}