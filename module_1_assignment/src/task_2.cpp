#include <iostream>
using namespace std;

namespace RobotNamespace::R1
{
    class Robot
    {
    public:
        struct SensorReadings
        {
            int temperature;
            int distance;
        };

    private:
        string name;
        SensorReadings SensorValue, RandomData;

    public:
        void DisplaySensorReading(SensorReadings reData)
        {
            SensorValue.temperature = reData.temperature;
            SensorValue.distance = reData.distance;
            cout << "Robot 1 Data = " << "Temperature: " << SensorValue.temperature << " \u00B0C" << ", Distance: " << SensorValue.distance << " cm" << endl;
        }

        SensorReadings getRandomData()
        {
            RandomData.temperature = rand() % 100;
            RandomData.distance = rand() % 1000;
            return RandomData;
        }
    };
}

namespace RobotNamespace::R2
{
    class Robot
    {
    public:
        struct SensorReadings
        {
            int temperature;
            int distance;
        };

    private:
        string name;
        SensorReadings SensorValue, RandomData;

    public:
        void DisplaySensorReading(SensorReadings reData)
        {
            SensorValue.temperature = reData.temperature;
            SensorValue.distance = reData.distance;
            cout << "Robot 2 Data = " << "Temperature: " << SensorValue.temperature << " \u00B0C" << ", Distance: " << SensorValue.distance << " cm" << endl;
        }

        SensorReadings getRandomData()
        {
            RandomData.temperature = rand() % 100;
            RandomData.distance = rand() % 1000;
            return RandomData;
        }
    };
}

int main()
{
    cout << "Running Task 1" << endl;
    RobotNamespace::R1::Robot Robot1;
    RobotNamespace::R1::Robot::SensorReadings getData1;

    RobotNamespace::R2::Robot Robot2;
    RobotNamespace::R2::Robot::SensorReadings getData2;
    while (1)
    {
        getData1 = Robot1.getRandomData();
        Robot1.DisplaySensorReading(getData1);

        getData2 = Robot2.getRandomData();
        Robot2.DisplaySensorReading(getData2);
    }

    return 0;
}