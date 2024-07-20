#include <iostream>
#include "task_3.h"
using namespace std;

void Temprature::display(int a)
{
    Temprature::temperature = a;
    cout << "temprature is = " << Temprature::temperature << endl;
}

void Distance::display(float b)
{

    Distance::Distance = b;
    cout << "Distance is = " << Distance::Distance << endl;
}

template <typename T>
void Template<T>::sensorData(T data)
{
    cout << "sensor data Template = " << data << endl;
}
int main()
{
    cout << "Running Task 3" << endl;
    Temprature temp_obj;
    int temp_data = 10;
    temp_obj.display(temp_data);

    float dis_data = 20.21;
    Distance dis_obj;
    dis_obj.display(dis_data);

    Template<int> sample_temp_obj;
    sample_temp_obj.sensorData(temp_data);

    Template<float> sample_obj;
    sample_obj.sensorData(dis_data);
    char c = 'c';
    Template<char> sample_char_obj;
    sample_char_obj.sensorData(c);
    return 0;
}