#ifndef TASK3_H
#define TASK3_H

class Temprature
{
private:
    int temperature;

public:
    void display(int);
};

class Distance
{
private:
    float Distance;

public:
    void display(float);
};
template <typename T>
class Template
{
public:
    void sensorData(T);
};

#endif