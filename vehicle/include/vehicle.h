#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle
{
public:
    // Constructors
    Vehicle(){};

    // Destructor
    virtual ~Vehicle(){};

    int id;
    int lane;
    vector<double> x;
    vector<double> y;
    vector<double> vx;
    vector<double> vy;
    vector<double> s;
    vector<double> d;
    vector<double> t;
    float acc; // accelaration
};

#endif // VEHICLE_H