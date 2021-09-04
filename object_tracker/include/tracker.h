#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H
#include <iostream>

#include "vehicle.h"
#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class ObjectTracker
{
public:
    map<int, Vehicle> objects;
    void update_object(vector<double> sf_object);
};

#endif