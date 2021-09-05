#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

double Vehicle::vel()
{
    if (t.size() == 0)

        return 0;
    double vx_ = vx.back();
    double vy_ = vy.back();
    return sqrt(vx_ * vx_ + vy_ * vy_);
}