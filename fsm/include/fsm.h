#ifndef FSM_H
#define FSM_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class FSM
{
public:
    // Constructors
    FSM();
    // Destructor
    virtual ~FSM();

    vector<string> successor_states();

    string state;
};

#endif // VEHICLE_H