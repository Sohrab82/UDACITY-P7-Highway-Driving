#ifndef _PRED_HELPERS_H_
#define _PRED_HELPERS_H_
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "classifier.h"

using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

// Helper functions to load .txt files
vector<vector<double>> Load_State(string file_name);
vector<string> Load_Label(string file_name);

// Load state from .txt file
vector<vector<double>> Load_State(string file_name)
{
    ifstream in_state_(file_name.c_str(), ifstream::in);
    vector<vector<double>> state_out;
    string line;

    while (getline(in_state_, line))
    {
        std::istringstream iss(line);
        vector<double> x_coord;

        string token;
        while (getline(iss, token, ','))
        {
            x_coord.push_back(stod(token));
        }
        state_out.push_back(x_coord);
    }

    return state_out;
}

// Load labels from .txt file
vector<string> Load_Label(string file_name)
{
    ifstream in_label_(file_name.c_str(), ifstream::in);
    vector<string> label_out;
    string line;
    while (getline(in_label_, line))
    {
        std::istringstream iss(line);
        string label;
        iss >> label;

        label_out.push_back(label);
    }

    return label_out;
}
#endif