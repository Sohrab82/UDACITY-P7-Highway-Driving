#include "tracker.h"

using std::cout;
using std::endl;

void ObjectTracker::update_object(unsigned int now, vector<double> sf_object)
{
    int id = (int)sf_object[0];
    double x = sf_object[1];
    double y = sf_object[2];
    double vx = sf_object[3];
    double vy = sf_object[4];
    double s = sf_object[5];
    double d = sf_object[6];

    Vehicle *v;
    if (objects.find(id) != objects.end())
    {
        v = &objects.find(id)->second;
    }
    else
    {
        v = new Vehicle();
        v->id = id;
        objects[id] = *v;
        v = &objects.find(id)->second;
    }
    v->t.push_back(now);
    v->x.push_back(x);
    v->y.push_back(y);
    v->vx.push_back(vx);
    v->vy.push_back(vy);
    v->s.push_back(s);
    v->d.push_back(d);
    // I only keep history_ms=1000 msec of history for each object
    while ((v->t.size() > 50) || ((v->t.size() > 0) && ((now - v->t[0]) > history_ms)))
    {
        v->t.erase(v->t.begin());
        v->x.erase(v->x.begin());
        v->y.erase(v->y.begin());
        v->vx.erase(v->vx.begin());
        v->vy.erase(v->vy.begin());
        v->s.erase(v->s.begin());
        v->d.erase(v->d.begin());
    }
    cout << id << " " << objects[id].x.size() << endl;
}

vector<int> ObjectTracker::occupied_lanes(double d)
{
    // returns the lane numbers occupied by the vehicle
    // might returns two numbers in case the car is changing lane for example
    vector<int> lanes;
    int lane = (int)(d / 4);
    lanes.push_back(lane);

    double d_normalized = d - lane * 4; //  d in range (0.0, 4.0)
    if (d_normalized < 0.75)
        // left of the object is partially in the left lane
        lanes.push_back(lane - 1);
    if (d_normalized > 4 - 0.75)
        // right of the object is partially in the right lane
        lanes.push_back(lane + 1);
    return lanes;
}

void ObjectTracker::analyze_scene(double ego_s, double ego_d)
{
    lbs_occupied = false; // left blind spot (from s-10 to s+10)
    rbs_occupied = false; // right blind spot (from s-10 to s+10)
    front_id = -1;
    front_left_id = -1;
    front_right_id = -1;
    rear_id = -1;
    rear_left_id = -1;
    rear_right_id = -1;

    int ego_lane = (int)(ego_d / 4);
    map<int, Vehicle>::iterator it;
    for (it = objects.begin(); it != objects.end(); it++)
    {
        if (it->second.t.size() == 0)
            // object is not tracked anymore
            continue;
        double target_s = it->second.s.back();
        double target_d = it->second.d.back();
        vector<int> lanes = occupied_lanes(target_d);
        if (target_s > ego_s + BS_FRONT_MARGIN)
        {
            // analyze front left
            if (ego_lane != 0)
            {
                vector<int>::iterator lane_it = std::find(lanes.begin(), lanes.end(), ego_lane - 1);
                if (lane_it != lanes.end())
                {
                    if (front_left_id == -1)
                    {
                        front_left_id = it->first;
                    }
                    else
                    {
                        if (target_s < objects[front_left_id].s.back())
                            // closer to the ego vehicle
                            front_left_id = it->first;
                    }
                }
            }
            // analyze front right
            if (ego_lane != 2)
            {
                vector<int>::iterator lane_it = std::find(lanes.begin(), lanes.end(), ego_lane + 1);
                if (lane_it != lanes.end())
                {
                    if (front_right_id == -1)
                    {
                        front_right_id = it->first;
                    }
                    else
                    {
                        if (target_s < objects[front_right_id].s.back())
                            // closer to the ego vehicle
                            front_right_id = it->first;
                    }
                }
            }
            // analyze front
            vector<int>::iterator lane_it = std::find(lanes.begin(), lanes.end(), ego_lane);
            if (lane_it != lanes.end())
            {
                if (front_id == -1)
                {
                    front_id = it->first;
                }
                else
                {
                    if (target_s < objects[front_id].s.back())
                        // closer to the ego vehicle
                        front_id = it->first;
                }
            }
        }
        else if (target_s < ego_s + BS_REAR_MARGIN)
        {
            // analyze rear left
            if (ego_lane != 0)
            {
                vector<int>::iterator lane_it = std::find(lanes.begin(), lanes.end(), ego_lane - 1);
                if (lane_it != lanes.end())
                {
                    if (rear_left_id == -1)
                    {
                        rear_left_id = it->first;
                    }
                    else
                    {
                        if (target_s > objects[rear_left_id].s.back())
                            // closer to the ego vehicle
                            rear_left_id = it->first;
                    }
                }
            }
            // analyze rear right
            if (ego_lane != 2)
            {
                vector<int>::iterator lane_it = std::find(lanes.begin(), lanes.end(), ego_lane + 1);
                if (lane_it != lanes.end())
                {
                    if (rear_right_id == -1)
                    {
                        rear_right_id = it->first;
                    }
                    else
                    {
                        if (target_s > objects[rear_right_id].s.back())
                            // closer to the ego vehicle
                            rear_right_id = it->first;
                    }
                }
            }
            // analyze rear
            vector<int>::iterator lane_it = std::find(lanes.begin(), lanes.end(), ego_lane);
            if (lane_it != lanes.end())
            {
                if (rear_id == -1)
                {
                    rear_id = it->first;
                }
                else
                {
                    if (target_s > objects[rear_id].s.back())
                        // closer to the ego vehicle
                        rear_id = it->first;
                }
            }
        }
        else
        {
            // detect objects in belind spots
            if (!lbs_occupied)
            {
                if (ego_lane == 0)
                    lbs_occupied = true; // it is not occupied by a vehicle but still not available
                else
                    lbs_occupied = (std::find(lanes.begin(), lanes.end(), ego_lane - 1) != lanes.end());
            }
            if (!rbs_occupied)
            {
                if (ego_lane == 2)
                    rbs_occupied = true; // it is not occupied by a vehicle but still not available
                else
                    rbs_occupied = (std::find(lanes.begin(), lanes.end(), ego_lane + 1) != lanes.end());
            }
        }
    }
}