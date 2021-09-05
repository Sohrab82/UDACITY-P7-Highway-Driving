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
    // cout << id << " " << objects[id].x.size() << endl;
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
    // don't assign false to these two variable in your code
    // if they become true, they should stay true
    lbs_occupied = false; // left blind spot (from s-10 to s+10)
    rbs_occupied = false; // right blind spot (from s-10 to s+10)
    front_id = -1;
    front_left_id = -1;
    front_right_id = -1;
    rear_id = -1;
    rear_left_id = -1;
    rear_right_id = -1;

    int ego_lane = (int)(ego_d / 4);

    if (ego_lane == 0)
        // no place on the left
        lbs_occupied = true;
    if (ego_lane == 2)
        // no place on the right
        rbs_occupied = true;

    map<int, Vehicle>::iterator it;
    for (it = objects.begin(); it != objects.end(); it++)
    {
        if (it->second.t.size() == 0)
            // object is not tracked anymore
            continue;
        double target_s = it->second.s.back();
        // out of range objects
        if ((target_s > MAX_DIST_FRONT + ego_s) || (target_s < ego_s - MAX_DIST_REAR))
            continue;

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
        else if (target_s < ego_s - BS_REAR_MARGIN)
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
                if (ego_lane != 0)
                    lbs_occupied = (std::find(lanes.begin(), lanes.end(), ego_lane - 1) != lanes.end());
            }
            if (!rbs_occupied)
            {
                if (ego_lane != 2)
                    rbs_occupied = (std::find(lanes.begin(), lanes.end(), ego_lane + 1) != lanes.end());
            }
        }
    }
    cout << lbs_occupied << " " << rbs_occupied << endl;
}

Vehicle *ObjectTracker::front_object()
{
    if (front_id == -1)
        return NULL;
    else
        return &objects[front_id];
};

Vehicle *ObjectTracker::front_left_object()
{
    if (front_left_id == -1)
        return NULL;
    else
        return &objects[front_left_id];
};

Vehicle *ObjectTracker::front_right_object()
{
    if (front_right_id == -1)
        return NULL;
    else
        return &objects[front_right_id];
};

Vehicle *ObjectTracker::rear_object()
{
    if (rear_id == -1)
        return NULL;
    else
        return &objects[rear_id];
};

Vehicle *ObjectTracker::rear_left_object()
{
    if (rear_left_id == -1)
        return NULL;
    else
        return &objects[rear_left_id];
};

Vehicle *ObjectTracker::rear_right_object()
{
    if (rear_right_id == -1)
        return NULL;
    else
        return &objects[rear_right_id];
};

bool ObjectTracker::can_change_left(double ego_s, double ego_d, double ego_vel)
{
    int ego_lane = (int)(ego_d / 4);
    if (ego_lane == 0)
        return false;
    if (lbs_occupied)
        return false;
    Vehicle *rlo = rear_left_object();
    if (rlo != NULL)
        if (rlo->vel() >= 0.95 * ego_vel)
            // change lane if the rear left object is driving at most 120% of your speed
            return false;

    // not to worry about rear left objects
    Vehicle *flo = front_left_object();
    if (flo == NULL)
        // no object in front (or rear) left to worry about
        return true;
    else
    {
        if (flo->vel() < ego_vel)
            // car in the left lane with smaller velocity
            return false;
        else
            return true;
    }
}

bool ObjectTracker::can_change_right(double ego_s, double ego_d, double ego_vel)
{
    int ego_lane = (int)(ego_d / 4);
    if (ego_lane == 2)
        return false;
    if (rbs_occupied)
        return false;

    Vehicle *rro = rear_right_object();
    if (rro != NULL)
        if (rro->vel() >= 0.95 * ego_vel)
            // change lane if the rear right object is driving at most 120% of your speed
            return false;

    // not to worry about rear left objects
    Vehicle *fro = front_right_object();
    if (fro == NULL)
        // no object in front (or rear) left to worry about
        return true;
    else
    {
        if (fro->vel() < ego_vel)
            // car in the left lane with smaller velocity
            return false;
        else
            return true;
    }
}