#include "tracker.h"

using std::cout;
using std::endl;

void ObjectTracker::update_object(vector<double> sf_object)
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
    v->x.push_back(x);
    v->y.push_back(y);
    v->vx.push_back(vx);
    v->vy.push_back(vy);
    v->s.push_back(s);
    v->d.push_back(d);
    while (v->x.size() > 50)
    {
        v->x.erase(v->x.begin());
        v->y.erase(v->y.begin());
        v->vx.erase(v->vx.begin());
        v->vy.erase(v->vy.begin());
        v->s.erase(v->s.begin());
        v->d.erase(v->d.begin());
    }
    cout << id << " " << objects[id].x.size() << endl;
}