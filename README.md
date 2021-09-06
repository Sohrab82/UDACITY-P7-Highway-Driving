# Path Planning Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
The goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Files
#### Vehicle.h, Vehicle.cpp
These files define a class Vehicle that is used to keep track of sensor fustion reported objects. The class keeps values of x, y, vx, vy, s, d, and time for one seond for each object. At the moment, time series of these variables are not used in the project and only the last time step is used.

#### Tracker.h, Tracker.cpp
These files implement a tracker class. The tracker has an `update` function that updates the data received from sensor fustion about a vehicle. This function is called for every sensor fusion object from `main.cpp` main loop. The `Analyze_scene()` function

    for (auto it = sensor_fusion.begin(); it != sensor_fusion.end(); it++)
    {
        tracker.update_object(now, (*it));
    }

After updating the tracker, the main loop calls `tracker.analyze_scene()`. This function detects any vehicles around the ego vehicle. The area around the ego vehicle has been divided into 8 zones: 
- front: same lane in the front up until MAX_DIST_FRONT=80m
- rear: same lane in the back up until MAX_DIST_REAR=24m
- front right, front left, rear left, rear right: in the neighboring lanes with the same detection range as above
- blind spots on the right and left side: on the sides of the car, extending from `s-BS_FRONT_MARGIN` to `s+BS_REAR_MARGIN`. The two constants BS_FRONT_MARGIN and BS_REAR_MARGIN are set to 4 and 12 respectively.

The tracker has functions to returns each detected vehicle in the 8 zones. For example, to retrive the front object,  
    Vehicle *front_object();
can be used. The function returns NULL if no object is detected in the front. Equivalently, `front_id` class member can be checked to see if any front object was detected. In case there is an object, this variable will have the `id` of the object reported by sensor fusion, otherwise it will be -1.

analyze_scene() function loops over all sensor fusion (SF) objects:
    
    map<int, Vehicle>::iterator it;
    for (it = objects.begin(); it != objects.end(); it++)

For each object, `occupied_lanes()` is called. This function returns the lanes that are occupied fully or partially by the object. In another word, a vehicle can be considered in more than one lane.

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

Two more functions in this class are `can_change_right()` and `can_change_left()` functions. 
`can_change_left()` function returns false if
- the ego car is on the left line
- the blind spot is occupied,
- if a vehicle in the rear left zone and has a velocity of more that 95% the ego vehicle velocty,
- if there is a front left vehicle with a smaller velocity that the ego vehicle.
    
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


#### 

- src/main.cpp
- object_tracker/object_tracker.cpp
- vecicle/vehicle.cpp

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
