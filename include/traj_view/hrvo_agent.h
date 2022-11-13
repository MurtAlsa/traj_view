//
// Created by redwan on 4/26/22.
//

#ifndef MPCHRVO_HRVO_AGENT_H
#define MPCHRVO_HRVO_AGENT_H

#include <iostream>
#include <cmath>
#include "HRVO.h"
#include <memory>
#include <vector>
#include <list>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <string>
#include <algorithm>
#include <functional>
#include <chrono>

#include "controller.h"
using namespace hrvo;
using namespace std;




class Robot;
typedef shared_ptr<Robot> RobotPtr;
typedef shared_ptr<Simulator> SimPtr;
typedef std::function<void(int, Vector2)> CallbackFunc;
static condition_variable conditionVariable;
static mutex mu;

class Robot{
    using  PATH2D = list<Vector2> ;
public:
    static int robotID;
    bool processed;
public:
    /**
     * hrvo operates centralized fashion, this class will make it distributed system
     * @param init_pos initial position of the robot Vector2
     * @param radius radius_ of the robot
     * @param sim hrvo simulator
     */
    Robot(const Vector2& init_pos, float radius, SimPtr sim, const CallbackFunc& callback=nullptr);
    ~Robot();


    /**
     * each robot needs to follow as of waypoints to reach to its destination
     * @param path a 2D geometric path that needs to follow
     */
    void set_path(const vector<vector<float>> & path);
    /**
     * each robot will be running in a separate thread.
     * This function will operate robot in a separate thread
     */
    void start(StateTransitionPtr state);
    void update_position(const Vector2& pos);
    /**
     *
     * @return true if the path has finished
     */
    bool isFinished();
    /**
     *
     * @param kpRho primarily responsible for linear velocity
     * @param kpAlpha primarily responsible for angular velocity
     * @param kpBeta 0 responsible for orientation
     * @param dt control time
     */
    void set_pid_controller(double kpRho, double kpAlpha, double kpBeta, double dt);



protected:
    /**
     * when robot finish to reach one waypoint location,
     * the goal location will be set to next waypoints and
     * the previous waypoint will be removed from the list
     */
    void update_goal();
    /**
     * this is internal hrvo loop invoked by the start function
     * this function will execute motion for the robot
     */
    void run();
    /**
     * this function will run in a separate thread with a higher frequency loop
     * this function will generate control commands
     * @param callback from main function (entry point)
     */
    void control_loop(StateTransitionPtr state);


private:
    PATH2D track_path_;
    Vector2 curr_pos_, curr_goal_;
    float radius_;
    int id_, control_dt_;
    bool enable_controller_;
    SimPtr sim_;
    thread hrvo_loop_;
    thread *cntrl_loop_;
    unique_ptr<controller> cntrl_;
    const CallbackFunc& callback_;


};



typedef vector<vector<float>> WP; // short for waypoints
struct PIDGains{
    union{
        struct{
            double kpRho, kpAlpha, kpBeta;
        };
        struct{
            std::array<double, 3> values;
        };
    };
};
struct HRVOParams{
    float neighborDist;
    std::size_t maxNeighbors;
    float prefSpeed;
    float maxSpeed;
};

struct MissionSetup{
    vector<WP> paths;
    float robot_radius;
    float simulation_step;
    float control_step;
    PIDGains gain;
    HRVOParams param;
};


class MissionCoordinator{
public:
    explicit MissionCoordinator();
    void doSetup(const MissionSetup& mission,  const std::function<void(int, Vector2)>& callback=nullptr);
    /**
     * generate execution commands for multiple robots
     */
    void execute(vector<StateTransitionPtr>& states);

    virtual ~MissionCoordinator();

private:
    SimPtr sim_;
    float sim_time_;
    vector<RobotPtr> robots_;
};



#endif //MPCHRVO_HRVO_AGENT_H