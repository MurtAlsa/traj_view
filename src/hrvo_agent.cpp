//
// Created by redwan on 4/26/22.
//
//#define HRVO_SIMULATION_DRIVE

#include "hrvo_agent.h"

Robot::Robot(const Vector2& init_pos, float radius, SimPtr sim, const CallbackFunc& callback ):
        curr_pos_(init_pos), curr_goal_(init_pos), sim_(sim), hrvo_loop_(), callback_(callback){
    this->radius_ = radius;
    this->id_ = robotID++;
    this->processed = false;
    this->enable_controller_ = false;
}



void Robot::set_path(const vector<vector<float>> & path)
{
    track_path_.clear();
    for(const auto& point: path)
    {
        track_path_.emplace_back(Vector2(point[0], point[1]));
    }
    sim_->addAgent(curr_pos_, sim_->addGoal(curr_goal_));
    update_goal();
}

void Robot::start(StateTransitionPtr state)
{
    hrvo_loop_ = thread(&Robot::run, this);
    if(enable_controller_)
        cntrl_loop_ = new thread(&Robot::control_loop,  this, state);

}


void Robot::run()
{


    while (!track_path_.empty() || !sim_->getAgentReachedGoal(id_))
    {
        unique_lock lk(mu);
        conditionVariable.wait(lk, [&](){return processed;});
        auto pos = sim_->getAgentPosition(id_);

        if(enable_controller_)
            cntrl_->set_points(pos.getX(), pos.getY());
        else if(callback_)
            callback_(id_, pos);

        if(!track_path_.empty())
            update_position( pos);

        processed = false;
        lk.unlock();

    }
    cout << "[robot]" << id_ << " reached its destination!! \n";
//    this_thread::sleep_for(1s);


}

void Robot::update_position(const Vector2& pos)
{
    curr_pos_ = pos;
    update_goal();
}
bool Robot::isFinished()
{
    return track_path_.empty() && cntrl_->isFinished();
//    return (enable_controller_)?track_path_.empty() && cntrl_->isFinished() : track_path_.empty();
}

void Robot::update_goal()
{
    auto diff = abs(track_path_.front() - curr_pos_);
    if (diff <= radius_)
    {
        track_path_.pop_front();
        if(track_path_.empty())return;
        // make sure not to update goal when track path is empty
        curr_goal_ = track_path_.front();
        sim_->setAgentGoal(id_, sim_->addGoal(curr_goal_));
        cout << "[Robot] " << id_ << " updating waypoint/goal " << curr_goal_ << endl;
    }

}

Robot::~Robot() {
    while(!hrvo_loop_.joinable())
        this_thread::sleep_for(100ms);

    hrvo_loop_.join();
    if(enable_controller_)
    {
        cntrl_loop_->join();
        delete cntrl_loop_;
    }

}

void Robot::set_pid_controller(double kpRho, double kpAlpha, double kpBeta, double dt) {

    double theta = M_PI_2;
    this->control_dt_ = dt * 1000; // milli sec

    vector<double> state{curr_pos_.getX(), curr_pos_.getY(), theta};
    cntrl_ = make_unique<controller>(state, kpRho, kpAlpha, kpBeta, dt, radius_/16);
    enable_controller_ = true;
}

void Robot::control_loop(StateTransitionPtr state)
{
    chrono::milliseconds cntrl_time(control_dt_);
    do
    {
        auto cmd_vel = cntrl_->compute_control(state);
        if(callback_)
        {

            callback_(id_, Vector2{(float)cmd_vel.first, (float)cmd_vel.second});
//            auto state = cntrl_->get_state();
//            callback_(id_, Vector2(state[0], state[1]));
        }
        this_thread::sleep_for(cntrl_time);
    }while(!isFinished());
    printf("[Robot %d] control loop finished \n", id_);
}


MissionCoordinator::MissionCoordinator() {


}

void MissionCoordinator::doSetup(const MissionSetup& mission, const std::function<void(int, Vector2)>& callback)
{
    sim_ = make_shared<Simulator>();
    robots_.clear();
    float radius = mission.robot_radius;
    sim_time_ = mission.simulation_step;
    sim_->setTimeStep(sim_time_);
    auto phrvo = mission.param;
    sim_->setAgentDefaults(phrvo.neighborDist, phrvo.maxNeighbors, radius, radius, phrvo.prefSpeed, phrvo.maxSpeed);

    for(auto path : mission.paths) {
        auto robot = make_shared<Robot>(Vector2(path[0][0], path[0][1]), radius, sim_, callback);
        robot->set_path(path);
#ifndef HRVO_SIMULATION_DRIVE
        auto gain_ = mission.gain;
        robot->set_pid_controller(gain_.kpRho, gain_.kpAlpha, gain_.kpBeta, mission.control_step);
#endif
        robots_.push_back(robot);
    }

}

void MissionCoordinator::execute(vector<StateTransitionPtr>& states)
{
    // start thread for each robot
    int count = 0;
    for (auto& robot:robots_) {
        robot->start(states[count++]);
    }

    bool allFinished = false;


    chrono::milliseconds sim_time(int(sim_time_ * 1000));

    do{
        vector<bool> process, finish;
        for (auto& robot:robots_) {
            process.push_back(robot->processed);
            finish.push_back(robot->isFinished());
        }
        // check current state of robots: are all robots running?
        bool allRunning = std::count(process.begin(), process.end(), false) == process.size();
        allFinished = std::count(finish.begin(), finish.end(), true) == finish.size();

        sim_->doStep();

        if(allFinished || allRunning)
        {
            for (auto& robot:robots_)
                robot->processed = true; // this will increase step for each robot
        }
        else
        {
            // if one robot isFinished and another robot is running process;
            for (int i = 0; i < process.size(); ++i) {
                for (int j = 0; j < finish.size(); ++j) {
                    if(!process[i] && finish[j] && i != j)
                    {
                        // this will increase step for alive robot
                        cout << "alived robot " << i << endl;
                        robots_[i]->processed = true;
                    }
                }
            }

        }
        conditionVariable.notify_all();
#ifndef HRVO_SIMULATION_DRIVE
        this_thread::sleep_for(sim_time);
#endif
    }while (!sim_->haveReachedGoals() || !allFinished);
    cout << "[MissionCoordinator] simulation finished !! \n";

}

MissionCoordinator::~MissionCoordinator() {
    robots_[0]->robotID = 0;
}

