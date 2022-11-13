//
// Created by redwan on 4/26/22.
//

#ifndef MPCHRVO_CONTROLLER_H
#define MPCHRVO_CONTROLLER_H

#include "state_transition.h"
using namespace std;



class controller {
public:
    /**
     *
     * @param state initial state of the vehicle
     * @param kpRho primarily responsible for linear velocity
     * @param kpAlpha primarily responsible for angular velocity
     * @param kpBeta 0 responsible for orientation
     * @param dt control time
     */
    controller(const vector<double>&state, double kpRho, double kpAlpha, double kpBeta, double dt, double goal_thres = 0.01);
    void set_points(double x, double y);
    std::pair<double, double> compute_control(StateTransitionPtr transition);
    bool isFinished();
    /**
     * get robot state [x, y, theta]
     */
    vector<double> get_state();


private:
    double kp_rho_, kp_alpha_, kp_beta_;
    vector<double> state_, goal_;
    double dt_, rho_;
    const double goal_thres_;


protected:
    /**
     *
     * @param x_diff The position of target with respect to current robot position
                 in x direction
     * @param y_diff The position of target with respect to current robot position
                 in y direction
     * @param theta The current heading angle of robot with respect to x axis
     * @param theta_goal The target angle of robot with respect to x axis
     * Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
     */
    std::tuple<double, double, double> calc_control_command(double x_diff, double y_diff, double theta, double theta_goal);

};


#endif //MPCHRVO_CONTROLLER_H
