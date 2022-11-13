//
// Created by redwan on 4/26/22.
//

#include "controller.h"

controller::controller(const vector<double>&state, double kpRho, double kpAlpha, double kpBeta, double dt, double goal_thres) :
kp_rho_(kpRho), kp_alpha_(kpAlpha),kp_beta_(kpBeta), state_(state), dt_(dt), goal_thres_(goal_thres)
{
    this->goal_.resize(3);

}

void controller::set_points(double x, double y) {
    this->goal_[0] = x;
    this->goal_[1] = y;
    double dx = goal_[0] -  state_[0];
    double dy = goal_[1] -  state_[1];
    this->goal_[2] = atan2(dy, dx);
}

std::tuple<double, double, double>  controller::calc_control_command(double x_diff, double y_diff, double theta, double theta_goal) {

    double rho = sqrt(x_diff * x_diff + y_diff * y_diff);

    double a1 = (atan2(y_diff, x_diff) - theta + M_PI);
    a1 = a1 < 0 ? a1 + (2 * M_PI ) : a1;
    double a2 = (2 * M_PI );
    double alpha = fmod(a1, a2) - M_PI;

    double b1 = (theta_goal - theta - alpha + M_PI);
    b1 = b1 < 0 ? b1 + (2 * M_PI ) : b1;
    double beta = fmod(b1, a2) - M_PI;
    double v = kp_alpha_ * rho;
    double w = kp_alpha_ * alpha - kp_beta_ * beta;

    /*
     *  # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn
     */

    if(alpha > M_PI_2 || alpha < -M_PI_2)
        v = -v;

    return make_tuple(rho, v, w);
}

std::pair<double, double> controller::compute_control(StateTransitionPtr transition) {

    transition->update_state(state_);
    double x_diff = goal_[0] - state_[0];
    double y_diff = goal_[1] - state_[1];
    double v, w;
    std::tie(rho_, v, w) = calc_control_command(x_diff, y_diff, state_[2], goal_[2]);

    v = transition->clip_vel(v);
    w = transition->clip_yaw_rate(w);

    if(rho_ <= goal_thres_)
        v = w = 0.0;

    auto cmd_vel = make_pair(v, w);
    transition->set_control(cmd_vel);

    return cmd_vel;

}


bool controller::isFinished() {
//    std::cout << rho_ << std::endl;
    return rho_ <= goal_thres_;
}

vector<double> controller::get_state() {
    return state_;
}
