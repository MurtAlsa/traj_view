//
// Created by redwan on 5/7/22.
//

#ifndef TRAJ_VIEW_STATE_TRANSITION_H
#define TRAJ_VIEW_STATE_TRANSITION_H

#include <vector>
#include <math.h>
#include <tuple>
#include <iostream>
#include <memory>
#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
using namespace std;

class StateTransition;
typedef shared_ptr<StateTransition> StateTransitionPtr;



class StateTransition{
public:
    StateTransition(double dt):dt_(dt)
    {
        this->max_w_ = 0.7 * 2;
        this->max_v_ = 1.5 * 2;
        initialize_ = false; 
    }
    void set_control(const std::pair<double, double>& cmd_vel)
    {
        lock_guard<mutex> lk(mu_);
        double v = cmd_vel.first;
        double w = cmd_vel.second;
        
      

        if(initialize_)
        {
            state_[0] = ekf_state_[0];
            state_[1] = ekf_state_[1];
            state_[2] = ekf_state_[2];
            initialize_ = false; 
        }
        else
        { 
            double theta = state_[2] + w * dt_;
            state_[0] += v * cos(theta) * dt_ * 1.1;
            state_[1] += v * sin(theta) * dt_ * 1.3;          
            state_[2] = theta;
        }
        cmd_vel_ = make_pair(v, w);
    }
    void set_state(const vector<double>& state)
    {
        state_.clear();
        std::copy(state.begin(), state.end(), back_inserter(state_));
        ekf_state_.resize(state_.size());
    }

    void set_state_sub(const string& topic)
    {
        ROS_INFO_STREAM(topic);
        state_sub_ = nh_.subscribe(topic, 10, &StateTransition::state_callback, this);
    }

    void set_max_vel(double vmax)
    {
        max_v_ = vmax;
    }

    void set_max_yaw_rate(double wmax)
    {
        max_w_ = wmax;
    }
    template <typename T>
    void update_cmd_vel(T* data)
    {
        data = new T{cmd_vel_.first, cmd_vel_.second};
    }

    template <typename T>
    void update_state(T& data)
    {
        for (int i = 0; i < state_.size(); ++i) {
            data[i] = state_[i];
        }
    }
    double clip_vel(double v)
    {
        return clip(v, max_v_);
    }
    double clip_yaw_rate(double w)
    {
        return clip(w, max_w_);
    }
protected:

    void state_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        lock_guard<mutex> lk(mu_);

        tf::StampedTransform transform;
        try{
            listener_.lookupTransform("/map", "/roomba20/base_link",  
                                ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

    //     // update x, y coordinage
    //     // state_[0] = msg->pose.pose.position.x;
    //     // state_[1] = msg->pose.pose.position.y;
    //    float x, y, theta; 
    //     x = msg->pose.pose.position.x;
    //     y = msg->pose.pose.position.y;
    //     // update orientation
    //     tf::Quaternion q;
    //     q.setX(msg->pose.pose.orientation.x);
    //     q.setY(msg->pose.pose.orientation.y);
    //     q.setZ(msg->pose.pose.orientation.z);
    //     q.setW(msg->pose.pose.orientation.w);

    //     theta = q.getAngle();
    //     // theta = q.getAngle();
    //     float dx, dy, dtheta; 
    //     dx = abs(ekf_state_[0]-x);    
    //     dy = abs(ekf_state_[1]-y); 
    //     dtheta = abs(ekf_state_[2] - theta);



        // ROS_INFO("[Odom] robot coord = (%f, %f) theta = %f", ekf_state_[0], ekf_state_[1], ekf_state_[2]);
        // ROS_INFO("[EKF] robot coord = (%f, %f) theta = %f", x, y, theta);
        // ROS_INFO("[Reading] robot coord = (%f, %f) theta = %f", dx, dy, dtheta);

        // ekf_state_[0] = x; 
        // ekf_state_[1] = y; 
        // ekf_state_[2] = theta;
        initialize_ = true; 
    }

    double clip(double val, double clip_val){
        auto sign = (val < 0)? -1 : 1;
        auto x = abs(val);
        val = min(x, clip_val) * sign;
        return val;
    }
private:
    vector<double> state_, ekf_state_;
    std::pair<double, double> cmd_vel_;
    double dt_, max_v_, max_w_;
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    mutex mu_;
    bool initialize_;
    tf::TransformListener listener_;
};

#endif //TRAJ_VIEW_STATE_TRANSITION_H
