//
// Created by redwan on 5/6/22.
//

#include "hrvo_agent.h"
#include <iostream>
#include <memory>
#include "viz_plan.h"
#include "map_parser.h"
#include <map>

using namespace hrvo;
using namespace std;
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <fmt/format.h>
#define STATE_DIM (4)


int Robot::robotID = 0;

class RobotTracker{
public:
    // x, y , theta, mes
    using state = std::array<float, STATE_DIM>;
    RobotTracker(const vector<WP> &paths, const vector<StateTransitionPtr>& statesPtr) : statesPtr_(statesPtr) {
        auto node_name = ros::this_node::getName();
        auto current_path = ros::package::getPath("traj_view");

        string image_file;
        double low, high;
        vector<string> topics_cmds;

        string param_low = fmt::format("{}/map/low", node_name);
        string param_high = fmt::format("{}/map/high", node_name);
        string param_img = fmt::format("{}/map/img", node_name);
        string topic_viz = fmt::format("{}/map/topics/viz", node_name);
        string topic_field = fmt::format("{}/map/topics/field", node_name);
        string cmd_vels = fmt::format("{}/controller/topics", node_name);

        nh_.getParam(param_low, low);
        nh_.getParam(param_high, high);
        nh_.getParam(param_img, image_file);

        nh_.getParam(topic_viz, topic_viz);
        nh_.getParam(topic_field, topic_field);
        nh_.getParam(cmd_vels, topics_cmds);

        image_file = fmt::format("{}/{}", current_path, image_file);

        map_ = make_unique<MapParser>(image_file, low, high);
        viz_ = make_unique<VizPlan>(topic_viz);
        state_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(topic_field, 1);

        trakers_.resize(paths.size());
        for(const auto& topic: topics_cmds)
            cmd_pubs_.emplace_back(nh_.advertise<geometry_msgs::Twist>(topic, 10));


    }

    void hrvo_callback(int robotId, const Vector2& cmd_vel)
    {
        statesPtr_[robotId]->update_state(trakers_[robotId]);
        float z = map_->get_reading(trakers_[robotId][0], trakers_[robotId][1])/255.0;
        trakers_[robotId][3] = z;
        // ROS_INFO("[Reading] robot = %d coord = (%f, %f) z = %f", robotId, trakers_[robotId][0], trakers_[robotId][1], z);
        viz_->update_robot(robotId, Vector2{trakers_[robotId][0], trakers_[robotId][1]}, trakers_[robotId][2] + M_PI_2);
        publish_state_message(robotId);
        publish_cmd_message(robotId, cmd_vel);
    }
protected:
    void publish_state_message(int robotID)
    {
        geometry_msgs::Vector3Stamped msg;
        msg.header.frame_id = to_string(robotID);
        msg.header.stamp = ros::Time::now();
        msg.vector.x = trakers_[robotID][0];
        msg.vector.y = trakers_[robotID][1];
        msg.vector.z = trakers_[robotID][3];
        state_pub_.publish(msg);
    }

    void publish_cmd_message(int robotID, const Vector2& cmd_vel)
    {
        geometry_msgs::Twist msg;
        msg.linear.x = cmd_vel.getX();
        msg.angular.z = cmd_vel.getY();
        cmd_pubs_[robotID].publish(msg);
    }

private:
    vector<state> trakers_;
    unique_ptr<VizPlan> viz_;
    unique_ptr<MapParser> map_;
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    vector<ros::Publisher> cmd_pubs_;
    vector<StateTransitionPtr> statesPtr_;
};


void copy_path(const vector<float>& x, const vector<float>& y, WP& data)
{
    for (int i = 0; i < x.size(); ++i) {
        data.push_back(std::vector<float>{x[i], y[i]});
    }
}


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "traj_view_node");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("traj view node will start in 5s ...");
   
    // read parameters
    auto node_name = ros::this_node::getName();
    auto current_path = ros::package::getPath("traj_view");

    float sim_time,  radius;
    float neigh_dist, max_neigh, pref_speed, max_speed;

    nh.getParam(fmt::format("{}/hrvo/dt", node_name), sim_time);
    nh.getParam(fmt::format("{}/hrvo/radius", node_name), radius);
    nh.getParam(fmt::format("{}/hrvo/neighbor_dist", node_name), neigh_dist);
    nh.getParam(fmt::format("{}/hrvo/max_neighbors", node_name), max_neigh);
    nh.getParam(fmt::format("{}/hrvo/pref_speed", node_name), pref_speed);
    nh.getParam(fmt::format("{}/hrvo/max_speed", node_name), max_speed);


    vector<float> x0, y0, x1, y1;

    nh.getParam(fmt::format("{}/paths/x0", node_name), x0);
    nh.getParam(fmt::format("{}/paths/y0", node_name), y0);
    nh.getParam(fmt::format("{}/paths/x1", node_name), x1);
    nh.getParam(fmt::format("{}/paths/y1", node_name), y1);

    float max_v, max_w, contrl_time;
    nh.getParam(fmt::format("{}/controller/max_vel", node_name), max_v);
    nh.getParam(fmt::format("{}/controller/max_yaw_rate", node_name), max_w);
    nh.getParam(fmt::format("{}/controller/dt", node_name), contrl_time);


    vector<string> state_subs;
    nh.getParam(fmt::format("{}/controller/subs", node_name), state_subs);




    WP path1, path2;
    copy_path(x0, y0, path1);
    copy_path(x1, y1, path2);

//    // simulation setup

    vector<WP> paths{path1, path2};
    vector<StateTransitionPtr> states;
    int index = 0;
    for(const auto& p:paths)
    {
        auto state = make_shared<StateTransition>(contrl_time);
        state->set_state({p[0][0], p[0][1], M_PI_2});
        state->set_max_vel(max_v);
        state->set_max_yaw_rate(max_w);
        state->set_state_sub(state_subs[index++]);
        states.emplace_back(state);
    }
    this_thread::sleep_for(5s);

    PIDGains gain{};
    vector<double> gain_params;
    nh.getParam(fmt::format("{}/controller/gain", node_name), gain_params);
    std::copy_n(gain_params.begin(), gain_params.size(), gain.values.begin());
    HRVOParams param{neigh_dist,(size_t)max_neigh, pref_speed, max_speed};
    MissionSetup mission{paths, radius, sim_time, contrl_time, gain, param};

    RobotTracker traker({path1, path2}, states);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    MissionCoordinator missionCoordinator;
    missionCoordinator.doSetup(mission, [&](int robotId, const Vector2& cmd_vel){traker.hrvo_callback(robotId, cmd_vel);});
    missionCoordinator.execute(states);

    return 0;
}