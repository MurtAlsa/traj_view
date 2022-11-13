//
// Created by redwan on 5/6/22.
//
#ifndef TRAJ_VIEW_VIZ_PLAN_H
#define TRAJ_VIEW_VIZ_PLAN_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>

class VizPlan{
public:

    VizPlan(const std::string& topic){
        state_pub_ = nh_.advertise<visualization_msgs::Marker>(topic, 100);
    }

    template<typename T>
    void update_robot(int robotID, const T& pos, double ori)
    {
        // generate a visualization marker message
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "/viz/state/robot" + std::to_string(robotID);
        marker.id = robotID;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.mesh_resource = "package://traj_view/config/ROOMBA.dae";

//         set up scale of the points
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 0.50;
        marker.color.r = 0.663;
        marker.color.g = 0.663;
        marker.color.b = 0.663;

        marker.pose.position.x = pos.getX();
        marker.pose.position.y = pos.getY();
        marker.pose.position.z = 0.0;


        tf::Quaternion q;
        q.setEuler(0, 0, ori);

        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        state_pub_.template publish(marker);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
};
#endif //TRAJ_VIEW_VIZ_PLAN_H
