#include <ros/ros.h>
#include <ros/console.h>

#include <string>

#include <cmath>

#include "nav_msgs/OccupancyGrid.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <tf/transform_listener.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PlanPath {

public: 
    PlanPath(ros::NodeHandle& nh);
private: 
    void occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm);
    bool moveToGoal(geometry_msgs::Point p);
    void visualizePoints(std::vector<geometry_msgs::Point> p_vec);

    nav_msgs::OccupancyGrid map_;

    MoveBaseClient ac_;

    double cost_threshold_;
    std::vector<geometry_msgs::Point> p_vec_;

    tf::TransformListener tf_;
    // costmap_2d::Costmap2DROS* planner_costmap_;

    visualization_msgs::MarkerArray markerArray_;

    ros::Publisher pub_path_;
    // ros::Publisher pub_test_;
    ros::Publisher pub_viz_goals_;
    ros::Subscriber sub_map_; 
};

PlanPath::PlanPath(ros::NodeHandle& nh):
    ac_("move_base", true) {

    // Wait for the action server to come up so that we can begin processing goals
    while( !ac_.waitForServer(ros::Duration(5.0)) ) {
        ROS_INFO("[Path Plan]: Waiting for the move_base action server to come up");
    }

    if( !nh.getParam("cost_threshold", cost_threshold_) ) {
        ROS_ERROR_STREAM("[Path Plan]: cost_threshold can't be loaded");
    }

    sub_map_ = nh.subscribe("map", 1, &PlanPath::occupency_grid_map_cb, this);
    pub_path_ = nh.advertise<move_base_msgs::MoveBaseGoal> ("goal_points", 1);
    pub_viz_goals_ = nh.advertise<visualization_msgs::MarkerArray> ("rviz/goal_points", 1);
    // pub_test_ = nh.advertise<costmap_2d::Costmap2DROS> ("test", 1);
}

void PlanPath::occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm) {
    map_ = ogm;

    geometry_msgs::Point p1;    p1.x = -0.5;   p1.y = -0.5;
    geometry_msgs::Point p2;    p2.x = 2.0;    p2.y = 0.5;
    geometry_msgs::Point p3;    p3.x = -0.5;   p3.y = -1.5;
    geometry_msgs::Point p4;    p4.x = -0.5;   p4.y = 1.5;

    p_vec_ =  {p1, p2, p3, p4};

    visualizePoints(p_vec_);

    moveToGoal(p1);
    moveToGoal(p2);
    moveToGoal(p3);
    moveToGoal(p4);
}

bool PlanPath::moveToGoal(geometry_msgs::Point p) {

    // Create a new goal to send to move_base 
    move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = p.x;
    goal.target_pose.pose.position.y = p.y;
    goal.target_pose.pose.orientation.w = 1.0;

    ac_.sendGoal(goal);
    ac_.waitForResult();

    return ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void PlanPath::visualizePoints(std::vector<geometry_msgs::Point> p_vec) {

    visualization_msgs::Marker pointMarker, textMarker;

    textMarker.header.frame_id  = pointMarker.header.frame_id   = "map";
    textMarker.header.stamp     = pointMarker.header.stamp      = ros::Time();
    textMarker.ns               = pointMarker.ns                = "goal_points";
    textMarker.action           = pointMarker.action            = visualization_msgs::Marker::ADD;

    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    pointMarker.type = visualization_msgs::Marker::SPHERE;

    int id = 0;
    int goal_count = 1;
    for_each(p_vec.begin(), p_vec.end(), [&](geometry_msgs::Point p) {
        pointMarker.id  = ++id;
        textMarker.id   = ++id;

        textMarker.pose.position.x = p.x;
        textMarker.pose.position.y = p.y;
        textMarker.pose.position.z = 0.4;
        textMarker.pose.orientation.w = 1.0;
        textMarker.scale.x  = 0.3;
        textMarker.scale.y  = 0.3;
        textMarker.scale.z  = 0.3;
        textMarker.color.a  = 1.0; // Don't forget to set the alpha!
        textMarker.color.r  = 1.0;
        textMarker.color.g  = 1.0;
        textMarker.color.b  = 1.0;

        textMarker.text     = "Goal: " + std::to_string(goal_count++);

        pointMarker.pose.position.x = p.x;
        pointMarker.pose.position.y = p.y;
        pointMarker.scale.x = 0.1;
        pointMarker.scale.y = 0.1;
        pointMarker.scale.z = 0.1;
        pointMarker.color.a = 1.0; // Don't forget to set the alpha!
        pointMarker.color.r = 1.0;
        pointMarker.color.g = 0.0;
        pointMarker.color.b = 0.5;

        markerArray_.markers.push_back(textMarker);
        markerArray_.markers.push_back(pointMarker);
    });

    pub_viz_goals_.publish(markerArray_);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "gridmap_to_pcl");

    ros::NodeHandle nh;

    PlanPath PlanPath(nh);
    ros::spin();
}
