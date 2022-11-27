#include <ros/ros.h>
#include <ros/console.h>

#include <string>

#include <cmath>
#include <algorithm>

#include "nav_msgs/OccupancyGrid.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <tf/transform_listener.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PlanPath {

public: 
    PlanPath(ros::NodeHandle& nh);
    void Plan();
private: 
    void occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm);
    void odomCallback(const nav_msgs::Odometry& odom);


    void moveToGoal(geometry_msgs::Point p);
    void visualizePoints(std::vector<geometry_msgs::Point> p_vec);
    void fillPathRequest(nav_msgs::GetPlan::Request &request, geometry_msgs::Point p1, geometry_msgs::Point p2);

    nav_msgs::Path callPlanningService (ros::ServiceClient& serviceClient, nav_msgs::GetPlan& srv);

    void doneCb(const actionlib::SimpleClientGoalState& state);

    nav_msgs::OccupancyGrid map_;

    MoveBaseClient ac_;

    double cost_threshold_;
    std::vector<geometry_msgs::Point> p_vec_;
    std::vector<geometry_msgs::Point> orderedPoints;

    tf::TransformListener tf_;
    // costmap_2d::Costmap2DROS* planner_costmap_;

    visualization_msgs::MarkerArray markerArray_;

    ros::ServiceClient serviceClient_;

    nav_msgs::Odometry odom_;
    bool odomRec_ = false;
    bool allInit_ = false;
    bool calcOnce = false;

    int saveGoalIndex;

    ros::Publisher pub_path_;
    ros::Publisher pub_final_plan_;
    ros::Publisher pub_viz_goals_;
    ros::Subscriber sub_map_; 
    ros::Subscriber sub_odom_; 
};

PlanPath::PlanPath(ros::NodeHandle& nh):
    ac_("move_base", true) {

    geometry_msgs::Point p1;    p1.x = 1.64;   p1.y = -1.2;
    geometry_msgs::Point p2;    p2.x = 1.4;    p2.y = 1.5;
    geometry_msgs::Point p3;    p3.x = -0.3;   p3.y = -2.3;
    geometry_msgs::Point p4;    p4.x = -0.91;   p4.y = -1.8;

    p_vec_ =  {p1, p2, p3, p4};

    // Wait for the action server to come up so that we can begin processing goals
    while( !ac_.waitForServer(ros::Duration(5.0)) ) {
        ROS_INFO("[Path Plan]: Waiting for the move_base action server to come up");
    }

    // Init service query for make plan
    std::string service_name = "/move_base/make_plan";
    while(!ros::service::waitForService(service_name, ros::Duration(3.0))) {
        ROS_INFO("Waiting for service move_base/make_plan to become available");
    }

    serviceClient_ = nh.serviceClient<nav_msgs::GetPlan>(service_name, true);
    
    if(!serviceClient_) {
        ROS_FATAL("Could not initialize get plan service from %s", serviceClient_.getService().c_str());
        return;
    }   

    sub_map_ = nh.subscribe("map", 1, &PlanPath::occupency_grid_map_cb, this);
    sub_odom_ = nh.subscribe("odom", 1, &PlanPath::odomCallback, this);
    pub_path_ = nh.advertise<move_base_msgs::MoveBaseGoal> ("goal_points", 1);
    pub_final_plan_ = nh.advertise<nav_msgs::Path> ("final_plan", 1);
    pub_viz_goals_ = nh.advertise<visualization_msgs::MarkerArray> ("rviz/goal_points", 1);
    allInit_ = true;
}

void PlanPath::odomCallback(const nav_msgs::Odometry& odom) {
    odom_ = odom;
    odomRec_ = true;
} 

void PlanPath::fillPathRequest(
    nav_msgs::GetPlan::Request &request, geometry_msgs::Point p1, geometry_msgs::Point p2) {

    request.start.header.frame_id = "map";
    request.start.pose.position.x = p1.x;
    request.start.pose.position.y = p1.y;
    request.start.pose.orientation.w = 1.0;
    request.goal.header.frame_id = "map";
    request.goal.pose.position.x = p2.x;
    request.goal.pose.position.y = p2.y;
    request.goal.pose.orientation.w = 1.0;
    request.tolerance = 0.5;
}

nav_msgs::Path PlanPath::callPlanningService (ros::ServiceClient& serviceClient, nav_msgs::GetPlan& srv) {
    // Perform the actual path planner call 
    if(serviceClient.call(srv)) {
        if (!srv.response.plan.poses.empty()) {
            return srv.response.plan;
        } else {
            ROS_WARN("Got empty plan");
        }
    } else {
        ROS_ERROR("Failed to call service %s -is the robot moving?", serviceClient.getService().c_str());
    }
}

void PlanPath::occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm) {
    map_ = ogm;
}

void PlanPath::Plan() {

    if (map_.data.size() < 1 || !odomRec_ || !allInit_) {
        ROS_WARN_THROTTLE(3, "No Plan Possible");
        return;
    }

    if (!calcOnce) {

        geometry_msgs::Point robot_point = odom_.pose.pose.position;

        std::vector<nav_msgs::Path> pat_vec;
        std::vector<nav_msgs::Path> final_plan;
        nav_msgs::GetPlan srv;

        std::vector<geometry_msgs::Point> calc_vec = p_vec_;

        forEach (const geometry_msgs::Point& p, calc_vec) {
            fillPathRequest(srv.request, robot_point, p);   
            pat_vec.push_back(callPlanningService(serviceClient_, srv));
        }

        bool firstElement = true;
        saveGoalIndex = 0;

        while (calc_vec.size() > 0) {

            if (pat_vec.size() == 0) break;

            nav_msgs::Path shortest = pat_vec.at(0);
            geometry_msgs::Point shortestPoint = calc_vec.at(0);
            int shortest_index = 0;

            for (int i = 0; i < pat_vec.size(); i++) {
                if (shortest.poses.size() > pat_vec.at(i).poses.size()) {
                    shortest = pat_vec.at(i);
                    shortest_index = i;
                    shortestPoint = calc_vec.at(i);
                    if (firstElement) {
                        saveGoalIndex = i;
                    }
                }
            }

            firstElement = false;

            orderedPoints.push_back(calc_vec.at(shortest_index));
            calc_vec.erase(calc_vec.begin() + shortest_index);
            final_plan.push_back(shortest);
            pat_vec.clear();

            for (const geometry_msgs::Point& p: calc_vec) {
                fillPathRequest(srv.request, shortestPoint, p);   
                pat_vec.push_back(callPlanningService(serviceClient_, srv));
            }
        }
        
        nav_msgs::Path pub_plan;

        pub_plan.header.frame_id = "map";
        pub_plan.header.stamp = ros::Time::now();
        forEach (const nav_msgs::Path& path, final_plan) {
            forEach(const geometry_msgs::PoseStamped &p, path.poses) {
                pub_plan.poses.push_back(p);
            }
        }
        
        pub_final_plan_.publish(pub_plan);

        visualizePoints(orderedPoints);
    
        calcOnce = true;
    }

    moveToGoal(orderedPoints.at(0));
    // moveToGoal(orderedPoints.at(1));
    // moveToGoal(orderedPoints.at(2));
    // moveToGoal(orderedPoints.at(3));

}

void PlanPath::doneCb(const actionlib::SimpleClientGoalState& state) {

    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        if (p_vec_.size() > 0)
            p_vec_.erase(orderedPoints.begin());        
    }

}

void PlanPath::moveToGoal(geometry_msgs::Point p) {

    // Create a new goal to send to move_base 
    move_base_msgs::MoveBaseGoal goal;
 
    // Send a goal to the robot
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = p.x;
    goal.target_pose.pose.position.y = p.y;
    goal.target_pose.pose.orientation.w = 1.0;

    // ac_.sendGoal(goal);
    // ac_.waitForResult();

    ac_.sendGoal(goal, boost::bind(&PlanPath::doneCb, this, _1), 
        MoveBaseClient::SimpleActiveCallback());
}

void PlanPath::visualizePoints(std::vector<geometry_msgs::Point> p_vec) {

    visualization_msgs::Marker pointMarker, textMarker;

    pointMarker.action = visualization_msgs::Marker::DELETEALL;    
    textMarker.action = visualization_msgs::Marker::DELETEALL;    

    textMarker.header.frame_id  = pointMarker.header.frame_id   = "map";
    textMarker.header.stamp     = pointMarker.header.stamp      = ros::Time();
    textMarker.ns               = pointMarker.ns                = "goal_points";
    textMarker.action           = pointMarker.action            = visualization_msgs::Marker::ADD;

    textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    pointMarker.type = visualization_msgs::Marker::SPHERE;

    ROS_WARN_STREAM(p_vec.size());

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

    ros::Rate loop_rate(1);

    while (ros::ok()) {

        PlanPath.Plan();

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;

    ros::spin();
}
