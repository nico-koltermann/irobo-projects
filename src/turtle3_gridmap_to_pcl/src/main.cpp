#include <ros/ros.h>
#include <ros/console.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include <tf/transform_listener.h>

#include <laser_geometry/laser_geometry.h>

#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class TransformGridMap {

public: 
    TransformGridMap(ros::NodeHandle& nh);
private: 
    void occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm);
    void occupency_scan_cb(const sensor_msgs::LaserScan& in_scan);
    void odom_cb(const nav_msgs::Odometry& odom);
    void calculateLaser();
  
    sensor_msgs::LaserScan scan_save_;

    nav_msgs::Odometry odom_;

    nav_msgs::OccupancyGrid map_;
    tf::TransformListener listener_;
    laser_geometry::LaserProjection projector_;

    ros::Publisher pub_;
    ros::Publisher point_cloud_publisher_;
    ros::Publisher point_cloud_publisher2_;
    ros::Publisher point_cloud_publisher3_;

    ros::Subscriber sub_ogm_; 
    ros::Subscriber sub_scan_; 
    ros::Subscriber sub_odom_; 
    
    bool loadMapOnce = false;
};

TransformGridMap::TransformGridMap(ros::NodeHandle& nh) {
    sub_ogm_ = nh.subscribe("map", 1, &TransformGridMap::occupency_grid_map_cb, this);
    sub_scan_ = nh.subscribe("scan", 1, &TransformGridMap::occupency_scan_cb, this);
    sub_odom_ = nh.subscribe("/odometry/filtered", 1, &TransformGridMap::odom_cb, this);

    pub_ = nh.advertise<nav_msgs::Odometry> ("/scanmatch/odom", 1);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/input", 100, false);
    point_cloud_publisher2_ = nh.advertise<sensor_msgs::PointCloud2> ("/input_reference", 100, false);
    point_cloud_publisher3_ = nh.advertise<sensor_msgs::PointCloud2> ("/final", 100, false);
}

void TransformGridMap::odom_cb(const nav_msgs::Odometry& odom) {
    odom_ = odom;
}

void TransformGridMap::occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm) {
    map_ = ogm;
}

void TransformGridMap::calculateLaser() {
    geometry_msgs::PointStamped ps;
    geometry_msgs::PointStamped out_ps;

    sensor_msgs::PointCloud2 pcl_scan;
    sensor_msgs::PointCloud2 pcl_map;

    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = map_.header.frame_id;

    std::string base_frame = "base_link";

    PointCloud cluster;
    cluster.height  = 1;
    cluster.is_dense = true;
    cluster.header.frame_id = base_frame;

    projector_.transformLaserScanToPointCloud(base_frame, scan_save_, pcl_scan, listener_);

    for (int width = 0; width < map_.info.width; ++width)
    {
        for (int height = 0; height < map_.info.height; ++height)
        {
            if(map_.data[height*map_.info.width + width] > 0)
            {
                ps.point.x = width * map_.info.resolution - 10;
                ps.point.y = height * map_.info.resolution - 10;
                listener_.transformPoint(base_frame, ps , out_ps);

                pcl::PointXYZ pclP;
                pclP.x = out_ps.point.x;
                pclP.y = out_ps.point.y;
                pclP.z = 0.0;
                cluster.push_back(pclP);
            }
        }
    }

    pcl::toROSMsg(cluster, pcl_map);

    // Map and Scan
    point_cloud_publisher_.publish(pcl_map);
    point_cloud_publisher2_.publish(pcl_scan);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_scan( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map( new pcl::PointCloud<pcl::PointXYZ>() );

    pcl::fromROSMsg(pcl_scan, *point_cloud_scan);
    pcl::fromROSMsg(pcl_map, *point_cloud_map);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(point_cloud_scan);
    icp.setInputTarget(point_cloud_map);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    Eigen::Matrix4f transform = icp.getFinalTransformation();

    // Corrected Scan
    pcl::toROSMsg(Final, pcl_scan);
    pcl_scan.header.frame_id = "map"; 
    point_cloud_publisher3_.publish(pcl_scan);
    
    double r_x1 = transform.coeff(0, 0);
    double r_x2 = transform.coeff(0, 1);
    double t_x  = transform.coeff(0, 3);

    double r_y1 = transform.coeff(1, 0);
    double r_y2 = transform.coeff(1, 1);
    double t_y  = transform.coeff(1, 3);

    auto newOdom = odom_;

    const double x = newOdom.pose.pose.position.x;
    const double y = newOdom.pose.pose.position.y;

    newOdom.pose.pose.position.x = x * r_x1 - y * r_x2 + t_x;
    newOdom.pose.pose.position.y = x * r_y1 + y * r_y2 + t_y;

    pub_.publish(newOdom);
}

void TransformGridMap::occupency_scan_cb(const sensor_msgs::LaserScan& in_scan) {
    if (!(map_.data.size() == 0)) {
        scan_save_ = in_scan;
        loadMapOnce = true;

        calculateLaser();
    }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "gridmap_to_pcl");

  ros::NodeHandle nh;

  TransformGridMap TransformGridMap(nh);

  // Spin
  ros::spin ();
}
