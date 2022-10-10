#include <ros/ros.h>
#include <ros/console.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

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
    void calculateLaser();
  
    sensor_msgs::LaserScan output_scan_;
    sensor_msgs::LaserScan scan_save_;

    nav_msgs::OccupancyGrid map_;
    tf::TransformListener listener_;
    laser_geometry::LaserProjection projector_;

    ros::Publisher pub_;
    ros::Publisher point_cloud_publisher_;
    ros::Publisher point_cloud_publisher2_;
    ros::Subscriber sub_ogm_; 
    ros::Subscriber sub_scan_; 
    
    bool loadMapOnce = false;
};

TransformGridMap::TransformGridMap(ros::NodeHandle& nh) {
    sub_ogm_ = nh.subscribe("map", 1, &TransformGridMap::occupency_grid_map_cb, this);
    sub_scan_ = nh.subscribe("scan", 1, &TransformGridMap::occupency_scan_cb, this);
    pub_ = nh.advertise<sensor_msgs::LaserScan> ("output", 1);
    point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
    point_cloud_publisher2_ = nh.advertise<sensor_msgs::PointCloud2> ("/cloud2", 100, false);
}

void TransformGridMap::occupency_grid_map_cb(const nav_msgs::OccupancyGrid& ogm) {
    map_ = ogm;
}


void TransformGridMap::calculateLaser() {
    geometry_msgs::PointStamped ps;
    geometry_msgs::PointStamped out_ps;

    sensor_msgs::PointCloud2 pcl;
    sensor_msgs::PointCloud2 pcl2;

    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = map_.header.frame_id;

    PointCloud cluster;
    cluster.height  = 1;
    cluster.is_dense = true;
    cluster.header.frame_id = "map";

    projector_.transformLaserScanToPointCloud("base_scan", scan_save_, pcl2, listener_);

    for (int width = 0; width < map_.info.width; ++width)
    {
        for (int height = 0; height < map_.info.height; ++height)
        {
            if(map_.data[height*map_.info.width + width] > 0)
            {
                ps.point.x = width * map_.info.resolution - 10;
                ps.point.y = height * map_.info.resolution - 10;
                listener_.transformPoint("base_scan", ps , out_ps);

                pcl::PointXYZ pclP;
                pclP.x = out_ps.point.x;
                pclP.y = out_ps.point.y;
                pclP.z = 0.0;
                cluster.push_back(pclP);
            }
        }
    }

    pcl::toROSMsg(cluster, pcl);
    point_cloud_publisher_.publish(pcl);
    point_cloud_publisher2_.publish(pcl2);

    PointCloud::Ptr cluster1;
    PointCloud::Ptr cluster2;

    // pcl::fromROSMsg(pcl, *cluster1);
    // pcl::fromROSMsg(pcl2, *cluster2);

    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(cluster1);
    // icp.setInputTarget(cluster2);

    // pcl::PointCloud<pcl::PointXYZ> Final;
    // icp.align(Final);
    // std::cout << icp.getFinalTransformation() << std::endl;

}

void TransformGridMap::occupency_scan_cb(const sensor_msgs::LaserScan& in_scan) {
    if (!(map_.data.size() == 0)) {
        output_scan_ = in_scan;
        output_scan_.header.frame_id = "base_scan";
        scan_save_ = in_scan;
        output_scan_.ranges.clear();
        loadMapOnce = true;

        calculateLaser();
    }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "gridmap_to_pcl");

  ros::NodeHandle nh;

    ROS_WARN_STREAM("INIT ASFOIASONG");

  TransformGridMap TransformGridMap(nh);

  // Spin
  ros::spin ();
}
