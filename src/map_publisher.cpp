#include <string>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "map_loader.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>


class MapPublisher{
    ros::NodeHandle nh;
    ros::Publisher pub_map;
    ros::Subscriber sub_pose;
    sensor_msgs::PointCloud2::Ptr map_cloud;
    ros::Timer timer;
    MapLoader loader;
public:
    MapPublisher(ros::NodeHandle _nh, const std::string map_path)
        :map_cloud(new sensor_msgs::PointCloud2), loader(map_path)
    {
        std::string pose_topic, map_topic;
        this->nh = _nh;
        loader.setSearchRadius(100.);
        pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
        sub_pose = nh.subscribe("/lidar_pose", 1, &MapPublisher::pose_cb, this);
        timer = nh.createTimer(ros::Duration(1.), &MapPublisher::timer_cb, this, false, false);

        ROS_INFO("%s initialized", ros::this_node::getName().c_str());
    }
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
        // ROS_INFO("pose cb");
        pcl::PointXYZ center;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        center.x = pose->pose.position.x;
        center.y = pose->pose.position.y;
        center.z = pose->pose.position.z;
        int status = loader.getSubmaps(center, cloud);
        if(status == STATUS::FAIL){
            ROS_ERROR("Loading submap fail");
        }else if(status == STATUS::SAME){
            ROS_INFO("Use same map");
        }else{ // status == STATUS::NEW
            ROS_INFO("New submap published");
            pcl::toROSMsg(*cloud, *map_cloud);
            map_cloud->header.frame_id = "world";
            pub_map.publish(*map_cloud);
            timer.start();
        }
        return;
    }

    void timer_cb(const ros::TimerEvent& event){
        ROS_INFO("Timer triggered");
        map_cloud->header.stamp = ros::Time::now();
        ROS_INFO("Point cloud size: %d", map_cloud->width);
        pub_map.publish(*map_cloud);
    }
};

int main(int argc, char * argv[]){
    std::string map_path;
    ros::init(argc, argv, "map_publisher");
    ros::NodeHandle n("~");
    n.param<std::string>("map_path", map_path, "/home/biomotion/nuscenes_maps");

    MapPublisher publisher(n, map_path);
    ros::spin();
    return 0;

}