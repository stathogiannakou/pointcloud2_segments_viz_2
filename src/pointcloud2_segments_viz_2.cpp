#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/colors.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>

ros::Publisher pub;
std::string base_link_frame;


void pc2s_callback (const pointcloud_msgs::PointCloud2_Segments& msg){

    sensor_msgs::PointCloud2 accumulator;

    sensor_msgs::PointCloud2 cluster_msgs;

    for (size_t i=0; i < msg.clusters.size(); i++){

        pcl::PCLPointCloud2 cloud2;
        pcl_conversions::toPCL( msg.clusters[i] , cloud2);

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromPCLPointCloud2(cloud2, cloud);

        for(size_t j=0; j < cloud.points.size(); j++){
            cloud.points[j].r = 0;
            cloud.points[j].g = 0; 
            cloud.points[j].b = 0;
        }


        pcl::PCLPointCloud2 clouds;
        pcl::toPCLPointCloud2(cloud, clouds);
        pcl_conversions::fromPCL(clouds, cluster_msgs);
        cluster_msgs.header.stamp = ros::Time::now();
        cluster_msgs.header.frame_id = base_link_frame;

        sensor_msgs::PointCloud2 tmp = sensor_msgs::PointCloud2(accumulator);

        pcl::concatenatePointCloud( cluster_msgs, tmp, accumulator);
    }

    pub.publish(accumulator);
}

int main (int argc, char** argv){
    ros::init (argc, argv, "pointcloud2_segments_viz_2");
    ros::NodeHandle n_;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    std::string input_topic;
    std::string out_topic;
    n_.param("pointcloud2_segments_viz_2/input_topic",input_topic, std::string("/new_pcl"));
    n_.param("pointcloud2_segments_viz_2/base_link_frame", base_link_frame, std::string("base_link"));
    n_.param("pointcloud2_segments_viz_2/out_topic", out_topic, std::string("pointcloud2_segments_viz_2/pointcloud2"));

    ros::Subscriber sub = n_.subscribe (input_topic, 1, pc2s_callback);

    pub = n_.advertise<sensor_msgs::PointCloud2> (out_topic, 1);

    ros::spin ();
}
