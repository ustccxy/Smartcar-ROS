/*
 * @Description: 
 * @Author: sunm
 * @Github: https://github.com/sunmiaozju
 * @LastEditors: sunm
 * @Date: 2019-04-29 16:22:31
 * @LastEditTime: 2019-04-29 22:22:26
 */

#include "lidar_euclidean_cluster.h"

namespace LidarDetector {
/**
 * @description: 点云发布函数 
 */
void LidarClusterDetector::pubPointCloud(
    const ros::Publisher& publisher,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_pointcloud)
{
    sensor_msgs::PointCloud2 msg_pointcloud;
    pcl::toROSMsg(*in_pointcloud, msg_pointcloud);
    msg_pointcloud.header = msg_header;
    publisher.publish(msg_pointcloud);
}

void LidarClusterDetector::pubClusters(const std::vector<ClusterPtr>& in_clusters,
    const ros::Publisher& pub)
{
}
}