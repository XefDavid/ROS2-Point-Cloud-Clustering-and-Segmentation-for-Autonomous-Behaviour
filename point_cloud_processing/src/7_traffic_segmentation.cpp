#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>


using namespace std::chrono_literals;
typedef pcl::PointXYZ PointT;

class VoxelGrid_filter : public rclcpp::Node
{
public:
  VoxelGrid_filter() : Node("minimal_publisher")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kitti/point_cloud", 10,
        std::bind(&VoxelGrid_filter::timer_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_cloud", 10);
  }

private:
  void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
  {
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*input_cloud, *pcl_cloud);

    // Voxel Filter
    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(pcl_cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_filter.filter(*voxel_cloud);

    // Road segmentation (normals + RANSAC)
    pcl::NormalEstimation<PointT, pcl::Normal> normals_estimator;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    pcl::PointCloud<pcl::Normal>::Ptr road_normals(new pcl::PointCloud<pcl::Normal>());

    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> road_seg_frm_normals;
    pcl::PointIndices::Ptr road_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr road_coefficients(new pcl::ModelCoefficients());
    pcl::ExtractIndices<PointT> road_extract_indices;
    pcl::PointCloud<PointT>::Ptr road_cloud(new pcl::PointCloud<PointT>);

    // Normals extraction
    normals_estimator.setSearchMethod(tree);
    normals_estimator.setInputCloud(voxel_cloud);
    normals_estimator.setKSearch(30);
    normals_estimator.compute(*road_normals);

    // Parameters for segmentation
    road_seg_frm_normals.setOptimizeCoefficients(true);
    road_seg_frm_normals.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    road_seg_frm_normals.setMethodType(pcl::SAC_RANSAC);
    road_seg_frm_normals.setNormalDistanceWeight(0.5);
    road_seg_frm_normals.setMaxIterations(100);
    road_seg_frm_normals.setDistanceThreshold(0.4);
    road_seg_frm_normals.setInputCloud(voxel_cloud);
    road_seg_frm_normals.setInputNormals(road_normals);
    road_seg_frm_normals.segment(*road_inliers, *road_coefficients);

    // Extraction
    road_extract_indices.setInputCloud(voxel_cloud);
    road_extract_indices.setIndices(road_inliers);
    road_extract_indices.setNegative(true);
    road_extract_indices.filter(*road_cloud);

    //=================================================== Traffic clustering ===================================================//
    pcl::PointCloud<PointT>::Ptr segmented_cluster(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr all_clusters(new pcl::PointCloud<PointT>);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ecludian_cluster_extractor;
    size_t min_cloud_threshold=110;
    size_t max_cloud_threshold=3000;

    struct BBOX{
        float x_min;
        float x_max;
        float y_min;
        float y_max;
        float z_min;
        float z_max;
        double r=1.0;
        double g=0.0;
        double b=0.0;
    };

    std::vector <BBOX> bboxes;

    //Ecludian based Clustering
    tree->setInputCloud(road_cloud);
    ecludian_cluster_extractor.setMinClusterSize(100);
    ecludian_cluster_extractor.setMaxClusterSize(2000);
    ecludian_cluster_extractor.setSearchMethod(tree);
    ecludian_cluster_extractor.setInputCloud(road_cloud);
    ecludian_cluster_extractor.extract(cluster_indices);

    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
      if (cluster_indices[i].indices.size() > min_cloud_threshold && cluster_indices[i].indices.size() <max_cloud_threshold)
      {
       pcl::PointCloud<PointT>::Ptr reasonable_cluster (new pcl::PointCloud<PointT>);
       pcl::ExtractIndices<PointT> extract;
       pcl::IndicesPtr indices(new std::vector<int>(cluster_indices[i].indices.begin(), cluster_indices[i].indices.end()));

       extract.setInputCloud(road_cloud);
       extract.setIndices(indices);
       extract.setNegative(false);
       extract.filter(*reasonable_cluster);

       //Bounding boxes drawing
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D<PointT>(*reasonable_cluster, min_pt, max_pt);

        pcl::PointXYZ center((min_pt[0] + max_pt[0]) / 2.0, (min_pt[1] + max_pt[1]) / 2.0, (min_pt[2] + max_pt[2]) / 2.0);
        BBOX bbox;
        bbox.x_min = min_pt[0];
        bbox.y_min = min_pt[1];
        bbox.z_min = min_pt[2];
        bbox.x_max = max_pt[0];
        bbox.y_max = max_pt[1];
        bbox.z_max = max_pt[2];

        bboxes.push_back(bbox);

      }
      
    }
    


    // Convert cloud to ros2 message
    sensor_msgs::msg::PointCloud2 voxel_cloud_ros2;
    pcl::toROSMsg(*road_cloud, voxel_cloud_ros2);
    voxel_cloud_ros2.header = input_cloud->header;

    publisher_->publish(voxel_cloud_ros2);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelGrid_filter>());
  rclcpp::shutdown();
  return 0;
}