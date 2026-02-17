#include <iostream>
// Tipos de puntos
#include <pcl/point_types.h>
// Lee y guarda archivos
#include <pcl/io/pcd_io.h>
// Reduce puntos
#include <pcl/filters/voxel_grid.h>
// Reduce puntos
#include <pcl/filters/passthrough.h>

// librerias necesarias para hacer la segmentacion
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
typedef pcl::PointXYZRGB PointT;

int main()
{
    // creamos el punto compàrtido de datos especificos de pointT
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr voxel_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr passthrough_cloud(new pcl::PointCloud<PointT>);
    pcl::PCDReader cloud_reader;
    pcl::PCDWriter cloud_writer;

    // Reading the cloud
    std::string path = "/home/david/ros2_ws/src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/";
    std::string cloud_name = "tb3_world.pcd";
    // Aquí es donde se guarda con le nombre que queremos la nube elegida.
    std::string output_name = "pass_xy_cloud.pcd";
    cloud_reader.read(path + cloud_name, *cloud);

    // voxel grid
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    // Estos valores son los que van a hacer la grilla más grande o mas pequeña
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);

    //************************************************* */ Pass through Filter x

    pcl::PassThrough<PointT> passing_x;
    passing_x.setInputCloud(voxel_cloud);
    passing_x.setFilterFieldName("x");
    // el primer valor es la izquierda y el sugundo la derecha
    passing_x.setFilterLimits(-1.2, 1.2);
    passing_x.filter(*passthrough_cloud);

    // passthrough Filter Y
    pcl::PassThrough<PointT> passing_y;
    passing_x.setInputCloud(passthrough_cloud);
    passing_x.setFilterFieldName("y");
    // el primer valor es la izquierda y el sugundo la derecha
    passing_x.setFilterLimits(-1.4, 1.4);
    passing_x.filter(*passthrough_cloud);

    // ***************************************************** Planner segmentation
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Creamos una nube vacía que se va a rellenar con los puntos del plano
    pcl::PointCloud<PointT>::Ptr plane_segmented_cloud(new pcl::PointCloud<PointT>);
    pcl::SACSegmentation<PointT> plane_segmentor;
    pcl::ExtractIndices<PointT> indices_extractor;

    plane_segmentor.setInputCloud(passthrough_cloud);
    plane_segmentor.setModelType(pcl::SACMODEL_PLANE); // dile qué buscar
    plane_segmentor.setMethodType(pcl::SAC_RANSAC);    // dile cómo buscarlo
    plane_segmentor.setDistanceThreshold(0.01);        // dile qué tolerancia
    plane_segmentor.segment(*inliers, *coefficients);  // AQUÍ se ejecuta todo

    indices_extractor.setInputCloud(passthrough_cloud);
    indices_extractor.setIndices(inliers);
    indices_extractor.setNegative(true);
    indices_extractor.filter(*plane_segmented_cloud);

    // clouds writing
    cloud_writer.write<PointT>(path + output_name,*plane_segmented_cloud, false);

    //*************************************************** Writing cloud
    // cloud_saver("plane_segmented.pcd",path,plane_segmented_cloud);

    return 0;
}
