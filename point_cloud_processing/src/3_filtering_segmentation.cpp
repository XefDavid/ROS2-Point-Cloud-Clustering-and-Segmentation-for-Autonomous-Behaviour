#include <iostream>
// Tipos de puntos
#include <pcl/point_types.h>
// Lee y guarda archivos
#include <pcl/io/pcd_io.h>
// Reduce puntos
#include <pcl/filters/voxel_grid.h>
//Reduce puntos
#include <pcl/filters/passthrough.h>

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
    //Aquí es donde se guarda con le nombre que queremos la nube elegida.
    std::string output_name = "pass_xy_cloud.pcd";
    cloud_reader.read(path + cloud_name, *cloud);

    // voxel grid
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    // Estos valores son los que van a hacer la grilla más grande o mas pequeña
    voxel_filter.setLeafSize(0.5, 0.5, 0.5);
    voxel_filter.filter(*voxel_cloud);

    // Pass through Filter x
    pcl::PassThrough<PointT> passing_x;
    passing_x.setInputCloud(voxel_cloud);
    passing_x.setFilterFieldName("x");
    //el primer valor es la izquierda y el sugundo la derecha
    passing_x.setFilterLimits(-1.5,1.5);
    passing_x.filter(*passthrough_cloud);

    //passthrough Filter Y
    pcl::PassThrough<PointT> passing_y;
    passing_x.setInputCloud(passthrough_cloud);
    passing_x.setFilterFieldName("y");
    //el primer valor es la izquierda y el sugundo la derecha
    passing_x.setFilterLimits(-1.5,1.5);
    passing_x.filter(*passthrough_cloud);

    

    // clouds writing
    cloud_writer.write<PointT>(path + output_name, *passthrough_cloud, false);

    return 0;
}
