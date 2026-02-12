#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
    // pcl son puntos en 3D, con coordenadas x, y, z pcl continene tres puntos para acada objeto cloud creado
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.push_back(pcl::PointXYZ(1.0, 2.0, 3.0)); // Agregar esos valores a cloud con push_back
    cloud.push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    cloud.push_back(pcl::PointXYZ(7.0, 8.0, 9.0));
    cloud.push_back(pcl::PointXYZ(10.0, 11.0, 12.0));
    cloud.push_back(pcl::PointXYZ(12.0, 14.0, 15.0));
    cloud.push_back(pcl::PointXYZ(14.0, 15.0, 18.0));
    cloud.push_back(pcl::PointXYZ(16.0, 18.0, 21.0));
    
    std::string path = "/home/david/ros2_ws/src/ROS2-Point-Cloud-Clustering-and-Segmentation-for-Autonomous-Behaviour/point_cloud_processing/point_clouds/plane_cloud.pcd";

    // Guardar estos valores en el path x como el objeto cloud
    pcl::io::savePCDFileASCII(path, cloud);
    std::cout << cloud.size();

    return 0;
}