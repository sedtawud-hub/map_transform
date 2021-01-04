#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

int main (int argc, char** argv)
{
  std::string in_pcd;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (argc != 8)
  {
    std::cout << "Please provide the input PCD and TF in the following format:" << '\n';
    std::cout << "./map_transform PCD_FILE X Y Z Roll Pitch Yaw" << '\n';
    exit(0);
  }
  else
  {
    in_pcd = argv[1];
    x = std::stod(argv[2]);
    y = std::stod(argv[3]);
    z = std::stod(argv[4]);
    roll = std::stod(argv[5]);
    pitch = std::stod(argv[6]);
    yaw = std::stod(argv[7]);

    Eigen::Translation3f tl_mtog(x, y, z);
    Eigen::AngleAxisf rot_x_mtog(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_mtog(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_mtog(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_mtog = (tl_mtog * rot_z_mtog * rot_z_mtog * rot_z_mtog).matrix();

    pcl::PointCloud<pcl::PointXYZI>::Ptr in_map (new pcl::PointCloud<pcl::PointXYZI>);
    if (!pcl::io::loadPCDFile<pcl::PointXYZI> (in_pcd, *in_map))
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr out_map (new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud (*in_map, *out_map, tf_mtog);
      // Change your directory here!!
      pcl::io::savePCDFileBinary("/home/patiphon/transformed_map.pcd", *out_map);
      return 0;
    }
    else
    {
      std::cout << "Could not load the PCD file!!" << '\n';
      exit(1);
    }
  }
}
