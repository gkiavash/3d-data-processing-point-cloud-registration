#ifndef MVS_REG_REGISTRATION_H
#define MVS_REG_REGISTRATION_H
#include <iostream>
#include <fstream>
#include <tuple>
#include <Eigen/Dense>
#include "open3d/Open3D.h"



class Registration {

public:
  Registration(std::string cloud_source_filename, std::string cloud_target_filename); //load point cloud from given path
  Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target);
  void draw_registration_result();
  open3d::pipelines::registration::RegistrationResult execute_global_registration(double voxel_size = 0.007);
  open3d::pipelines::registration::RegistrationResult execute_icp_registration(double threshold = 0.02, double relative_fitness = 1e-6, double relative_rmse = 1e-6, int max_iteration = 1000);
  void set_transformation(Eigen::Matrix4d init_transformation);
  Eigen::Matrix4d get_transformation();
  void write_tranformation_matrix(std::string filename);
  void save_merged_cloud(std::string filename);



private:
  static void preprocess(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down_ptr, std::shared_ptr<open3d::pipelines::registration::Feature> &pcd_fpfh);
  open3d::geometry::PointCloud source_;
  open3d::geometry::PointCloud target_;
  Eigen::Matrix4d transformation_ = Eigen::Matrix4d::Identity();

};





#endif //MVS_REG_REGISTRATION_H
