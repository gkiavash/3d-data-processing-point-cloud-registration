#include "Registration.h"


Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename)
{
    open3d::io::ReadPointCloud(cloud_source_filename, this->source_);
    open3d::io::ReadPointCloud(cloud_target_filename, this->target_);

    source_.PaintUniformColor(Eigen::Vector3d(0.5,0,0.5));

  // TO COMPLETE
}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target)
{
    this->source_ = cloud_source;
    this->target_ = cloud_target;
  // TO COMPLETE
}


void Registration::draw_registration_result()
{
  //visualize target and source with two different colors
    auto pc0_pointer = std::make_shared<open3d::geometry::PointCloud>(this->source_);
    auto pc1_pointer = std::make_shared<open3d::geometry::PointCloud>(this->target_);
    open3d::visualization::DrawGeometries({pc0_pointer, pc1_pointer});
  // TO COMPLETE
}


void Registration::preprocess(open3d::geometry::PointCloud pcd, double voxel_size, std::shared_ptr<open3d::geometry::PointCloud> &pcd_down_ptr, std::shared_ptr<open3d::pipelines::registration::Feature> &pcd_fpfh)
{
  //downsample, estimate normals and compute FPFH features

    // downsample
    pcd_down_ptr = pcd.VoxelDownSample(voxel_size);
    //Estimate point cloud normals
    pcd_down_ptr -> EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxel_size*2, 30));

    double radius_feature = 200;
    pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_down_ptr, open3d::geometry::KDTreeSearchParamHybrid(radius_feature,100));

  // TO COMPLETE
  return;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_global_registration(double voxel_size)
{
  // remember to apply the transformation_ matrix to source_cloud
  // create two point cloud to contain the downsampled point cloud and two structure to contain the features
  // call the Registration::preprocess function on target and transformed source
  // execute global transformation and update the transformation matrix
    std::shared_ptr<open3d::geometry::PointCloud> source_down_ptr;
    std::shared_ptr<open3d::geometry::PointCloud> target_down_ptr;

    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh;
    std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh;

    preprocess(source_, voxel_size, source_down_ptr, source_fpfh);
    preprocess(target_, voxel_size, target_down_ptr, target_fpfh);

  // TO COMPLETE
  open3d::pipelines::registration::RegistrationResult result;

  result = RegistrationRANSACBasedOnFeatureMatching (
       *(source_down_ptr.get()),
       *(target_down_ptr.get()),
       *(source_fpfh.get()),
       *(target_fpfh.get()),
       false,
       voxel_size*3
  );
  this->set_transformation(result.transformation_);
//  this->source_.Transform(this->get_transformation());

  return result;
}

open3d::pipelines::registration::RegistrationResult Registration::execute_icp_registration(double threshold, double relative_fitness, double relative_rmse, int max_iteration)
{

    Eigen::Matrix4d tr = Eigen::Matrix4d::Identity();
    auto result = open3d::pipelines::registration::RegistrationICP(
            source_,
            target_,
            threshold,
//            tr,
            this->get_transformation(),
            open3d::pipelines::registration::TransformationEstimationPointToPoint(),
            open3d::pipelines::registration::ICPConvergenceCriteria(
                    relative_fitness,
                    relative_rmse,
                    max_iteration
            )
    );
    this->set_transformation(result.transformation_);
    this->source_.Transform(this->get_transformation());

  return result;
}


void Registration::set_transformation(Eigen::Matrix4d init_transformation)
{
  transformation_=init_transformation;
}


Eigen::Matrix4d  Registration::get_transformation()
{
  return transformation_;
}


void Registration::write_tranformation_matrix(std::string filename)
{
  std::ofstream outfile (filename);
  if (outfile.is_open())
  {
    outfile << transformation_;
    outfile.close();
  }
}

void Registration::save_merged_cloud(std::string filename)
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  source_clone.Transform(transformation_);
  open3d::geometry::PointCloud merged = target_clone+source_clone;
  open3d::io::WritePointCloud(filename, merged );
}

