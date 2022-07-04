#include <iostream>
#include <Eigen/Dense>
#include "Registration.h"

// ./registration data/one/source.ply data/one/target.ply data/one/transformation.txt data/one/merged.ply
// ./registration data/two/source.ply data/two/target.ply data/two/transformation.txt data/two/merged.ply


int main(int argc, char *argv[]) {
  //load source and target cloud
  Registration registration(argv[1], argv[2]);
  //show point cloud
  registration.draw_registration_result();
  //show initial transformation matrix
  std::cout<<registration.get_transformation()<<std::endl;
  //execute global registration
  registration.execute_global_registration();
  //show point cloud
  registration.draw_registration_result();
  //show transformation matrix after global registration
  std::cout<<registration.get_transformation()<<std::endl;

  //icp registration
  auto result = registration.execute_icp_registration(0.1, 1e-5, 1e-5, 1000);
  //show point cloud
  registration.draw_registration_result();
  //show transformation matrix after icp
  std::cout<<registration.get_transformation()<<std::endl;

  //save transformation matrix and registered cloud 
  registration.write_tranformation_matrix(argv[3]);
  registration.save_merged_cloud(argv[4]);
  std::cout<<"fitness: "<<result.fitness_<<std::endl;
  std::cout<<"inlier_rmse: "<<result.inlier_rmse_<<std::endl;



 
  return 0;
}
