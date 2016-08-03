#include "tango-scene-reconstructor/io/VTK.h"

namespace tango_scene_reconstructor {

  VTK::VTK(int optimization_methods) {
    optimization_methods_ = optimization_methods;
  }

  VTK::~VTK() {

  }

  void VTK::GenerateMeshesAndSave(PointCloudManager* point_cloud_manager, std::string folder_name) {
    switch (optimization_methods_) {
      case 0:
        point_cloud_manager->point_cloud_reconstructor_->GenerateAndSaveMesh(folder_name+"/RAW_mesh.vtk");
        point_cloud_manager->point_cloud_reconstructor_ftfsm_downsampled_->GenerateAndSaveMesh(folder_name+"/FTFSM_mesh.vtk");
        break;
      case 1:
        point_cloud_manager->point_cloud_reconstructor_->GenerateAndSaveMesh(folder_name+"/RAW_mesh.vtk");
        point_cloud_manager->point_cloud_reconstructor_mfsm_downsampled_->GenerateAndSaveMesh(folder_name+"/MFSM_mesh.vtk");
        break;
      case 2:
        point_cloud_manager->point_cloud_reconstructor_->GenerateAndSaveMesh(folder_name+"/RAW_mesh.vtk");
        point_cloud_manager->point_cloud_reconstructor_ftfsm_downsampled_->GenerateAndSaveMesh(folder_name+"/FTFSM_mesh.vtk");
        point_cloud_manager->point_cloud_reconstructor_mfsm_downsampled_->GenerateAndSaveMesh(folder_name+"/MFSM_mesh.vtk");
        break;
      default:
        break;
    }
  }
}