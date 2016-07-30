#include "tango-scene-reconstructor/io/PCD.h"

namespace tango_scene_reconstructor {

  PCD::PCD(int optimization_methods) {
    optimization_methods_ = optimization_methods;
  }

  PCD::~PCD() {

  }

  void PCD::SavePointCloudContainer(PointCloudManager* point_cloud_manager, std::string folder_name) {

    CreateSubFolders(folder_name);

    int lastIndex = point_cloud_manager->GetPCDContainerLastIndex();

    switch (optimization_methods_) {
      case 0:
        for (int i = 0; i <= lastIndex; i++) {
          std::string dir_path = folder_name + "PCD/RAW/";
          char filename_raw[1024];
          sprintf(filename_raw, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCD(filename_raw);

          dir_path = folder_name + "PCD/FTFSM/";
          char filename_ftfsm[1024];
          sprintf(filename_ftfsm, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCDWithFTFSMPose(filename_ftfsm);
        }
        break;

      case 1:
        for (int i = 0; i <= lastIndex; i++) {
          std::string dir_path = folder_name + "PCD/RAW/";
          char filename_raw[1024];
          sprintf(filename_raw, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCD(filename_raw);

          dir_path = folder_name + "PCD/MFSM/";
          char filename_mfsm[1024];
          sprintf(filename_mfsm, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCDWithMFSMPose(filename_mfsm);
        }
        break;

      case 2:
        for (int i = 0; i <= lastIndex; i++) {
          std::string dir_path = folder_name + "PCD/RAW/";
          char filename_raw[1024];
          sprintf(filename_raw, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCD(filename_raw);

          dir_path = folder_name + "PCD/FTFSM/";
          char filename_ftfsm[1024];
          sprintf(filename_ftfsm, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCDWithFTFSMPose(filename_ftfsm);

          dir_path = folder_name + "PCD/MFSM/";
          char filename_mfsm[1024];
          sprintf(filename_mfsm, "%s/%05d.pcd", dir_path.c_str(), i);
          point_cloud_manager->point_cloud_container_[i]->SaveAsPCDWithMFSMPose(filename_mfsm);
        }
        break;
      default:
        break;
    }
  }

  void PCD::CreateSubFolders(std::string folder_name) {
    std::string dir_name = folder_name + "PCD/FTFSM";
    boost::filesystem::path dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);

    dir_name = folder_name + "PCD/MFSM";
    dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);

    dir_name = folder_name + "PCD/RAW";
    dir = dir_name.c_str();
    boost::filesystem::create_directories(dir);
  }
}