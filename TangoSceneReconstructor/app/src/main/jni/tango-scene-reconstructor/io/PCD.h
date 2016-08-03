#include <string>
#include <boost/filesystem.hpp>
#include <tango-scene-reconstructor/point_cloud_manager.h>

#ifndef RGBPOINTCLOUDBUILDER_PCD_H
#define RGBPOINTCLOUDBUILDER_PCD_H


namespace tango_scene_reconstructor {

  class PCD {
    public:
      PCD(int optimization_methods);
      ~PCD();
      void SavePointCloudContainer(PointCloudManager* point_cloud_manager, std::string folder_name);
      void SaveMergedPointClouds(PointCloudManager* point_cloud_manager, std::string folder_name);
    private:
      void CreateSubFolders(std::string folder_name);
      int optimization_methods_;
  };

} // namespace tango_scene_reconstructor

#endif //RGBPOINTCLOUDBUILDER_PCD_H
