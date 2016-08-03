#include <string>
#include <tango-scene-reconstructor/point_cloud_manager.h>

#ifndef TANGOSCENERECONSTRUCTOR_VTK_H
#define TANGOSCENERECONSTRUCTOR_VTK_H

namespace tango_scene_reconstructor {

  class VTK {
    public:
      VTK(int optimization_methods);
      ~VTK();
      void GenerateMeshesAndSave(PointCloudManager* point_cloud_manager, std::string folder_name);
    private:
      int optimization_methods_;
  };

}// namespace tango_scene_reconstructor

#endif //TANGOSCENERECONSTRUCTOR_VTK_H
