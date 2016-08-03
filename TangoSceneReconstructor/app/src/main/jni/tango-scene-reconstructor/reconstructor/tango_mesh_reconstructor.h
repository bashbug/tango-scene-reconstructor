#ifndef TANGOSCENERECONSTRUCTOR_TANGO_MESH_RECONSTRUCTOR_H
#define TANGOSCENERECONSTRUCTOR_TANGO_MESH_RECONSTRUCTOR_H

#include <mutex>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <tango_3d_reconstruction_api.h>
#include "tango-scene-reconstructor/pose_data.h"
#include "tango-scene-reconstructor/point_cloud.h"

namespace tango_scene_reconstructor {

  class TangoMeshReconstructor {
    public:
      TangoMeshReconstructor(float resolution, float min_depth, float max_depth);
      ~TangoMeshReconstructor();
      void SetColorCamera3DRIntrinsics();
      void Update(TangoXYZij* xyz_ij, TangoImageBuffer* image_buffer);
      std::vector<float> GetXYZ();
      std::vector<uint8_t> GetRGB();
      std::vector<unsigned int> GetIndices();
      void GenerateAndSaveMesh(std::vector<PointCloud*> point_cloud_container, std::string folder_name);
    private:
      Tango3DR_Pose ConvertPoseMatrixToPose3DR(glm::mat4 pose);
      Tango3DR_PointCloud ConvertPointCloudToPointCloud3DR(TangoXYZij* xyz_ij);
      Tango3DR_ImageBuffer ConvertImageBufferToImage3DRBuffer(TangoImageBuffer* image_buffer);
      Tango3DR_Context context_;
      Tango3DR_CameraCalibration color_camera_;
      Tango3DR_GridIndexArray* gridindexarray_;
      Tango3DR_Mesh* mesh_;
      PoseData* pose_data_;

      std::vector<float> vertices_;
      std::vector<uint8_t> colors_;
      std::vector<unsigned int> indices_;

      std::mutex render_mutex_;
  };

} // namespace tango_scene_reconstructor

#endif //TANGOSCENERECONSTRUCTOR_TANGO_MESH_RECONSTRUCTOR_H
