//
// Created by anastasia on 01.08.16.
//

#ifndef TANGOSCENERECONSTRUCTOR_TANGO_MESH_RECONSTRUCTOR_H
#define TANGOSCENERECONSTRUCTOR_TANGO_MESH_RECONSTRUCTOR_H

#include <mutex>

#include <tango_3d_reconstruction_api.h>
#include <tango-scene-reconstructor/pose_data.h>

namespace tango_scene_reconstructor {

  class TangoMeshReconstructor {
    public:
      TangoMeshReconstructor(float resolution, float min_depth, float max_depth);
      ~TangoMeshReconstructor();
      void SetColorCamera3DRIntrinsics();
      void Update(TangoXYZij* xyz_ij, TangoImageBuffer* image_buffer);
      void Render(glm::mat4 projection_mat,
                  glm::mat4 view_mat,
                  glm::mat4 model_mat);
      std::vector<float> GetXYZ();
      std::vector<uint8_t> GetRGB();
      std::vector<unsigned int> GetIndices();
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

      GLuint shader_program_;
      GLuint vertex_buffer_;
      GLuint vertices_handle_;
      GLuint mvp_handle_;
      GLuint color_handle_;
      GLuint color_buffer_;
      GLuint indices_handle_;
      GLuint indices_buffer_;
      std::mutex render_mutex_;
  };

} // namespace tango_scene_reconstructor

#endif //TANGOSCENERECONSTRUCTOR_TANGO_MESH_RECONSTRUCTOR_H
