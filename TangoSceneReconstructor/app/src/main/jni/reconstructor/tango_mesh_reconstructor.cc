#include "tango-scene-reconstructor/reconstructor/tango_mesh_reconstructor.h"

namespace tango_scene_reconstructor {

  TangoMeshReconstructor::TangoMeshReconstructor(float resolution, float min_depth, float max_depth) {

    // Set Tango3DR_ConfigH and Tango3DR_Context for 3D reconstruction
    Tango3DR_ConfigH config = Tango3DR_Config_create(TANGO_3DR_CONFIG_CONTEXT);
    Tango3DR_Config_setDouble(config, "resolution", resolution);
    Tango3DR_Config_setDouble(config, "min_depth", min_depth);
    Tango3DR_Config_setDouble(config, "max_depth", max_depth);
    Tango3DR_Config_setBool(config, "generate_color", true);
    Tango3DR_Config_setBool(config, "use_space_clearing", true);

    context_ = Tango3DR_create(config);
    Tango3DR_Config_destroy(config);

    gridindexarray_ = new Tango3DR_GridIndexArray();

    pose_data_ = PoseData::GetInstance();
  }

  TangoMeshReconstructor::~TangoMeshReconstructor(){}

  void TangoMeshReconstructor::SetColorCamera3DRIntrinsics() {

    TangoCameraIntrinsics color_camera_intrinsics = pose_data_->GetColorCameraIntrinsics();

    // Set Tango3DR_CameraCalibration
    color_camera_.height = color_camera_intrinsics.height;
    color_camera_.width = color_camera_intrinsics.width;

    color_camera_.distortion[0] = color_camera_intrinsics.distortion[0];
    color_camera_.distortion[1] = color_camera_intrinsics.distortion[1];
    color_camera_.distortion[2] = color_camera_intrinsics.distortion[2];
    color_camera_.distortion[3] = color_camera_intrinsics.distortion[3];
    color_camera_.distortion[4] = color_camera_intrinsics.distortion[4];

    color_camera_.fx = color_camera_intrinsics.fx;
    color_camera_.fy = color_camera_intrinsics.fy;
    color_camera_.cx = color_camera_intrinsics.cx;
    color_camera_.cy = color_camera_intrinsics.cy;

    color_camera_.calibration_type = (Tango3DR_TangoCalibrationType)color_camera_intrinsics.calibration_type;
  }

  void TangoMeshReconstructor::Update(TangoXYZij* xyz_ij, TangoImageBuffer* image_buffer) {
    Tango3DR_ImageBuffer image_buffer_3dr = ConvertImageBufferToImage3DRBuffer(image_buffer);
    Tango3DR_PointCloud xyz_ij_3dr = ConvertPointCloudToPointCloud3DR(xyz_ij);
    Tango3DR_Pose image_buffer_pose = ConvertPoseMatrixToPose3DR(pose_data_->GetSSTColorCamera(image_buffer_3dr.timestamp));
    Tango3DR_Pose xyz_ij_pose = ConvertPoseMatrixToPose3DR(pose_data_->GetSSTDepthCamera(xyz_ij_3dr.timestamp));

    mesh_ = new Tango3DR_Mesh();
    Tango3DR_Status status = Tango3DR_update(context_, &xyz_ij_3dr, &xyz_ij_pose,
                                             &image_buffer_3dr, &image_buffer_pose, &color_camera_,
                                             &gridindexarray_);

    if (status == TANGO_3DR_ERROR) {
      LOGE("UPDATE STATUS some sort of hard error occurred");
    }
    if (status == TANGO_3DR_INSUFFICIENT_SPACE) {
      LOGE("UPDATE STATUS not enough space in a provided buffer");
    }
    if (status == TANGO_3DR_INVALID) {
      LOGE("UPDATE STATUS input argument is invalid");
    }
    if (status == TANGO_3DR_SUCCESS) {
      status = Tango3DR_extractFullMesh(context_, &mesh_);

      if (status == TANGO_3DR_ERROR) {
          LOGE("EXTRACT STATUS some sort of hard error occurred");
      }

      if (status == TANGO_3DR_INSUFFICIENT_SPACE) {
          LOGE("EXTRACT STATUS not enough space in a provided buffer");
      }

      if (status == TANGO_3DR_INVALID) {
          LOGE("EXTRACT STATUS input argument is invalid");
      }

      if (status == TANGO_3DR_SUCCESS) {
        std::vector<float> vertices_tmp;
        std::vector<unsigned int> indices_tmp;
        std::vector<uint8_t> colors_tmp;

        if (mesh_->num_faces > 0) {
            for (int i=0; i<mesh_->num_vertices; i++) {
                vertices_tmp.push_back(mesh_->vertices[i][0]);
                vertices_tmp.push_back(mesh_->vertices[i][1]);
                vertices_tmp.push_back(mesh_->vertices[i][2]);
                colors_tmp.push_back(mesh_->colors[i][0]);
                colors_tmp.push_back(mesh_->colors[i][1]);
                colors_tmp.push_back(mesh_->colors[i][2]);
            }

            for (int i=0; i<mesh_->num_faces; i++) {
              indices_tmp.push_back((unsigned int)mesh_->faces[i][0]);
              indices_tmp.push_back((unsigned int)mesh_->faces[i][1]);
              indices_tmp.push_back((unsigned int)mesh_->faces[i][2]);
            }

            Tango3DR_Mesh_destroy(mesh_);

            {
              std::lock_guard <std::mutex> lock(render_mutex_);
              vertices_ = vertices_tmp;
              colors_ = colors_tmp;
              indices_ = indices_tmp;
            }
        }
      }
    }
  }

  Tango3DR_PointCloud TangoMeshReconstructor::ConvertPointCloudToPointCloud3DR(TangoXYZij* xyz_ij) {
    Tango3DR_PointCloud xyz_ij_3dr;
    xyz_ij_3dr.num_points = xyz_ij->xyz_count;
    xyz_ij_3dr.points = new Tango3DR_Vector4[xyz_ij_3dr.num_points];
    for (int i = 0; i < xyz_ij_3dr.num_points; ++i) {
      xyz_ij_3dr.points[i][0] = xyz_ij->xyz[i][0];
      xyz_ij_3dr.points[i][1] = xyz_ij->xyz[i][1];
      xyz_ij_3dr.points[i][2] = xyz_ij->xyz[i][2];
      // last is confidence
      xyz_ij_3dr.points[i][3] = 1;
    }
    xyz_ij_3dr.timestamp = xyz_ij->timestamp;
    return xyz_ij_3dr;
  }

  Tango3DR_ImageBuffer TangoMeshReconstructor::ConvertImageBufferToImage3DRBuffer(TangoImageBuffer* image_buffer) {
    Tango3DR_ImageBuffer image_buffer_3dr;
    image_buffer_3dr.width = image_buffer->width;
    image_buffer_3dr.height = image_buffer->height;
    image_buffer_3dr.stride = 1280;
    image_buffer_3dr.timestamp = image_buffer->timestamp;
    image_buffer_3dr.format = (Tango3DR_ImageFormatType)image_buffer->format;
    image_buffer_3dr.data = image_buffer->data;
    return image_buffer_3dr;
  }

  Tango3DR_Pose TangoMeshReconstructor::ConvertPoseMatrixToPose3DR(glm::mat4 pose) {
    Tango3DR_Pose pose_3dr;
    glm::vec3 translation;
    glm::quat rotation;
    glm::vec3 scale;

    tango_gl::util::DecomposeMatrix(pose, translation, rotation, scale);

    pose_3dr.orientation[0] = rotation.x;
    pose_3dr.orientation[1] = rotation.y;
    pose_3dr.orientation[2] = rotation.z;
    pose_3dr.orientation[3] = rotation.w;

    pose_3dr.translation[0] = translation.x;
    pose_3dr.translation[1] = translation.y;
    pose_3dr.translation[2] = translation.z;

    return pose_3dr;
  }

  std::vector<float> TangoMeshReconstructor::GetXYZ() {
    return vertices_;
  }

  std::vector<uint8_t> TangoMeshReconstructor::GetRGB() {
    return colors_;
  }

  std::vector<unsigned int> TangoMeshReconstructor::GetIndices() {
    return indices_;
  }

  void TangoMeshReconstructor::GenerateAndSaveMesh(std::vector<PointCloud*> point_cloud_container, std::string folder_name) {
    LOGE("Function TangoMeshReconstructor::GenerateAndSaveMesh is not implemented yet");

    /*std::vector<float> vertices_tmp;
    std::vector<unsigned int> indices_tmp;
    std::vector<uint8_t> colors_tmp;
    for (int i=0; i < point_cloud_container.size(); i++) {
      Tango3DR_ImageBuffer image_buffer_3dr = ConvertImageBufferToImage3DRBuffer(point_cloud_container[i]->GetYUV());
      Tango3DR_PointCloud xyz_ij_3dr = ConvertPointCloudToPointCloud3DR(point_cloud_container[i]->GetXYZ());
      Tango3DR_Pose image_buffer_pose = ConvertPoseMatrixToPose3DR(point_cloud_container[i]->yuv_pose_);
      Tango3DR_Pose xyz_ij_pose = ConvertPoseMatrixToPose3DR(point_cloud_container[i]->xyz_pose_);

      mesh_ = new Tango3DR_Mesh();
      Tango3DR_Status status = Tango3DR_update(context_, &xyz_ij_3dr, &xyz_ij_pose,
                                               &image_buffer_3dr, &image_buffer_pose, &color_camera_,
                                               &gridindexarray_);

      if (status == TANGO_3DR_ERROR) {
        LOGE("UPDATE STATUS some sort of hard error occurred");
      }
      if (status == TANGO_3DR_INSUFFICIENT_SPACE) {
        LOGE("UPDATE STATUS not enough space in a provided buffer");
      }
      if (status == TANGO_3DR_INVALID) {
        LOGE("UPDATE STATUS input argument is invalid");
      }
      if (status == TANGO_3DR_SUCCESS) {
        status = Tango3DR_extractFullMesh(context_, &mesh_);

        if (status == TANGO_3DR_ERROR) {
            LOGE("EXTRACT STATUS some sort of hard error occurred");
        }

        if (status == TANGO_3DR_INSUFFICIENT_SPACE) {
            LOGE("EXTRACT STATUS not enough space in a provided buffer");
        }

        if (status == TANGO_3DR_INVALID) {
            LOGE("EXTRACT STATUS input argument is invalid");
        }

        if (status == TANGO_3DR_SUCCESS) {

          if (mesh_->num_faces > 0) {
            for (int i=0; i<mesh_->num_vertices; i++) {
                vertices_tmp.push_back(mesh_->vertices[i][0]);
                vertices_tmp.push_back(mesh_->vertices[i][1]);
                vertices_tmp.push_back(mesh_->vertices[i][2]);
                colors_tmp.push_back(mesh_->colors[i][0]);
                colors_tmp.push_back(mesh_->colors[i][1]);
                colors_tmp.push_back(mesh_->colors[i][2]);
            }

            for (int i=0; i<mesh_->num_faces; i++) {
              indices_tmp.push_back((unsigned int)mesh_->faces[i][0]);
              indices_tmp.push_back((unsigned int)mesh_->faces[i][1]);
              indices_tmp.push_back((unsigned int)mesh_->faces[i][2]);
            }
          }
        }
      }
    }

    Tango3DR_Mesh_destroy(mesh_);

    vertices_ = vertices_tmp;
    colors_ = colors_tmp;
    indices_ = indices_tmp;

    pcl::PolygonMesh triangles;
    triangles.polygons.resize(indices_.size()/3);
    int k = 0;
    for (int i = 0; i < triangles.polygons.size(); i++) {
      pcl::Vertices vertices;
      vertices.vertices.push_back(vertices_[indices_[k]]);
      vertices.vertices.push_back(vertices_[indices_[k+1]]);
      vertices.vertices.push_back(vertices_[indices_[k+2]]);
      k+=3;
      triangles.polygons[i] = vertices;
    }*/

  }

}