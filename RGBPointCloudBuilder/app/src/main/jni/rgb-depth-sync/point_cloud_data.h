/*
 * Class writes rgb point cloud either to an file or
 * sends it via TCP socket.
 */

#ifndef RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
#define RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H

#include "tango-gl/util.h"
#include "rgb-depth-sync/util.h"

namespace rgb_depth_sync {

  class PointCloudData {
    public:
      explicit PointCloudData();
      ~PointCloudData();
      void SaveOrderedPointCloudToFile(const std::vector<float> point_cloud,
                                       const glm::vec3 translation,
                                       const glm::quat rotation,
                                       int width, int height,
                                       const double timestamp);
      void setOrdered(int width, int height);
      void setUnordered();
      void saveToFile();
      void saveToSocket(std::string addr, int port);
      std::vector<float> getPCDData();
      void setPCDData(const std::vector<float> point_cloud,
                      const glm::vec3 translation,
                      const glm::quat rotation,
                      const double timestamp);
      std::string timestampToString(double value);
      std::string getHeader();
      void setHeader(int width, int height);
    private:
      std::string header_;
      std::string timestamp_;
      std::vector <float> pcd_;
      glm::vec3 translation_;
      glm::quat rotation_;
  };

} // namespace rgb_depth_sync

#endif //RGB_DEPTH_SYNC_POINT_CLOUD_DATA_H
