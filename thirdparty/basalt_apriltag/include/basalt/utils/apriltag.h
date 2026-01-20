
#include <basalt/image/image.h> //basalt-headers
#include <basalt/utils/sophus_utils.hpp>
#include <vector>

namespace basalt {

struct ApriltagDetectorData;

class ApriltagDetector {
 public:
  ApriltagDetector(int numTags);

  ~ApriltagDetector();

  void detectTags(basalt::ManagedImage<uint8_t>& img_raw,
                  Eigen::aligned_vector<Eigen::Vector2d>& corners,
                  std::vector<int>& ids, std::vector<double>& radii,
                  Eigen::aligned_vector<Eigen::Vector2d>& corners_rejected,
                  std::vector<int>& ids_rejected,
                  std::vector<double>& radii_rejected);

 private:
  ApriltagDetectorData* data;
};

}  // namespace basalt
