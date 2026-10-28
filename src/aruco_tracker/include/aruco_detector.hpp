#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace aruco_tracker {

class ArucoDetector {
public:
  struct CameraCalibration {
    cv::Matx33d camera_matrix{};
    std::vector<double> distortion_coefficients{};
  };

  struct MarkerObservation {
    int marker_id{};

    // Marker origin expressed in the camera coordinate frame.
    cv::Vec3d translation{};

    // Rotation of the marker frame relative to the camera frame.
    cv::Matx33d rotation{};

    // Detected marker corners in image pixel coordinates.
    std::array<cv::Point2f, 4> image_corners{};
  };

  ArucoDetector(const int target_marker_id, const double marker_size_m,
                const cv::aruco::PredefinedDictionaryType dictionary_type)
      : target_marker_id_{target_marker_id}, marker_size_m_{marker_size_m},
        dictionary_{cv::aruco::getPredefinedDictionary(dictionary_type)},
        detector_{dictionary_, make_detector_parameters()},
        marker_object_points_{make_marker_object_points(marker_size_m)} {
    if (target_marker_id < 0) {
      throw std::invalid_argument("target_marker_id must be >= 0");
    }

    if (marker_size_m <= 0.0) {
      throw std::invalid_argument("marker_size_m must be > 0");
    }
  }

  [[nodiscard]]
  std::optional<MarkerObservation>
  detect(const cv::Mat &image, const CameraCalibration &calibration) const {
    if (image.empty()) {
      return std::nullopt;
    }

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    detector_.detectMarkers(image, marker_corners, marker_ids);

    const auto marker_it =
        std::find(marker_ids.begin(), marker_ids.end(), target_marker_id_);

    if (marker_it == marker_ids.end()) {
      return std::nullopt;
    }

    const auto marker_index =
        static_cast<std::size_t>(std::distance(marker_ids.begin(), marker_it));

    const auto &image_points = marker_corners.at(marker_index);

    if (image_points.size() != 4) {
      return std::nullopt;
    }

    cv::Vec3d rotation_vector;
    cv::Vec3d translation_vector;

    const bool pose_found = cv::solvePnP(
        marker_object_points_, image_points, calibration.camera_matrix,
        calibration.distortion_coefficients, rotation_vector,
        translation_vector, false, cv::SOLVEPNP_IPPE_SQUARE);

    if (!pose_found) {
      return std::nullopt;
    }

    cv::Mat rotation_matrix;

    cv::Rodrigues(rotation_vector, rotation_matrix);

    MarkerObservation observation;

    observation.marker_id = target_marker_id_;

    observation.translation = translation_vector;

    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        observation.rotation(row, column) =
            rotation_matrix.at<double>(row, column);
      }
    }

    std::copy_n(image_points.begin(), 4, observation.image_corners.begin());

    return observation;
  }

private:
  [[nodiscard]]
  static cv::aruco::DetectorParameters make_detector_parameters() {
    cv::aruco::DetectorParameters parameters;

    // The marker corners directly influence the PnP pose.
    // Subpixel refinement reduces pose jitter.
    parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    return parameters;
  }

  [[nodiscard]]
  static std::vector<cv::Point3d>
  make_marker_object_points(const double marker_size_m) {
    const double half_size = marker_size_m / 2.0;

    //
    // Required ordering for SOLVEPNP_IPPE_SQUARE:
    //
    // 0: [-L/2, +L/2, 0]
    // 1: [+L/2, +L/2, 0]
    // 2: [+L/2, -L/2, 0]
    // 3: [-L/2, -L/2, 0]
    //
    return {
        {-half_size, half_size, 0.0},
        {half_size, half_size, 0.0},
        {half_size, -half_size, 0.0},
        {-half_size, -half_size, 0.0},
    };
  }

  int target_marker_id_;
  double marker_size_m_;

  cv::aruco::Dictionary dictionary_;
  cv::aruco::ArucoDetector detector_;

  std::vector<cv::Point3d> marker_object_points_;
};

} // namespace aruco_tracker
