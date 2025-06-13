// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
#define AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/Forward.h>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>

#include <string>
#include <vector>

namespace autoware::experimental::marker_utils
{
// TODO: Apply these and refactor all types 
// using visualization_msgs::msg::Marker;
// using MarkerArray = visualization_msgs::msg::MarkerArray;
// using std_msgs::msg::ColorRGBA;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_position;
using autoware_utils::create_marker_scale;

/** 
 * @brief create centroid point of lanelet's BasicPolygon3d
 * @param [in] poly lanelet BasicPolygon3d
 * @return BasicPoint3d which is centroid of polygon
 */
lanelet::BasicPoint3d get_centroid_point(const lanelet::BasicPolygon3d & poly);

/**
 * @brief convert point into ROS2 message
 * @param [in] point lanelet BasicPoint3d (from get_centroid_point)
 * @return geometry type Point (ROS2 message)
 */
geometry_msgs::msg::Point to_msg(const lanelet::BasicPoint3d & point);

/**
 * @brief create lanelet info (correspond) marker array
 * @param [in] reg_elem regulatory element (in this case, only DetectionArea and NoStoppingArea)
 * @param [in] now current time
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] marker_lifetime lifetime of the marker
 * @return marker arrat of the correspond lanelet info
 */

template <typename T>
struct always_false : std::false_type {};

template <typename RegElemT>
visualization_msgs::msg::MarkerArray create_lanelet_info_marker_array(
  const RegElemT & reg_elem, const rclcpp::Time & now, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color, const double marker_lifetime)
{
  lanelet::ConstPolygons3d reg_elem_areas;
  auto polygon_to_stop_line_marker_type = visualization_msgs::msg::Marker::LINE_LIST;
  std::string ns_prefix;
  boost::optional<lanelet::ConstLineString3d> stop_line;

  // Check type of Regulatory Element
  if constexpr (std::is_same_v<RegElemT, lanelet::autoware::DetectionArea>) {
    reg_elem_areas = reg_elem.detectionAreas();
    polygon_to_stop_line_marker_type = visualization_msgs::msg::Marker::LINE_LIST;
    ns_prefix = "detection_area";
    stop_line = reg_elem.stopLine();
  } else if constexpr (std::is_same_v<RegElemT, lanelet::autoware::NoStoppingArea>) {
    reg_elem_areas = reg_elem.noStoppingAreas();
    polygon_to_stop_line_marker_type = visualization_msgs::msg::Marker::LINE_STRIP;
    ns_prefix = "no_stopping_area";
    stop_line = reg_elem.stopLine();
  } else {
    throw std::runtime_error("Unsupported regulatory element type");
  }

  visualization_msgs::msg::MarkerArray msg;

  // ID
  {
    auto marker = create_default_marker(
      "map", now, ns_prefix + "_id", static_cast<int32_t>(reg_elem.id()), visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0, 1.0, 1.0, 0.999));
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (const auto & area : reg_elem_areas) {
      const auto poly = area.basicPolygon();

      marker.pose.position = to_msg(poly.front());
      marker.pose.position.z += 2.0;
      marker.text = std::to_string(reg_elem.id());

      msg.markers.push_back(marker);
    }
  }
  // Polygon
  {
    auto marker = create_default_marker(
      "map", now, ns_prefix + "_polygon", static_cast<int32_t>(reg_elem.id()), visualization_msgs::msg::Marker::LINE_LIST,
      scale, color);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (const auto & area : reg_elem_areas) {
      const auto poly = area.basicPolygon();

      for (size_t i = 0; i < poly.size(); ++i) {
        const auto idx_front = i;
        const auto idx_back = (i == poly.size() - 1) ? 0 : i + 1;

        const auto & p_front = poly.at(idx_front);
        const auto & p_back = poly.at(idx_back);

        marker.points.push_back(to_msg(p_front));
        marker.points.push_back(to_msg(p_back));
      }

      msg.markers.push_back(marker);
    }
  }

  // Polygon to Stop Line 
  // Set stop_line to be all optional (in detection_area, it's not optional.)
  if (stop_line) {
    const auto stop_line_center_point =
      (stop_line->front().basicPoint() + stop_line->back().basicPoint()) / 2;

    auto marker = create_default_marker(
      "map", now, ns_prefix + "_correspondence", static_cast<int32_t>(reg_elem.id()),
      polygon_to_stop_line_marker_type, scale, color);

    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime);

    for (auto & area : reg_elem_areas) {
      const auto poly = area.basicPolygon();
      const auto centroid_point = get_centroid_point(poly);
      for (size_t i = 0; i < poly.size(); ++i) {
        marker.points.push_back(to_msg(centroid_point));
        marker.points.push_back(to_msg(stop_line_center_point));
      }
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

/**
 * @brief create marker array from geometry polygon based on the marker type
 * @details if marker_type is LINE_LIST, the polygon is drawn as a line list
 *          if marker_type is LINE_STRIP, the polygon is drawn as a line strip
 * @param [in] polygon geometry polygon
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the geometry polygon
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 & scale,
  const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create footprint from LinearRing2d (used mainly for goal_footprint)
 * @param [in] ring LinearRing2d  to convert into marker
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @return marker array of the boost LinearRing2d
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::LinearRing2d & ring, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from boost MultiPolygon2d (Pull over area)
 * @param [in] area_polygon boost MultiPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & area_polygons, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  double z = 0.0);

/**
 * @brief return marker array from centroid of MultiPolygon2d and trajectory point
 * @param [in] multi_polygon boost MultiPolygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils_geometry::MultiPolygon2d & multi_polygon, const size_t & trajectory_index,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from stop obstacle point
 * @param [in] stop_obstacle_point point of the stop obstacle
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the stop obstacle point
 */
visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Point & stop_obstacle_point, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker from Autoware Polygon2d
 * @param [in] polygon Autoware Polygon2d
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] marker_type type of the marker (LINE_LIST or LINE_STRIP)
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] z z position of the marker
 * @return marker of the Autoware Polygon2d
 */
visualization_msgs::msg::Marker create_autoware_geometry_marker(
  const autoware_utils_geometry::Polygon2d & polygon, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 & scale, const std_msgs::msg::ColorRGBA & color,
  double z = 0.0);

/**
 * @brief return marker array from lanelets
 * @details This function creates a marker array from lanelets, draw the lanelets either as triangle
 * marker or boundary as marker.
 * @param [in] lanelets lanelets to create markers from
 * @param [in] color color of the marker
 * @param [in] ns namespace of the marker
 * @return marker array of the lanelets
 */
visualization_msgs::msg::MarkerArray create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color, const geometry_msgs::msg::Vector3 scale,
  const double z = 0.0, const bool planning = false);

/**
 * @brief create marker array from predicted object
 * @details This function creates a marker array from a PredictedObjects object
 * (initial_pose_with_covariance), draw the vehicle based on initial pose.
 * @param [in] objects PredictedObjects object
 * @param [in] stamp current time
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const rclcpp::Time & stamp,
  const std::string & ns, const int32_t id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create predicted path marker array from PredictedPath
 * @param [in] predicted_path PredictedPath object
 * @param [in] vehicle_info vehicle information to calculate footprint
 * @param [in] ns namespace for the marker
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of the predicted path
 */
visualization_msgs::msg::MarkerArray create_predicted_path_marker_array(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t & id, const std_msgs::msg::ColorRGBA & color);

/**
 * @brief create marker array from predicted object (tier4 msg or internal planning msg)
 * @param [in] path PathWithLaneId object
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] now current time
 * @param [in] scale scale of the marker
 * @param [in] color color of the marker
 * @param [in] with_text if true, add text to the marker
 * @return marker array of the boost MultiPolygon2d (Pull over area)
 */
visualization_msgs::msg::MarkerArray create_path_with_lane_id_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int32_t id, const rclcpp::Time & now, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, const bool with_text);

/**
 * @brief create a vehicle trajectory point marker array object
 * @param [in] mpt_traj trajectory points to create markers from
 * @param [in] vehicle_info vehicle information to calculate footprint
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray create_vehicle_trajectory_point_marker_array(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t id);

/**
 * @brief create marker array from lanelet polygon (CompoundPolygon3d)
 * @param [in] polygon lanelet polygon
 * @param [in] stamp time stamp of the marker
 * @param [in] ns namespace
 * @param [in] id id of the marker
 * @param [in] color color of the marker
 * @return marker array of the lanelet polygon
 */
visualization_msgs::msg::MarkerArray create_lanelet_polygon_marker_array(
  const lanelet::CompoundPolygon3d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, const std_msgs::msg::ColorRGBA & color);

}  // namespace autoware::experimental::marker_utils

#endif  // AUTOWARE__MARKER_UTILS__MARKER_CONVERSION_HPP_
