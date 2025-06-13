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

#include <autoware/marker_utils/marker_conversion.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/centroid.hpp>
#include <boost/geometry/strategies/cartesian/centroid_bashein_detmer.hpp>

#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace autoware::experimental::marker_utils
{
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_color;
using autoware_utils::create_marker_position;
using autoware_utils::create_marker_scale;
// TODO: Apply these and refactor all types 
// using visualization_msgs::msg::Marker;
// using MarkerArray = visualization_msgs::msg::MarkerArray;
// using std_msgs::msg::ColorRGBA;

// ===================== Helper Function ==================================

lanelet::BasicPoint3d get_centroid_point(const lanelet::BasicPolygon3d & poly)
{
  lanelet::BasicPoint3d p_sum{0.0, 0.0, 0.0};
  for (const auto & p : poly) {
    p_sum += p;
  }
  return p_sum / poly.size();
}

geometry_msgs::msg::Point to_msg(const lanelet::BasicPoint3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}



// ====================== Main Function ===================================

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const geometry_msgs::msg::Polygon & polygon, const std::string & frame_id,
  const rclcpp::Time & stamp, const std::string & ns, int32_t id, uint32_t marker_type,
  const geometry_msgs::msg::Vector3 scale, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker(frame_id, stamp, ns, id, marker_type, scale, color);

  marker.lifetime = rclcpp::Duration::from_seconds(0.3);

  const auto & points_ = polygon.points;
  const size_t N = points_.size();

  if (marker_type == visualization_msgs::msg::Marker::LINE_LIST) {
    for (size_t i = 0; i < N; ++i) {
      const auto & cur = points_[i];
      const auto & nxt = points_[(i + 1) % N];
      geometry_msgs::msg::Point p1, p2;
      p1.x = cur.x;
      p1.y = cur.y;
      p1.z = cur.z;
      p2.x = nxt.x;
      p2.y = nxt.y;
      p2.z = nxt.z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else {
    for (const auto & p : points_) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
  }

  marker_array.markers.push_back(marker);
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const rclcpp::Time & stamp, const geometry_msgs::msg::Point & stop_obstacle_point)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto marker = create_default_marker(
    "map", stamp, "no_start_obstacle_text", 0, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0, 1.0, 1.0, 0.999));

  marker.pose.position = stop_obstacle_point;
  marker.pose.position.z += 2.0;

  marker.text = "!";
  marker_array.markers.push_back(marker);
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  autoware_utils::LinearRing2d ring)
{
  visualization_msgs::msg::MarkerArray msg;
  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), "goal_footprint", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.99, 0.99, 0.2, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  for (size_t i = 0; i < ring.size(); ++i) {
    geometry_msgs::msg::Point pt;
    pt.x = ring[i][0];
    pt.y = ring[i][1];
    pt.z = 0.0;
    marker.points.push_back(pt);
  }

  if (!marker.points.empty()) {
    marker.points.push_back(marker.points.front());
  }

  msg.markers.push_back(marker);

  return msg;
}

visualization_msgs::msg::MarkerArray create_autoware_geometry_marker_array(
  const autoware_utils::MultiPolygon2d & area_polygons, const rclcpp::Time & stamp,
  const std::string & ns, int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, double z)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < area_polygons.size(); ++i) {
    const auto marker = create_autoware_geometry_marker(
      area_polygons[i], stamp, ns, id, marker_type, scale, color, z);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void create_autoware_geometry_marker_array(
  const autoware_utils::MultiPolygon2d & multi_polygon,
  visualization_msgs::msg::MarkerArray & marker_array, const size_t & trajectory_index,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory)
{
  for (const auto & polygon : multi_polygon) {
    marker_array.markers.back().points.push_back(trajectory[trajectory_index].pose.position);
    const auto centroid = boost::geometry::return_centroid<autoware_utils::Point2d>(polygon);
    marker_array.markers.back().points.push_back(
      geometry_msgs::msg::Point().set__x(centroid.x()).set__y(centroid.y()));
  }
}

visualization_msgs::msg::Marker create_autoware_geometry_marker(
  const autoware_utils::Polygon2d & polygon, const rclcpp::Time & stamp, const std::string & ns,
  int32_t id, uint32_t marker_type, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, double z)
{
  visualization_msgs::msg::Marker marker =
    create_default_marker("map", stamp, ns, id, marker_type, scale, color);

  if (marker_type == visualization_msgs::msg::Marker::LINE_LIST) {
    for (size_t i = 0; i < polygon.outer().size(); ++i) {
      const auto & cur = polygon.outer().at(i);
      const auto & nxt = polygon.outer().at((i + 1) % polygon.outer().size());
      geometry_msgs::msg::Point p1, p2;
      p1.x = cur.x();
      p1.y = cur.y();
      p1.z = z;
      p2.x = nxt.x();
      p2.y = nxt.y();
      p2.z = z;
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
  } else {
    marker.pose.orientation = autoware_utils::create_marker_orientation(0, 0, 0, 1.0);
    for (const auto & p : polygon.outer()) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = z;
      marker.points.push_back(pt);
    }
  }
  return marker;
}

visualization_msgs::msg::MarkerArray create_predicted_objects_marker_array(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & ns,
  const int64_t id, const rclcpp::Time & now, const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray marker_array;

  auto marker = create_default_marker(
    "map", now, ns, 0, visualization_msgs::msg::Marker::CUBE, create_marker_scale(3.0, 1.0, 1.0),
    color);
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);

  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects.at(i);
    marker.id = static_cast<int>((id << (sizeof(int32_t) * 8 / 2)) + static_cast<int32_t>(i));
    marker.pose = object.kinematics.initial_pose_with_covariance.pose;
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void create_predicted_objects_marker_array(
  visualization_msgs::msg::MarkerArray & marker_array, visualization_msgs::msg::Marker base_marker,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const geometry_msgs::msg::Pose & ego_pose)
{
  base_marker.ns = "objects";
  base_marker.color = autoware_utils::create_marker_color(0.0, 1.0, 0.0, 1.0);
  base_marker.type = visualization_msgs::msg::Marker::LINE_LIST;

  lanelet::BasicPolygons2d object_polygons;
  constexpr double max_draw_distance = 50.0;
  for (const auto & obj : objects.objects) {
    for (const auto & path : obj.kinematics.predicted_paths) {
      for (const auto & pose : path.path) {
        if (autoware_utils::calc_distance2d(pose, ego_pose) >= max_draw_distance) {
          continue;
        }
        const auto outer = autoware_utils::to_polygon2d(pose, obj.shape).outer();
        for (size_t i = 0, n = outer.size(); i < n; ++i) {
          const auto & p1 = outer[i];
          const auto & p2 = outer[(i + 1) % n];
          base_marker.points.push_back(autoware_utils::create_marker_position(p1.x(), p1.y(), 0.0));
          base_marker.points.push_back(autoware_utils::create_marker_position(p2.x(), p2.y(), 0.0));
        }
      }
    }
  }
  marker_array.markers.push_back(base_marker);
}

// Temporary (TODO: function from "autoware/behavior_path_planner_common/utils/path_utils.hpp" is it
// ok to import?)
std::vector<double> calcPathArcLengthArray(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const size_t start = 0,
  const size_t end = std::numeric_limits<size_t>::max(), const double offset = 0.0)
{
  const auto bounded_start = std::max(start, size_t{0});
  const auto bounded_end = std::min(end, path.points.size());
  std::vector<double> out;
  out.reserve(bounded_end - bounded_start);

  double sum = offset;
  out.push_back(sum);

  for (size_t i = bounded_start + 1; i < bounded_end; ++i) {
    sum += autoware_utils::calc_distance2d(path.points.at(i).point, path.points.at(i - 1).point);
    out.push_back(sum);
  }
  return out;
}

visualization_msgs::msg::MarkerArray create_path_with_lane_id_marker_array(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path, const std::string & ns,
  const int64_t lane_id, const rclcpp::Time & now, const geometry_msgs::msg::Vector3 scale,
  const std_msgs::msg::ColorRGBA & color, const bool with_text)
{
  auto uid = lane_id << (sizeof(int32_t) * 8 / 2);
  int32_t idx = 0;
  int32_t i = 0;
  const auto arclength = calcPathArcLengthArray(path);
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker marker = create_default_marker(
    "map", now, ns, static_cast<int32_t>(uid), visualization_msgs::msg::Marker::ARROW, scale,
    color);

  for (const auto & p : path.points) {
    marker.id = uid + i++;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    marker.pose = p.point.pose;

    if (
      std::find(p.lane_ids.begin(), p.lane_ids.end(), lane_id) == p.lane_ids.end() && !with_text) {
      marker.color = create_marker_color(0.5, 0.5, 0.5, 0.999);
    }
    msg.markers.push_back(marker);
    if (i % 10 == 0 && with_text) {
      visualization_msgs::msg::Marker marker_text = create_default_marker(
        "map", now, ns, 0L, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        create_marker_scale(0.2, 0.1, 0.3), create_marker_color(1, 1, 1, 0.999));
      marker_text.id = uid + i++;
      std::stringstream ss;
      ss << std::fixed << std::setprecision(1) << "i=" << idx << "\ns=" << arclength.at(idx);
      marker_text.text = ss.str();
      msg.markers.push_back(marker_text);
    }
    ++idx;
  }
  return msg;
}

void create_vehicle_footprint_marker(
  visualization_msgs::msg::Marker & marker, const geometry_msgs::msg::Pose & pose,
  const double & base_to_right, const double & base_to_left, const double & base_to_front,
  const double & base_to_rear)
{
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, base_to_front, base_to_left, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, base_to_front, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, -base_to_rear, -base_to_right, 0.0).position);
  marker.points.push_back(
    autoware_utils::calc_offset_pose(pose, -base_to_rear, base_to_left, 0.0).position);
  marker.points.push_back(marker.points.front());
}

visualization_msgs::msg::MarkerArray create_vehicle_trajectory_point_marker_array(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & mpt_traj,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t sampling_num)
{
  auto marker = create_default_marker(
    "map", rclcpp::Clock().now(), "mpt_footprints", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.05, 0.0, 0.0), create_marker_color(0.99, 0.99, 0.2, 0.99));
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  const double base_to_right = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.right_overhang_m;
  const double base_to_left = (vehicle_info.wheel_tread_m / 2.0) + vehicle_info.left_overhang_m;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  visualization_msgs::msg::MarkerArray marker_array;
  for (size_t i = 0; i < mpt_traj.size(); ++i) {
    if (i % sampling_num != 0) {
      continue;
    }

    marker.id = i;
    marker.points.clear();

    const auto & traj_point = mpt_traj.at(i);
    create_vehicle_footprint_marker(
      marker, traj_point.pose, base_to_right, base_to_left, base_to_front, base_to_rear);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

visualization_msgs::msg::MarkerArray create_predicted_path_marker_array(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const std::string & ns,
  const int32_t & id, const std_msgs::msg::ColorRGBA & color)
{
  if (predicted_path.path.empty()) {
    return visualization_msgs::msg::MarkerArray{};
  }

  const auto current_time = rclcpp::Clock{RCL_ROS_TIME}.now();
  const auto & path = predicted_path.path;

  visualization_msgs::msg::Marker marker = create_default_marker(
    "map", current_time, ns, id, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(0.1, 0.1, 0.1), color);
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);

  visualization_msgs::msg::MarkerArray marker_array;
  const double half_width = -vehicle_info.vehicle_width_m / 2.0;
  const double base_to_front = vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  for (size_t i = 0; i < path.size(); ++i) {
    marker.id = i + id;
    marker.points.clear();

    const auto & predicted_path_pose = path.at(i);
    create_vehicle_footprint_marker(
      marker, predicted_path_pose, half_width, half_width, base_to_front, base_to_rear);
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, visualization_msgs::msg::Marker & marker,
  visualization_msgs::msg::MarkerArray & marker_array, double z)
{
  for (const auto & ll : lanelets) {
    marker.points.clear();
    for (const auto & p : ll.polygon2d().basicPolygon()) {
      marker.points.push_back(autoware_utils::create_marker_position(p.x(), p.y(), z + 0.5));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    marker_array.markers.push_back(marker);
    ++marker.id;
  }
}

void create_lanelets_marker_array(
  const lanelet::ConstLanelets & lanelets, const std_msgs::msg::ColorRGBA & color,
  visualization_msgs::msg::MarkerArray & marker_array, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray lanelet_marker;
  if (ns.empty()) {
    lanelet_marker = lanelet::visualization::laneletsBoundaryAsMarkerArray(lanelets, color, false);
  } else {
    lanelet_marker = lanelet::visualization::laneletsAsTriangleMarkerArray(ns, lanelets, color);
  }
  marker_array.markers.insert(
    marker_array.markers.end(), lanelet_marker.markers.begin(), lanelet_marker.markers.end());
}

}  // namespace autoware::experimental::marker_utils
